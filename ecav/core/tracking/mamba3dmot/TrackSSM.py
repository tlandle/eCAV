"""
TrackSSM: A General Motion Predictor by State-Space Model
"""
import math 
import torch 
import torch.nn as nn
import torch.nn.functional as F

from einops import rearrange, repeat

from typing import Dict

from mamba_ssm import Mamba  # directly use Mamba
from mamba_ssm.ops.selective_scan_interface import selective_scan_fn, mamba_inner_fn
from mamba_ssm.ops.triton.selective_state_update import selective_state_update

class MambaEncoder(nn.Module):
    '''
    Mamba encoder module
    '''

    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs

        if self.cfgs['cls_token']:
            self.cls_token = nn.Parameter(torch.randn(size=(1, 1, self.cfgs['d_m'])))
        else:
            self.cls_token = None 

        self.proj_embedding_layer = nn.Linear(in_features=8, 
                                              out_features=self.cfgs['d_m'], 
                                              )

        self.encoder = Mamba(d_model=self.cfgs['d_m'], 
                             d_state=self.cfgs['d_state'], 
                             d_conv=4,  # default in Mamba
                             expand=2, )
        
    def forward(self, x):
        # x: shape (bs, q, 8)
        if self.cfgs['cls_token']:
            x = self.proj_embedding_layer(x)

            bs, q, d = x.shape[0], x.shape[1], x.shape[2]

            cls_tokens = repeat(self.cls_token, '() q d -> b q d', b=bs)
            # concat
            if self.cfgs['cls_token_pos'] == 'head':
                x_ = torch.cat([cls_tokens, x], dim=1)
                cls_token_idx = 0
            else:
                x_ = torch.cat([x, cls_tokens], dim=1)
                cls_token_idx = -1

            x_ = self.encoder(x_)

            return x_[:, cls_token_idx]  # (bs, dm)  the cls token as motion embedding
        else:
            x = self.proj_embedding_layer(x)
            x = self.encoder(x)

            return x[:, -1]  # (bs, dm)  the last traj feature as motion embedding


class PositionEmbeddingSine(nn.Module):
    """
    This is a more standard version of the position embedding, very similar to the one
    used by the Attention is all you need paper, generalized to work on images.
    """

    def __init__(self, num_pos_feats=64, temperature=10000, normalize=False, scale=None):
        super().__init__()
        self.num_pos_feats = num_pos_feats
        self.temperature = temperature
        self.normalize = normalize
        if scale is not None and normalize is False:
            raise ValueError("normalize should be True if scale is passed")
        if scale is None:
            scale = 2 * math.pi
        self.scale = scale

    def forward(self, x, class_token=False):
        x = x.permute(1, 2, 0)

        num_feats = x.shape[1]
        num_pos_feats = num_feats
        mask = torch.zeros(x.shape[0], x.shape[2], device=x.device).to(torch.bool)
        batch = mask.shape[0]
        assert mask is not None
        not_mask = ~mask
        y_embed = not_mask.cumsum(1, dtype=torch.float32)

        dim_t = torch.arange(num_pos_feats, dtype=torch.float32, device=x.device)
        dim_t = self.temperature ** (2 * (dim_t // 2) / num_pos_feats)

        pos_y = y_embed[:, :, None] / dim_t
        pos_y = torch.stack((pos_y[:, :, 0::2].sin(), pos_y[:, :, 1::2].cos()), dim=3).flatten(2)
        return pos_y
    

class FlowDecoderLayer(nn.Module):
    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs

        # positional encoding layer
        self.position_embedding = PositionEmbeddingSine(normalize=True)

        # linear projection layer
        self.d_m = self.cfgs['d_m']  # flow feature dim
        self.d_d = self.cfgs['d_d']  # bbox faeture dim
        self.d_inner = 2 * self.d_d  # obey the setting of mamba

        self.dt_rank = math.ceil(self.d_m / 16) if self.cfgs['dt_rank'] == 'auto' else self.cfgs['d_d']  # traj feature dim
        self.d_state = self.cfgs['d_state']  # state dim

        self.in_proj = nn.Linear(in_features=4, out_features=self.d_inner * 2)  # * 2 for chunk

        # activation func
        self.activation = nn.SiLU()

        # dt
        self.dt_proj = nn.Linear(in_features=self.dt_rank, out_features=self.d_inner)
        dt_init_std = self.dt_rank**-0.5
        nn.init.uniform_(self.dt_proj.weight, -dt_init_std, dt_init_std)

        # Initialize dt bias so that F.softplus(dt_bias) is between dt_min and dt_max
        dt_min = 1e-3
        dt_max = 0.1
        dt_init_floor=1e-4
        dt = torch.exp(
            torch.rand(self.d_inner) * (math.log(dt_max) - math.log(dt_min))
            + math.log(dt_min)
        ).clamp(min=dt_init_floor)
        # Inverse of softplus: https://github.com/pytorch/pytorch/issues/72759
        inv_dt = dt + torch.log(-torch.expm1(-dt))
        with torch.no_grad():
            self.dt_proj.bias.copy_(inv_dt)
        # Our initialization would set all Linear.bias to zero, need to mark this one as _no_reinit
        self.dt_proj.bias._no_reinit = True


        # FlowSSM
        # linear projection of flow embedding to get delta_t, B and C
        self.flow_proj = nn.Linear(in_features=self.d_m, out_features=self.dt_rank + self.d_state * 2, bias=False)

        # A 
        A = repeat(
            torch.arange(1, self.d_state + 1, dtype=torch.float32),
            "n -> d n",
            d=self.d_inner,
        ).contiguous()
        A_log = torch.log(A)  # Keep A_log in fp32
        self.A_log = nn.Parameter(A_log)
        self.A_log._no_weight_decay = True

        # D 
        self.D = nn.Parameter(torch.ones(self.d_inner))  # Keep in fp32
        self.D._no_weight_decay = True

        # out ffn
        self.out_proj = nn.Linear(in_features=self.d_inner, out_features=4)  # to bbox

    def forward(self, bbox, hidden_state, flow_embed, use_fast_path=False):
        '''
        bbox: (bs, 4)
        hidden_state: (bs, d_d, d_state)
        flow_embed: (bs, d_m)
        '''

        # TODO position embedding

        # proj bbox to high dim
        bbox_hdim = self.in_proj(bbox)  # (bs, 2 * d_inner)
        e_i, r_i = bbox_hdim.chunk(2, dim=-1)  # bbox embed and residual

        e_i = self.activation(e_i)        

        # FlowSSM
        A = -torch.exp(self.A_log.float())

        proj_f = self.flow_proj(flow_embed)  # (bs, dt_rank + 2 * d_state)
        dt, B, C = torch.split(proj_f, [self.dt_rank, self.d_state, self.d_state], dim=-1)

        # proj dt from (bs, dt_rank) to (bs, d_inner)
        dt = F.linear(dt, self.dt_proj.weight)

        # if the first step, set hidden state to zeros
        hidden_state = hidden_state if hidden_state is not None else torch.zeros((bbox.shape[0], self.d_inner, self.d_state)).to(bbox.device)

        if use_fast_path:  # inference
            r_i = self.activation(r_i)
            y = selective_state_update(
                hidden_state, e_i, dt, A, B, C, self.D, z=r_i, dt_bias=self.dt_proj.bias, dt_softplus=True
            ) 
        else:  # training
            dt = F.softplus(dt + self.dt_proj.bias.to(dtype=dt.dtype))
            dA = torch.exp(torch.einsum("bd,dn->bdn", dt, A))
            dB = torch.einsum("bd,bn->bdn", dt, B)
            # hidden_state.copy_(hidden_state * dA + rearrange(e_i, "b d -> b d 1") * dB)
            hidden_state = hidden_state * dA + rearrange(e_i, "b d -> b d 1") * dB
            y = torch.einsum("bdn,bn->bd", hidden_state, C)
            y = y + self.D * e_i
            y = y * self.activation(r_i)  

            # detach the hidden state
            hidden_state = hidden_state.detach()

        out = self.out_proj(y)

        return out, hidden_state


class TrackSSM(nn.Module):
    
    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs
        # mamba encoder 
        self.mamba_encoder = MambaEncoder(self.cfgs)

        # flow decoder
        self.flow_decoder_layers = nn.Sequential()

        self.decoder_layer_num = self.cfgs['L']
        for i in range(self.decoder_layer_num):
            self.flow_decoder_layers.append(FlowDecoderLayer(self.cfgs))

        self.loss_func = BBoxLoss(self.cfgs)

    def _gen_pesudo_bbox_label(self, cur_bbox, future_bbox):
        
        if self.decoder_layer_num == 1:
            return [future_bbox]
        
        delta_xc = (future_bbox[:, 0] - cur_bbox[:, 0]) / (self.decoder_layer_num - 1)
        delta_yc = (future_bbox[:, 1] - cur_bbox[:, 1]) / (self.decoder_layer_num - 1)
        delta_w = (future_bbox[:, 2] - cur_bbox[:, 2]) / (self.decoder_layer_num - 1)
        delta_h = (future_bbox[:, 3] - cur_bbox[:, 3]) / (self.decoder_layer_num - 1)

        ret = []
        for i in range(self.decoder_layer_num):
            ret.append(cur_bbox + torch.stack([delta_xc, delta_yc, delta_w, delta_h], dim=-1) * i)

        return ret

    def forward(self, x, future_bbox=None):
        '''
        x: (bs, time_window + 1, 8)
        future_bbox: (bs, 4)
        '''

        is_training = future_bbox is not None

        hist_traj = x  # (bs, time_window, 8)
        cur_bbox = x[:, -1, :4]  # (bs, 4)

        flow_embed = self.mamba_encoder(hist_traj)

        hidden_state = None

        if is_training:
            future_bbox = future_bbox[:, :4]

            loss = 0.0

            pesudo_bbox_label = self._gen_pesudo_bbox_label(cur_bbox, future_bbox)
         
            for i in range(self.decoder_layer_num):
                pred_bbox, hidden_state_update = self.flow_decoder_layers[i](cur_bbox, hidden_state, flow_embed, use_fast_path=False)
                cur_bbox = pred_bbox.detach() 
                hidden_state = hidden_state_update

                loss += self.loss_func(pred_bbox, pesudo_bbox_label[i])

            return loss

        else:
            for i in range(self.decoder_layer_num):
                pred_bbox, hidden_state_update = self.flow_decoder_layers[i](cur_bbox, hidden_state, flow_embed, use_fast_path=True)
                cur_bbox = pred_bbox 
                hidden_state = hidden_state_update

            return pred_bbox

class BBoxLoss(nn.Module):
    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs
        self.lambda1 = self.cfgs['lambda1']
        self.lambda2 = self.cfgs['lambda2']

        self.smooth_loss = nn.SmoothL1Loss()

    def _xywh_to_tlbr(self, bbox):
        bbox[:, 0] -= 0.5 * bbox[:, 2]
        bbox[:, 1] -= 0.5 * bbox[:, 3]

        bbox[:, 2] += bbox[:, 0]
        bbox[:, 3] += bbox[:, 1]

        return bbox

    def _g_iou_loss(self, pred_bbox, future_bbox):
        '''
        pred_bbox: (bs, 4)  xywh    
        future_bbox: (bs, 4)  xywh
        '''
        
        # convert xywh to tlbr
        # pred_bbox = self._xywh_to_tlbr(pred_bbox)
        # future_bbox = self._xywh_to_tlbr(future_bbox)

        inter_x1 = torch.max(pred_bbox[:, 0] - 0.5 * pred_bbox[:, 2], future_bbox[:, 0] - 0.5 * future_bbox[:, 2])
        inter_y1 = torch.max(pred_bbox[:, 1] - 0.5 * pred_bbox[:, 3], future_bbox[:, 1] - 0.5 * future_bbox[:, 3])
        inter_x2 = torch.min(pred_bbox[:, 2] + pred_bbox[:, 0], future_bbox[:, 2] + future_bbox[:, 0])
        inter_y2 = torch.min(pred_bbox[:, 3] + pred_bbox[:, 1], future_bbox[:, 3] + future_bbox[:, 1])  
        
        inter_area = torch.clamp(inter_x2 - inter_x1, min=0) * torch.clamp(inter_y2 - inter_y1, min=0)
        
        pred_bbox_area = pred_bbox[:, 2] * pred_bbox[:, 3]
        future_bbox_area = future_bbox[:, 2] * future_bbox[:, 3]
        
        union_area = pred_bbox_area + future_bbox_area - inter_area
        
        ac_x1 = torch.min(pred_bbox[:, 0] - 0.5 * pred_bbox[:, 2], future_bbox[:, 0] - 0.5 * future_bbox[:, 2])
        ac_y1 = torch.min(pred_bbox[:, 1] - 0.5 * pred_bbox[:, 3], future_bbox[:, 1] - 0.5 * future_bbox[:, 3])
        ac_x2 = torch.max(pred_bbox[:, 2] + pred_bbox[:, 0], future_bbox[:, 2] + future_bbox[:, 0])
        ac_y2 = torch.max(pred_bbox[:, 3] + pred_bbox[:, 1], future_bbox[:, 3] + future_bbox[:, 1])
        
        ac_area = (ac_x2 - ac_x1) * (ac_y2 - ac_y1)
        
        giou = inter_area / union_area - (ac_area - union_area) / ac_area
        
        giou_loss = 1 - giou
        
        return giou_loss

    def forward(self, pred_bbox, future_bbox):
        '''
        pred_bbox: (bs, 4)
        future_bbox: (bs, 4)
        '''

        loss = self.smooth_loss(pred_bbox, future_bbox)

        if self.lambda2 > 0:
            loss += self.lambda2 * self._g_iou_loss(pred_bbox, future_bbox).mean(dim=0)

        return loss

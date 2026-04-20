"""
MambaTrack3D: MambaTrack (Xiao et al., ACM MM 2024) adapted for 3D MOT.

Original: 2D bounding boxes [cx, cy, w, h] (4-dim)
Adapted:  3D bounding boxes [x, y, z, l, w, h, yaw] (7-dim)

The Bi-Mamba architecture is unchanged. Only input/output dimensions
and prediction head are adapted for 3D.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

from typing import Dict

from mamba_ssm import Mamba

# Configurable box dimension: 4 for 2D, 7 for 3D
DEFAULT_BOX_DIM = 7

class BiMambaBlock(nn.Module):
    '''
    Bi-Mamba Block in MTP
    '''
    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs

        self.forward_mamba = Mamba(d_model=self.cfgs['d_m'], 
                                   d_state=self.cfgs['d_state'], 
                                   d_conv=4,  # default in Mamba
                                   expand=2, )
        
        self.backward_mamba = Mamba(d_model=self.cfgs['d_m'], 
                                   d_state=self.cfgs['d_state'], 
                                   d_conv=4,  # default in Mamba
                                   expand=2, )
        
        self.layer_norm = nn.LayerNorm((self.cfgs['d_m']), )

        self.proj_layer = nn.Sequential(
            nn.Linear(in_features=self.cfgs['d_m'], out_features=self.cfgs['d_m'] * 2),
            nn.LeakyReLU(),
            nn.Linear(in_features=self.cfgs['d_m'] * 2, out_features=self.cfgs['d_m']),
        )

    def forward(self, x):
        # x: shape (bs, q, d_m)
        
        # reverse the temporal casual 
        x_back = torch.flip(x, dims=[-2])

        forward_mamba_out = self.forward_mamba(x)
        backward_mamba_out = self.backward_mamba(x_back)

        mamba_out = forward_mamba_out + backward_mamba_out

        out = mamba_out + self.proj_layer(self.layer_norm(mamba_out))

        return out

class MambaTrack(nn.Module):
    '''
    Mamba Motion Predictor in paper (MTP)
    '''
    def __init__(self, cfgs) -> None:
        super().__init__()

        self.cfgs = cfgs

        self.box_dim = self.cfgs.get('box_dim', DEFAULT_BOX_DIM)

        # projection layer, one-layer mlp
        self.proj_embedding_layer = nn.Linear(in_features=self.box_dim,
                                              out_features=self.cfgs['d_m'],
                                              )
        
        # L-layers Bi-Mamba Block
        self.bi_mamba_blocks = nn.Sequential()
        for i in range(self.cfgs['L']):
            self.bi_mamba_blocks.append(BiMambaBlock(self.cfgs))

        # prediction head, avg pooling + 2-layer mlp
        # self.avg_pool = nn.AdaptiveAvgPool2d(output_size=self.cfgs['avg_pool_out_dim'])
        self.pred_head = nn.Sequential(nn.AdaptiveAvgPool2d(output_size=self.cfgs['avg_pool_out_dim']), 
                                        nn.Linear(self.cfgs['avg_pool_out_dim'][1], self.cfgs['pred_head_dims'][0]), 
                                        nn.LeakyReLU(), 
                                        nn.Linear(self.cfgs['pred_head_dims'][0], self.cfgs['pred_head_dims'][1]), 
                                        )
        
        self.loss_func = BboxLoss(self.cfgs)
        
    def forward(self, x, label=None):

        x = self.proj_embedding_layer(x)

        y = self.bi_mamba_blocks(x)

        out = self.pred_head(y)

        if label is not None:
            loss = self.loss_func(out, label)
            return loss
        else:
            return out

    
class BboxLoss(nn.Module):
    '''Smooth L1 loss for training'''
    def __init__(self, cfgs):
        super().__init__()
        self.cfgs = cfgs

        self.loss_func = nn.SmoothL1Loss()

        self.const_term = 1

    def forward(self, pred, label):
        '''
        pred: shape (bs, 1, 4)  the predicted offset
        label: shape (bs, 4)  the gt offset
        '''

        if pred.ndim == 3:
            pred = pred.squeeze(dim=1)

        loss = self.loss_func.forward(pred, label)
        return loss
    

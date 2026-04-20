#!/usr/bin/env python3
import sys,os,time,torch,numpy as np
sys.path.insert(0,'ecav/worldfusion');sys.path.insert(0,'.')
from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
import opencood.tools.train_utils as tu
import torch.nn.functional as F
md='ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10'
h=load_yaml(os.path.join(md,'config.yaml'))
m=PointPillarWorldFusion(h['model']['args']).to('cuda:0').eval()
_,m=tu.load_model(md,m,epoch=50)
for N in [4,8,16,24,32]:
    sf=torch.randn(N,64,200,200,device='cuda:0')
    rl=torch.tensor([N],dtype=torch.int64,device='cuda:0')
    pw=torch.eye(4,device='cuda:0').reshape(1,1,1,4,4).expand(1,N,N,4,4).contiguous()
    bd=m.sensor.backbone({'spatial_features':sf});bev=bd['spatial_features_2d']
    if m.sensor.shrink_flag:bev=m.sensor.shrink_conv(bev)
    psm=m.cls_head(bev)
    H1=int(m.sensor.nx[1].item())//2;W1=int(m.sensor.nx[0].item())//2
    if psm.shape[-2:]!=(H1,W1):psm=F.interpolate(psm,size=(H1,W1),mode='nearest')
    th=torch.ones_like(psm[:,:1])*0.5
    shrk=m.sensor.shrink_conv if m.sensor.shrink_flag else None
    hds=[shrk,m.cls_head,m.reg_head]
    for _ in range(3):
        with torch.no_grad():m.fusion(sf,psm,rl,pw,backbone=m.sensor.backbone,heads=hds)
    tw=[]
    for _ in range(5):
        torch.cuda.synchronize();t0=time.perf_counter()
        with torch.no_grad():m.fusion(sf,psm,rl,pw,backbone=m.sensor.backbone,heads=hds)
        torch.cuda.synchronize();tw.append((time.perf_counter()-t0)*1000)
    tm=[]
    for _ in range(5):
        torch.cuda.synchronize();t0=time.perf_counter()
        with torch.no_grad():_=sf.max(dim=0,keepdim=True)[0]
        torch.cuda.synchronize();tm.append((time.perf_counter()-t0)*1000)
    # AttenComm timing
    from opencood.models.bm2cp_modules.attentioncomm import AttenComm
    if N == 4:  # only init once
        ac_cfg = {
            'voxel_size': [0.4, 0.4, 20],
            'downsample_rate': 1,
            'multi_scale': True,
            'layer_nums': h['model']['args']['fusion_args']['layer_nums'],
            'num_filters': h['model']['args']['fusion_args']['num_filters'],
            'agg_operator': {'feature_dim': 256},
        }
        attencomm = AttenComm(ac_cfg).to('cuda:0').eval()
    for _ in range(3):
        with torch.no_grad():attencomm(sf,psm,th,rl,pw,backbone=m.sensor.backbone,heads=hds)
    ta=[]
    for _ in range(5):
        torch.cuda.synchronize();t0=time.perf_counter()
        with torch.no_grad():attencomm(sf,psm,th,rl,pw,backbone=m.sensor.backbone,heads=hds)
        torch.cuda.synchronize();ta.append((time.perf_counter()-t0)*1000)
    print(f'N={N:3d}: W2C={np.mean(tw):.1f}ms  AttenComm={np.mean(ta):.1f}ms  Maxout={np.mean(tm):.1f}ms')

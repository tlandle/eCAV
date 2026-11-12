import sys, pandas as pd, numpy as np
t=pd.read_csv(sys.argv[1]+'/evaluation_output/per_tick_age.csv')
g=t.groupby('tick').first()[['conflict_gt_x','conflict_gt_y','decision','ego_speed_mps']]
g['gt_v']=np.hypot(g.conflict_gt_x.diff(),g.conflict_gt_y.diff())/0.05
c=t[t.is_conflict_actor==1].groupby('tick').agg(obs_v=('obs_speed_mps','first'),age=('age_at_use_ms','min'),ox=('obs_x','first'),oy=('obs_y','first'),tid=('track_id','first'))
r=g.join(c)
mv=r[r.gt_v>2]
print('ticks',len(r),'conflict actor moving ticks',len(mv),'tracked on',mv.obs_v.notna().sum(),'track ids',sorted(set(mv.tid.dropna().astype(int))))
m=mv.dropna(subset=['obs_v'])
print('speed error obs-gt: mean %.1f  p5 %.1f  p50 %.1f  p95 %.1f ; ratio obs/gt p50 %.2f ; published at rest (<0.5) on %d of %d'%((m.obs_v-m.gt_v).mean(),(m.obs_v-m.gt_v).quantile(.05),(m.obs_v-m.gt_v).median(),(m.obs_v-m.gt_v).quantile(.95),(m.obs_v/m.gt_v).median(),(m.obs_v<0.5).sum(),len(m)))
step=max(1,len(mv)//24)
print(mv[['gt_v','obs_v','age','tid','decision']].iloc[::step].round(1).to_string())

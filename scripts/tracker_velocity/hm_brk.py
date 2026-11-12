import sys, glob, re, pandas as pd, numpy as np
root='experiment_results/highway_merge_edge_late_fusion/%s/speed_spd_97/baseline_default'%sys.argv[1]
mgr,anch=sys.argv[2],sys.argv[3]
print('lat run | lane change starts | first held brake (ticks after start, s) | brake ticks before start | min gap m | obs speed sd m/s, |vy-like drift| | outcome')
for d in sorted(glob.glob(f'{root}/mgr_{mgr}/anchoring_{anch}/lat_*closing_speed_mps-10_*/run_*'), key=lambda p:(int(re.search(r'lat_(\d+)',p).group(1)),p)):
    lat=int(re.search(r'lat_(\d+)',d).group(1)); run=d.split('_')[-1]
    t=pd.read_csv(d+'/evaluation_output/per_tick_age.csv')
    g=t.groupby('tick').first()
    c=t[t.is_conflict_actor==1].groupby('tick').first()
    off=g.conflict_gt_y-0.0041*g.conflict_gt_x
    y0=off.iloc[20:60].median()
    mv=[k for k in g.index if k>60 and (off[k]-y0)<-0.10 and (off.reindex(range(k,k+5)).dropna()-y0<-0.10).all()]
    if not mv: print(lat,run,'no lane change'); continue
    s=mv[0]
    br=(g.decision=='brake')&(g.ctrl_brake>=0.75)
    held=[k for k in g.index if k>=40 and all(br.reindex(range(k,k+5),fill_value=False))]
    fb=held[0] if held else None
    pre=int((br.loc[40:s-1]).sum())
    gap=(g.conflict_gt_x-g.ego_x-4.9).loc[s:].min()
    cv=c.obs_speed_mps.loc[50:s]
    out=open(d+'/runner_stdout.txt',errors='ignore').read()
    coll='collision' if re.search(r'vehicle_manager.*Collision',out) else 'no collision'
    print(f"{lat:4d} {run} | tick {s} | {fb} ({'%+d'%(fb-s) if fb else 'none'}, {'%.2f'%((fb-s)*0.05) if fb else '-'} s) | {pre} | {gap:5.1f} | {cv.std():.1f} (mean {cv.mean():.1f}) | {coll}")

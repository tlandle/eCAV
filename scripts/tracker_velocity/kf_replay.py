import sys, pandas as pd, numpy as np
I=pd.read_csv(sys.argv[1]); mode=sys.argv[2] if len(sys.argv)>2 else 'first'
DT=0.05
def per_tick(I,mode):
    out={}
    for t,g in I.groupby('tick'):
        if mode=='first': r=g.iloc[0]; out[t]=(r.x,r.y)
        elif mode=='mean': out[t]=(g.x.mean(),g.y.mean())
        elif mode=='front': r=g.loc[g.x.idxmax()]; out[t]=(r.x,r.y)
    return out
gt=I.groupby('tick').first()[['conflict_gt_x','conflict_gt_y']]
gv=gt.diff()/DT
def run(meas,R,Qp,Qv,ticks):
    # two independent 2-state filters (x,vx),(y,vy), dt=1 step
    F=np.array([[1,1],[0,1.]]); H=np.array([[1,0.]]); Q=np.diag([Qp,Qv])
    X=None; res={}
    for t in ticks:
        if X is None:
            if t in meas:
                X=[np.array([meas[t][0],0.]),np.array([meas[t][1],0.])]; P=[np.diag([10.,10000.]) for _ in range(2)]
            continue
        for a in range(2):
            X[a]=F@X[a]; P[a]=F@P[a]@F.T+Q
            if t in meas:
                S_=H@P[a]@H.T+R; K=(P[a]@H.T)/S_
                X[a]=X[a]+(K*(meas[t][a]-X[a][0])).ravel(); P[a]=(np.eye(2)-K@H)@P[a]
        res[t]=(X[0][0],X[1][0],X[0][1]/DT,X[1][1]/DT)
    return pd.DataFrame(res,index=['x','y','vx','vy']).T
ticks=range(int(I.tick.min()),int(I.tick.max())+1)
meas=per_tick(I,mode)
cfgs={'current R3 Qp1 Qv.01':(3,1,0.01),'R1 Qp.05 Qv.02':(1,0.05,0.02),'R.5 Qp.02 Qv.01':(0.5,0.02,0.01),'R1 Qp0 Qv.005':(1,0.0,0.005),'R.25 Qp.01 Qv.01':(0.25,0.01,0.01)}
lo,hi=int(sys.argv[3]),int(sys.argv[4]); lc0=int(sys.argv[5])
print('gt speed mean %.1f lateral v at lane change (m/s):'%gv.conflict_gt_x.loc[lo:hi].mean(), gv.conflict_gt_y.loc[lc0:lc0+25].round(2).tolist()[::5])
for k,(R,Qp,Qv) in cfgs.items():
    r=run(meas,R,Qp,Qv,ticks).join(gt).join(gv,rsuffix='_v')
    w=r.loc[lo:hi]; sp=np.hypot(w.vx,w.vy)
    lc=r.loc[lc0:lc0+30]
    # lateral lag: first tick where |vy| > 0.5*|gt vy| sustained
    lag=next((t-lc0 for t in lc.index if abs(lc.vy[t])>0.5*abs(lc.conflict_gt_y_v[t]) and abs(lc.conflict_gt_y_v[t])>0.5),None)
    print(f"{k:22s} cruise speed mean {sp.mean():5.1f} sd {sp.std():4.1f} min {sp.min():5.1f} max {sp.max():5.1f} | cruise |vy| p95 {w.vy.abs().quantile(.95):4.2f} m/s | lane change: vy at +10/+20/+30 ticks {lc.vy.iloc[10]:5.2f} {lc.vy.iloc[20]:5.2f} {lc.vy.iloc[30]:5.2f} (gt {lc.conflict_gt_y_v.iloc[10]:5.2f} {lc.conflict_gt_y_v.iloc[20]:5.2f} {lc.conflict_gt_y_v.iloc[30]:5.2f}) lag50 {lag}")

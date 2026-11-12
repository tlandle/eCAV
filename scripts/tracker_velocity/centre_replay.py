import sys, pandas as pd, numpy as np
I=pd.read_csv(sys.argv[1]); lo,hi,lc0=map(int,sys.argv[2:5]); DT=0.05
gt=I.groupby('tick').first()[['conflict_gt_x','conflict_gt_y']]
T=np.array(gt.index); gv=pd.DataFrame({'gvx':np.gradient(gt.conflict_gt_x.values,T)/DT,'gvy':np.gradient(gt.conflict_gt_y.values,T)/DT},index=gt.index)
def meas(pick):
    m={}
    for t,g in I.groupby('tick'):
        if pick=='first': r=g.iloc[0]; m[t]=((r.x0+r.x1)/2,(r.y0+r.y1)/2, r.x, r.y)
        else: m[t]=(((g.x0+g.x1)/2).mean(),((g.y0+g.y1)/2).mean(), g.x.mean(), g.y.mean())
    return m
def ls(hist,window,col):
    f1=hist[-1][0]; pts=[(f,v[col]) for f,v in hist if f>=f1-window]
    if len(pts)<4: return 0.0
    fr=np.array([p[0] for p in pts],float); v=np.array([p[1] for p in pts]); fr-=fr.mean()
    return float(((v-v.mean())*fr).sum()/(fr*fr).sum())
def run(pick,wx,wy,cols):
    m=meas(pick); hist=[]; out={}
    for t in range(min(m),max(m)+1):
        if t in m: hist.append((t,m[t])); hist=hist[-45:]
        if len(hist)<10: continue
        out[t]=(ls(hist,wx,cols[0])/DT, ls(hist,wy,cols[1])/DT)
    return pd.DataFrame(out,index=['vx','vy']).T.join(gv)
for pick in ['first','mean']:
  for cols,cn in [((2,3),'box centre'),((0,1),'edge midpoint')]:
    for wx,wy in [(10,10),(10,20),(20,20),(10,30)]:
        r=run(pick,wx,wy,cols); w=r.loc[lo:hi]; sp=np.hypot(w.vx,w.vy); lc=r.loc[lc0:lc0+40]
        lag=next((t-lc0 for t in lc.index if abs(lc.gvy[t])>0.5 and lc.vy[t]*lc.gvy[t]>0 and abs(lc.vy[t])>0.5*abs(lc.gvy[t])),None)
        print(f"{pick:5s} {cn:13s} wx{wx} wy{wy}: speed mean {sp.mean():5.1f} (gt {np.hypot(w.gvx,w.gvy).mean():4.1f}) sd {sp.std():4.1f} min {sp.min():5.1f} max {sp.max():5.1f} |vy| p95 {w.vy.abs().quantile(.95):4.2f} max {w.vy.abs().max():4.2f} | lane change vy +6/+10/+16/+24 {lc.vy.iloc[6]:5.2f} {lc.vy.iloc[10]:5.2f} {lc.vy.iloc[16]:5.2f} {lc.vy.iloc[24]:5.2f} lag50 {lag}")

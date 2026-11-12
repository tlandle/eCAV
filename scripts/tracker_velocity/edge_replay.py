import sys, pandas as pd, numpy as np
I=pd.read_csv(sys.argv[1]); lo,hi,lc0=map(int,sys.argv[2:5]); pick=sys.argv[5] if len(sys.argv)>5 else 'first'
DT=0.05
gt=I.groupby('tick').first()[['conflict_gt_x','conflict_gt_y']]
T=np.array(gt.index); gvx=np.gradient(gt.conflict_gt_x.values,T)/DT; gvy=np.gradient(gt.conflict_gt_y.values,T)/DT
gv=pd.DataFrame({'gvx':gvx,'gvy':gvy},index=gt.index)
meas={}
for t,g in I.groupby('tick'):
    r=g.iloc[0] if pick=='first' else g.loc[g.conf.idxmax()]
    meas[t]=(r.x0,r.x1,r.y0,r.y1)
def slopes(hist,f1,window,min_samples):
    pts=[(f,v) for f,v in hist if f>=f1-window]
    if len(pts)<min_samples or pts[-1][0]-pts[0][0]<2: return None
    fr=np.array([p[0] for p in pts],float); vals=np.array([p[1] for p in pts],float); fr-=fr.mean(); den=(fr*fr).sum()
    return ((vals-vals.mean(0))*fr[:,None]).sum(0)/den
def slower(a,b,vr=2.0*DT):
    if a*b<=0: return 0.0
    if abs(a)>=vr and abs(b)>=vr and abs(a-b)<=vr: return 0.5*(a+b)
    return a if abs(a)<abs(b) else b
def est(mode,win=10,latwin=10):
    hist=[]; out={}
    for t in range(min(meas),max(meas)+1):
        if t in meas:
            hist.append((t,meas[t])); hist=hist[-41:]
        if len(hist)<10: continue
        s=slopes(hist,hist[-1][0],win,4)
        if s is None: s=slopes(hist,hist[-1][0],2*win,3)
        if s is None: out[t]=(0,0); continue
        if mode=='slower':
            vx=slower(s[0],s[1]); vy=slower(s[2],s[3])
        else:
            # rigid: on the dominant axis both edges move the same way and the slower is at least half the faster
            ax=(0,1) if abs(s[0]+s[1])>=abs(s[2]+s[3]) else (2,3)
            a,b=s[ax[0]],s[ax[1]]
            rigid = a*b>0 and min(abs(a),abs(b))>=2.0*DT and min(abs(a),abs(b))>=0.5*max(abs(a),abs(b))
            if rigid:
                sl=slopes(hist,hist[-1][0],latwin,4) if latwin!=win else s
                if sl is None: sl=s
                cx=(0.5*(s[0]+s[1]),0.5*(s[2]+s[3])); cl=(0.5*(sl[0]+sl[1]),0.5*(sl[2]+sl[3]))
                vx,vy=(cx[0],cl[1]) if ax==(0,1) else (cl[0],cx[1])
            else:
                vx=slower(s[0],s[1]); vy=slower(s[2],s[3])
        if np.hypot(vx,vy)/DT<2.0: vx=vy=0.0
        out[t]=(vx/DT,vy/DT)
    return pd.DataFrame(out,index=['vx','vy']).T.join(gv)
for name,kw in [('slower (current)',dict(mode='slower')),('rigid w10',dict(mode='rigid')),('rigid w10 lateral w20',dict(mode='rigid',latwin=20)),('rigid w10 lateral w14',dict(mode='rigid',latwin=14))]:
    r=est(**kw); w=r.loc[lo:hi]; sp=np.hypot(w.vx,w.vy); lc=r.loc[lc0:lc0+40]
    lag=next((t-lc0 for t in lc.index if abs(lc.gvy[t])>0.5 and lc.vy[t]*lc.gvy[t]>0 and abs(lc.vy[t])>0.5*abs(lc.gvy[t])),None)
    print(f"{name:24s} cruise: speed mean {sp.mean():5.1f} (gt {np.hypot(w.gvx,w.gvy).mean():4.1f}) sd {sp.std():4.1f} min {sp.min():5.1f} zero-frames {(sp<0.1).sum():2d} |vy| p95 {w.vy.abs().quantile(.95):4.2f} max {w.vy.abs().max():4.2f} | lane change vy at +6/+10/+16/+24: {lc.vy.iloc[6]:5.2f} {lc.vy.iloc[10]:5.2f} {lc.vy.iloc[16]:5.2f} {lc.vy.iloc[24]:5.2f} (gt {lc.gvy.iloc[6]:5.2f} {lc.gvy.iloc[10]:5.2f} {lc.gvy.iloc[16]:5.2f} {lc.gvy.iloc[24]:5.2f}) lag50 {lag} ticks")

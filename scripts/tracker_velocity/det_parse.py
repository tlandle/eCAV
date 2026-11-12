import re, sys, glob, pandas as pd, numpy as np
d=sys.argv[1]
rx=re.compile(r"\[DET\] tick=(\d+) src=(\S+) x=([-\d.]+) y=([-\d.]+) conf=([\d.]+) cid=(-?\d+) hwl=([\d.,]+) e=([-\d.,]+)")
rows=[]
for l in open(d+'/runner_stdout.txt',errors='ignore'):
    m=rx.search(l)
    if m:
        e=[float(v) for v in m.group(8).split(',')]
        rows.append(dict(tick=int(m.group(1)),src=m.group(2),x=float(m.group(3)),y=float(m.group(4)),conf=float(m.group(5)),cid=int(m.group(6)),hwl=m.group(7),x0=e[0],x1=e[1],y0=e[2],y1=e[3]))
D=pd.DataFrame(rows)
t=pd.read_csv(d+'/evaluation_output/per_tick_age.csv').groupby('tick').first()[['conflict_gt_x','conflict_gt_y','ego_x','ego_y']]
D=D.join(t,on='tick')
D['dx']=D.x-D.conflict_gt_x; D['dy']=D.y-D.conflict_gt_y
print(D.src.value_counts().to_dict())
I=D[(D.src!='beacon')&(D.dx.abs()<6)&(D.dy.abs()<3)]
pd.set_option('display.width',220); pd.set_option('display.max_rows',400)
print(I.groupby('tick').size().describe()[['count','mean','max']].to_dict(), 'ticks total', t.index.max())
print(I[(I.tick>=int(sys.argv[2]))&(I.tick<=int(sys.argv[3]))][['tick','src','x','y','conf','hwl','x0','x1','y0','y1','conflict_gt_x','conflict_gt_y','dx','dy']].round(2).to_string(index=False))
I.to_csv(sys.argv[4] if len(sys.argv)>4 else '/dev/null',index=False)

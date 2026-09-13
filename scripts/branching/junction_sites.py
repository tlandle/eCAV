import xml.etree.ElementTree as ET, math, sys
M="/home/atlas/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive"
def norm(a): return (a+math.pi)%(2*math.pi)-math.pi
def load(town):
    root=ET.parse(f"{M}/{town}.xodr").getroot()
    roads={r.get("id"):r for r in root.findall("road")}
    jr={rid for rid,r in roads.items() if r.get("junction","-1")!="-1"}
    return root,roads,jr
def links(roads,rid):
    lk=roads[rid].find("link"); out={}
    if lk is not None:
        for tag in ("predecessor","successor"):
            e=lk.find(tag)
            if e is not None: out[tag]=(e.get("elementType"),e.get("elementId"))
    return out
def endpt(g):
    x0,y0,h=float(g.get("x")),float(g.get("y")),float(g.get("hdg")); L=float(g.get("length"))
    arc=g.find("arc"); c=float(arc.get("curvature")) if arc is not None else 0.0
    if c==0: return x0+L*math.cos(h), y0+L*math.sin(h), h
    he=h+c*L; return x0+(math.sin(he)-math.sin(h))/c, y0-(math.cos(he)-math.cos(h))/c, he
def arms(town,jid):
    root,roads,jr=load(town)
    j=[x for x in root.findall("junction") if x.get("id")==jid][0]
    ext=set()
    for c in j.findall("connection"):
        inc=c.get("incomingRoad")
        if inc not in jr: ext.add(inc)
        for t,(ty,i) in links(roads,c.get("connectingRoad")).items():
            if ty=="road" and i not in jr: ext.add(i)
    out=[]
    for rid in sorted(ext,key=int):
        r=roads[rid]; geos=r.findall("planView/geometry"); L=float(r.get("length"))
        lk=links(roads,rid)
        if lk.get("predecessor")==("junction",jid):
            g=geos[0]; jx,jy,h=float(g.get("x")),float(g.get("y")),float(g.get("hdg")); hdg=h
        else:
            jx,jy,he=endpt(geos[-1]); hdg=norm(he+math.pi)
        widths=[float(w.get("a")) for w in r.findall("lanes/laneSection/*/lane/width")]
        driving=len([l for l in r.findall("lanes/laneSection/*/lane") if l.get("type")=="driving"])
        out.append((rid,L,math.degrees(norm(hdg)),jx,jy,driving,sum(widths)))
    return out
def show(town,jid):
    print(f"\n== {town} junction {jid} ==")
    for rid,L,deg,x,y,drv,wsum in arms(town,jid):
        print(f"  arm road {rid:>4s}: at ({x:7.1f},{y:7.1f}) outward hdg {deg:6.1f} deg, length {L:6.1f} m, driving lanes {drv}, total width {wsum:.1f} m")
# k=4 site
show("Town03","861")
# k=3 candidates: Town05 4-arm junctions, print arm lengths (want all 4 arms >= 40 m, symmetric)
root,roads,jr=load("Town05")
print("\n== Town05 4-arm junctions (arm lengths) ==")
for j in root.findall("junction"):
    a=arms("Town05",j.get("id"))
    if len(a)==4:
        print(f"  jid {j.get('id'):>4s} at ({a[0][3]:.0f},{a[0][4]:.0f}) arm lengths {[round(x[1]) for x in a]} hdgs {[round(x[2]) for x in a]}")
root,roads,jr=load("Town01")
print("\n== Town01 3-arm junctions (arm lengths) ==")
for j in root.findall("junction"):
    a=arms("Town01",j.get("id"))
    if len(a)==3:
        print(f"  jid {j.get('id'):>4s} at ({a[0][3]:.0f},{a[0][4]:.0f}) arm lengths {[round(x[1]) for x in a]} hdgs {[round(x[2]) for x in a]}")

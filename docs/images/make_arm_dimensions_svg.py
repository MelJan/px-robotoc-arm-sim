import math

S = 10.0  # px per cm
W, H = 1000, 960
out = []
def add(s): out.append(s)

INK = "#1f2a44"; DIM = "#0a7d6b"; RED = "#c0392b"; LIGHT = "#dfe6ee"; GREY = "#8a94a6"
FONT = "font-family='Helvetica, Arial, sans-serif'"

def line(x1,y1,x2,y2,stroke=INK,w=2,dash=None):
    d = f" stroke-dasharray='{dash}'" if dash else ""
    add(f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' stroke='{stroke}' stroke-width='{w}' stroke-linecap='round'{d}/>")
def rect(x,y,w,h,fill=LIGHT,stroke=INK,sw=2):
    add(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' fill='{fill}' stroke='{stroke}' stroke-width='{sw}'/>")
def circle(cx,cy,r,fill="white",stroke=INK,sw=2):
    add(f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{r:.1f}' fill='{fill}' stroke='{stroke}' stroke-width='{sw}'/>")
def text(x,y,s,size=13,fill=INK,anchor="middle",weight="normal",rot=0):
    r = f" transform='rotate({rot} {x:.1f} {y:.1f})'" if rot else ""
    add(f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' fill='{fill}' text-anchor='{anchor}' font-weight='{weight}' {FONT}{r}>{s}</text>")

def dim(x1,y1,x2,y2,label,off=18,color=DIM,flip=False):
    """dimension line parallel to (x1,y1)-(x2,y2), offset perpendicular."""
    dx,dy = x2-x1, y2-y1; L = math.hypot(dx,dy); ux,uy = dx/L, dy/L
    nx,ny = -uy, ux
    if flip: nx,ny = -nx,-ny
    ax,ay = x1+nx*off, y1+ny*off; bx,by = x2+nx*off, y2+ny*off
    line(x1,y1,ax+nx*4,ay+ny*4,color,1); line(x2,y2,bx+nx*4,by+ny*4,color,1)
    line(ax,ay,bx,by,color,1.5)
    for (px,py) in ((ax,ay),(bx,by)):
        line(px-nx*5,py-ny*5,px+nx*5,py+ny*5,color,1.5)
    ang = math.degrees(math.atan2(dy,dx))
    if ang>90 or ang<-90: ang += 180
    if abs(abs(ang)-90) < 20:
        side = 1 if nx>=0 else -1
        text((ax+bx)/2+side*8,(ay+by)/2+4,label,12,color,"start" if side>0 else "end","bold")
    else:
        mx,my = (ax+bx)/2+nx*12,(ay+by)/2+ny*12
        text(mx,my+4,label,12,color,"middle","bold",ang)

add(f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {W} {H}' width='{W}' height='{H}'>")
add(f"<rect width='{W}' height='{H}' fill='white'/>")
text(30,34,"Robot arm dimensions (from whiteboard sketch, docs/images/Image (1).jpeg)",18,INK,"start","bold")
text(30,54,"All values in cm. Scale 1 cm = 10 px. Green = measured on the whiteboard, grey = not measured (drawn for shape only).",12,GREY,"start")

# ---------------- Panel 1: side view ----------------
ox, oy = 150, 420   # origin = S1 axis at mount-box top surface; y grows downward
def P(x,z): return (ox + x*S, oy - z*S)
text(30,90,"1. Side view, kinematic chain",15,INK,"start","bold")

# base plate (30 wide along the view), thickness unknown -> 1.0 drawn
bx0,_ = P(-9.5,0)
rect(bx0, oy+1.5*S, 30*S, 1.0*S, LIGHT, GREY)
text(bx0+30*S, oy+1.5*S+1.0*S+18, "base plate 30 x 38 (thickness not measured)", 11, GREY, "end")
# mount box: footprint unknown, drawn 12 wide, 1.5 high
rect(ox-6*S, oy, 12*S, 1.5*S, "#eef2f6", INK)
dim(ox+6*S, oy+1.5*S, ox+6*S, oy, "1,5", 22, RED)
# column up to shoulder
colw = 5.0
rect(ox-colw/2*S, oy-9.6*S, colw*S, 9.6*S, "#eef2f6", INK)
text(ox-colw/2*S-8, oy-2.5*S, "S1 base rotation", 11, GREY, "end"); text(ox-colw/2*S-8, oy-2.5*S+13, "(vertical axis)", 11, GREY, "end")
line(ox, oy+1.5*S+1.0*S+4, ox, oy-9.6*S, GREY, 1, "6,4")

dim(ox+colw/2*S, oy, ox+colw/2*S, oy-9.6*S, "9,6", 14)

# pose
def deg(a): return math.radians(a)
S2 = (0,9.6)
a1,a2,a3 = 70,-20,-50
S3 = (S2[0]+10.5*math.cos(deg(a1)), S2[1]+10.5*math.sin(deg(a1)))
S4 = (S3[0]+12.5*math.cos(deg(a2)), S3[1]+12.5*math.sin(deg(a2)))
S5 = (S4[0]+6.7*math.cos(deg(a3)),  S4[1]+6.7*math.sin(deg(a3)))
TIP= (S4[0]+17.5*math.cos(deg(a3)), S4[1]+17.5*math.sin(deg(a3)))

def link(a,b,w=10):
    (x1,y1),(x2,y2) = P(*a),P(*b)
    line(x1,y1,x2,y2,"#9fb3c8",w+4); line(x1,y1,x2,y2,"#3b5a84",w)
link(S2,S3,12); link(S3,S4,10); link(S4,S5,8)
# gripper body from S5 to tip (two fingers)
(gx,gy)=P(*S5); (tx,ty)=P(*TIP)
ux,uy = (tx-gx),(ty-gy); L=math.hypot(ux,uy); ux/=L; uy/=L; nx,ny=-uy,ux
for s in (-1,1):
    line(gx+nx*s*2.5*S, gy+ny*s*2.5*S, tx+nx*s*2.15*S, ty+ny*s*2.15*S, "#3b5a84", 6)
    line(gx, gy, gx+nx*s*2.5*S, gy+ny*s*2.5*S, "#3b5a84", 6)
for j,lab in ((S2,"S2 shoulder"),(S3,"S3 elbow"),(S4,"S4 wrist flexion"),(S5,"S5 wrist rotation")):
    x,y=P(*j); circle(x,y,7,"white",INK,2.5); circle(x,y,2,INK,INK,1)
x,y=P(*S2); text(x-34,y+4,"S2 shoulder",12,INK,"end","bold")
x,y=P(*S3); text(x-24,y-8,"S3 elbow",12,INK,"end","bold")
x,y=P(*S4); text(x+12,y-10,"S4 wrist flexion",12,INK,"start","bold")
x,y=P(*S5); text(x+62,y+4,"S5 wrist rotation",12,INK,"start","bold")
x,y=P(*TIP); text(x+6,y+18,"S6 gripper tip",12,INK,"start","bold")

# dims on links
dim(*P(*S2),*P(*S3),"10,5",30,DIM,True)
dim(*P(*S3),*P(*S4),"12,5",30,DIM,True)
dim(*P(*S4),*P(*S5),"6,7",-42,DIM,True)
dim(*P(*S4),*P(*TIP),"17,5",50,DIM,False)

# ---------------- Panel 2: top view ----------------
tx0, ty0 = 640, 110
text(tx0,ty0-12,"2. Top view of base plate",15,INK,"start","bold")
rect(tx0, ty0, 30*S, 38*S, LIGHT, INK)
rect(tx0+9*S, ty0, 12*S, 6*S, "#eef2f6", INK)   # mount box footprint (not measured)
cx,cy = tx0+15*S, ty0+9.5*S
circle(cx,cy,6,"white",INK,2); circle(cx,cy,2,INK,INK,1)
text(cx+10,cy+4,"S1 axis",11,INK,"start")
dim(cx,cy,cx,ty0,"9,5",-80,RED,True)
dim(tx0,ty0+38*S,tx0+30*S,ty0+38*S,"30",-22,DIM,True)
dim(tx0+30*S,ty0,tx0+30*S,ty0+38*S,"38",22,DIM,False)
text(tx0+15*S, ty0+3.6*S, "mount box (footprint", 10, GREY); text(tx0+15*S, ty0+3.6*S+11, "not measured)", 10, GREY)
# arm reach arrow
line(cx,cy+10,cx,ty0+34*S,GREY,1.5,"5,5"); text(cx,ty0+37*S-4,"arm reach direction (assumed)",10,GREY)

# ---------------- Panel 3: gripper ----------------
gx0, gy0 = 60, 600
text(gx0-30,gy0-30,"3. Gripper (S6)",15,INK,"start","bold")
# open
def pent(cx,top,h,gap,w,label):
    top_pt=(cx,top); l1=(cx-w/2,top+h*0.3); r1=(cx+w/2,top+h*0.3)
    lb=(cx-gap/2,top+h); rb=(cx+gap/2,top+h)
    for a,b in ((top_pt,l1),(top_pt,r1),(l1,lb),(r1,rb)):
        line(a[0],a[1],b[0],b[1],"#3b5a84",5)
    for p in (top_pt,l1,r1): circle(p[0],p[1],5,"white",INK,2)
    rect(lb[0]-6,lb[1]-4,12,8,"#3b5a84",INK,1); rect(rb[0]-6,rb[1]-4,12,8,"#3b5a84",INK,1)
    text(cx,top-14,label,12,INK,"middle","bold")
    return top_pt,lb,rb
tp,lb,rb = pent(gx0+70, gy0, 10.0*S, 4.3*S, 12*S, "open")
dim(gx0+70+70, gy0+10*S, gx0+70+70, gy0, "10,0", 4, DIM, False)
dim(lb[0],lb[1]+6,rb[0],rb[1]+6,"4,3",14,DIM,False)
# closed on block
cx2 = gx0+300
top=(cx2,gy0); l1=(cx2-5.5*S,gy0+4.5*S); r1=(cx2+5.5*S,gy0+4.5*S); bl=(cx2-1.8*S,gy0+11*S-1.8*S); br=(cx2+1.8*S,gy0+11*S-1.8*S)
for a,b in ((top,l1),(top,r1),(l1,bl),(r1,br)):
    line(a[0],a[1],b[0],b[1],"#3b5a84",5)
for p in (top,l1,r1): circle(p[0],p[1],5,"white",INK,2)
rect(cx2-1.8*S, gy0+11*S-3.6*S, 3.6*S, 3.6*S, "#f6d6c8", RED, 1.5)
text(cx2, gy0-14, "closed on block", 12, INK, "middle", "bold")
dim(cx2+70, gy0+11*S, cx2+70, gy0, "11,0", 4, DIM, False)
dim(cx2-1.8*S, gy0+11*S+6, cx2+1.8*S, gy0+11*S+6, "3,6 block", 14, RED, False)
# finger link
fx = gx0+520
line(fx,gy0,fx,gy0+11.5*S,"#3b5a84",6); circle(fx,gy0,5,"white",INK,2); circle(fx,gy0+5.5*S,5,"white",INK,2); circle(fx,gy0+11.5*S,5,"white",INK,2)
dim(fx+14,gy0+11.5*S,fx+14,gy0,"11,5",4,DIM,False)
text(fx, gy0-14, "finger link", 12, INK, "middle", "bold")
text(fx, gy0+11.5*S+24, "(mid joint position", 10, GREY); text(fx, gy0+11.5*S+35, "not measured)", 10, GREY)

# ---------------- Panel 4: cube ----------------
qx, qy = 760, 640
text(qx-20,qy-30,"4. Test cube",15,INK,"start","bold")
e = 2.65*S*2.5  # enlarged 2.5x for readability
d = e*0.45
pts_front=[(qx,qy+d),(qx+e,qy+d),(qx+e,qy+d+e),(qx,qy+d+e)]
add(f"<polygon points='{qx},{qy+d} {qx+d},{qy} {qx+e+d},{qy} {qx+e},{qy+d}' fill='#eef2f6' stroke='{INK}' stroke-width='2'/>")
add(f"<polygon points='{qx+e},{qy+d} {qx+e+d},{qy} {qx+e+d},{qy+e} {qx+e},{qy+d+e}' fill='#dfe6ee' stroke='{INK}' stroke-width='2'/>")
rect(qx,qy+d,e,e,"#f7f9fb",INK)
dim(qx,qy+d+e,qx+e,qy+d+e,"2,65",20,DIM,False)
dim(qx,qy+d,qx,qy+d+e,"2,65",14,DIM,False)
text(qx+e+d+10, qy+d/2+4, "depth 2,65", 12, DIM, "start", "bold")
text(qx+e/2+10, qy+d+e+60, "(drawn 2.5x larger than scale)", 10, GREY)

# ---------------- chain summary ----------------
sy = 860
text(30, sy, "Kinematic chain (link lengths between joint axes)", 15, INK, "start", "bold")
chain = [("S1 base",None),("S2 shoulder","9,6 up"),("S3 elbow","10,5"),("S4 wrist flex","12,5"),("S5 wrist rot","6,7"),("gripper tip","10,8 (=17,5-6,7)")]
x = 60
for i,(name,ln) in enumerate(chain):
    if ln:
        line(x-160+10, sy+40, x-10, sy+40, "#3b5a84", 5)
        text(x-80, sy+30, ln, 12, DIM, "middle", "bold")
    circle(x, sy+40, 8, "white", INK, 2.5)
    text(x, sy+66, name, 11, INK)
    x += 160
add("</svg>")
open("/Users/jan/projects/px-robotoc-arm-sim/docs/images/arm-dimensions.svg","w").write("\n".join(out))
print("ok")

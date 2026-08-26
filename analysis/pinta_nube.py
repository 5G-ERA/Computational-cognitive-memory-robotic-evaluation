"""Plano cenital de la nube Summit en frame G1, coloreado por banda de altura.
Dos salidas: completa (con techo) y solo-pared (continuidad de muros).
Rejilla de 1 m rotulada para que Adrian pueda referirse a zonas."""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

P = np.load("/home/ros/isaac_ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305
zrel = P[:,2] - (fa*P[:,0] + fb*P[:,1] + fc)

F = 0.05
X0, X1, Y0, Y1 = -9.0, 9.0, -11.0, 12.5
W = int((X1-X0)/F); H = int((Y1-Y0)/F)
ix = ((P[:,0]-X0)/F).astype(int); iy = ((P[:,1]-Y0)/F).astype(int)
ok = (ix>=0)&(ix<W)&(iy>=0)&(iy<H)
ix, iy, zr = ix[ok], iy[ok], zrel[ok]

# banda dominante por celda = la mas ALTA con >=3 puntos (robusto a transeuntes sueltos)
# codigos: 0 nada, 1 suelo, 2 bajo, 3 mueble, 4 alto, 5 pared, 6 techo
lim = [(2.45, 6), (1.75, 5), (1.15, 4), (0.45, 3), (0.06, 2), (-1.0, 1)]
cnt = np.zeros((7, H, W), dtype=np.int32)
band = np.ones(len(zr), dtype=np.int8)
for i,(lo,code) in enumerate(lim[:-1]):
    band[(zr>=lo)] = np.maximum(band[(zr>=lo)], 0)  # placeholder
# mas simple: banda por punto
bp = np.ones(len(zr), dtype=np.int8)
bp[zr>=0.06]=2; bp[zr>=0.45]=3; bp[zr>=1.15]=4; bp[zr>=1.75]=5; bp[zr>=2.45]=6
for c in range(1,7):
    m = bp==c
    np.add.at(cnt[c], (iy[m], ix[m]), 1)

celda = np.zeros((H,W), dtype=np.int8)
for c in range(1,7):
    celda[(cnt[c]>=3)] = c            # el bucle asciende: gana la banda mas alta con >=3

COL = {0:(16,16,20), 1:(52,56,66), 2:(70,95,140), 3:(58,150,90), 4:(215,170,60), 5:(220,70,60), 6:(240,240,245)}
S = 3   # px por celda de 5 cm
im = Image.new("RGB", (W*S, H*S), COL[0])
d = ImageDraw.Draw(im)
px = im.load()
for yy in range(H):
    for xx in range(W):
        c = celda[yy,xx]
        if c:
            for a in range(S):
                for b in range(S):
                    px[xx*S+a, (H-1-yy)*S+b] = COL[c]

def xy2px(x, y):
    return int((x-X0)/F*S), int((H-1-(y-Y0)/F)*S)

f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
fb2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
for gx in range(int(X0), int(X1)+1):
    a,_ = xy2px(gx, 0)
    d.line([(a,0),(a,H*S)], fill=(255,255,255,30) if gx else (120,120,140), width=1)
    d.text((a+2, 4), str(gx), font=f, fill=(150,150,160))
for gy in range(int(Y0), int(Y1)+1):
    _,b = xy2px(0, gy)
    d.line([(0,b),(W*S,b)], fill=(255,255,255,30) if gy else (120,120,140), width=1)
    d.text((4, b+2), str(gy), font=f, fill=(150,150,160))

# ruta real
RUTA = [(0.99,0.57),(-3.12,0.47),(-3.90,1.25),(-4.82,2.17)]
for i in range(len(RUTA)-1):
    d.line([xy2px(*RUTA[i]), xy2px(*RUTA[i+1])], fill=(80,200,255), width=3)
for (x,y),e in [((0.99,0.57),"A"),((-3.90,1.25),"puerta"),((-4.71,2.84),"B")]:
    a,b = xy2px(x,y)
    d.ellipse([a-6,b-6,a+6,b+6], outline=(80,200,255), width=3)
    d.text((a+8,b-8), e, font=fb2, fill=(80,200,255))

# leyenda
ley = [("suelo",1),("bajo <0.45",2),("mueble 0.45-1.15",3),("alto 1.15-1.75",4),("pared >1.75",5),("techo >2.45",6)]
d.rectangle([W*S-260, H*S-150, W*S-8, H*S-8], fill=(10,10,14))
for i,(e,c) in enumerate(ley):
    d.rectangle([W*S-250, H*S-140+i*22, W*S-232, H*S-126+i*22], fill=COL[c])
    d.text((W*S-226, H*S-142+i*22), e, font=f, fill=(220,220,230))
im.save("/tmp/nube_alturas.png")

# version SOLO pared/techo, para continuidad de muros
im2 = Image.new("RGB", (W*S, H*S), (12,12,16))
px2 = im2.load()
for yy in range(H):
    for xx in range(W):
        if cnt[5][yy,xx] >= 2 or cnt[6][yy,xx] >= 2:
            c = (240,240,245) if cnt[6][yy,xx] >= 2 else (220,70,60)
            for a in range(S):
                for b in range(S):
                    px2[xx*S+a, (H-1-yy)*S+b] = c
d2 = ImageDraw.Draw(im2)
for gx in range(int(X0), int(X1)+1):
    a,_ = xy2px(gx,0); d2.line([(a,0),(a,H*S)], fill=(60,60,70)); d2.text((a+2,4), str(gx), font=f, fill=(140,140,150))
for gy in range(int(Y0), int(Y1)+1):
    _,b = xy2px(0,gy); d2.line([(0,b),(W*S,b)], fill=(60,60,70)); d2.text((4,b+2), str(gy), font=f, fill=(140,140,150))
for i in range(len(RUTA)-1):
    d2.line([xy2px(*RUTA[i]), xy2px(*RUTA[i+1])], fill=(80,200,255), width=3)
im2.save("/tmp/nube_pared.png")
print("guardados /tmp/nube_alturas.png y /tmp/nube_pared.png", im.size)

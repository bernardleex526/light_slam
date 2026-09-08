from pathlib import Path
import json,numpy as np,math
from PIL import Image
out=Path(__file__).resolve().parents[2]/'data/validation/nav2';p=np.loadtxt(out/'trajectory.csv',delimiter=',',skiprows=1);im=np.asarray(Image.open(out/'room.pgm'))[::-1];cells=np.argwhere(im==0);centers=-8+(cells[:,::-1]+.5)*.1;collisions=0
for _,x,y,a in p:
 co,si=math.cos(a),math.sin(a);d=centers-[x,y];world=(abs(d[:,0])<=.46*abs(co)+.303*abs(si)+.05)&(abs(d[:,1])<=.46*abs(si)+.303*abs(co)+.05);local=(abs(d[:,0]*co+d[:,1]*si)<=.46+.05*(abs(co)+abs(si)))&(abs(-d[:,0]*si+d[:,1]*co)<=.303+.05*(abs(co)+abs(si)));collisions+=bool(np.any(world&local))
r=json.loads((out/'result.json').read_text());r.update(footprint_collision_samples=collisions,footprint_checked_samples=len(p));(out/'result.json').write_text(json.dumps(r,indent=2));print(r)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(7,5),layout='constrained');ax.imshow(im,cmap='gray',origin='lower',extent=[-8,8,-8,8],vmin=0,vmax=255);ax.plot(p[:,1],p[:,2],label='Synthetic robot trajectory');ax.scatter([0,3],[0,1],c=['green','red'])
for _,x,y,a in p[::max(1,len(p)//15)]:
 co,si=math.cos(a),math.sin(a);points=np.array([(x+co*xx-si*yy,y+si*xx+co*yy) for xx,yy in [(.46,.303),(.46,-.303),(-.46,-.303),(-.46,.303),(.46,.303)]]);ax.plot(points[:,0],points[:,1],color='tab:blue',alpha=.3)
ax.set(xlim=(-.8,3.8),ylim=(-1,2),xlabel='x (m)',ylabel='y (m)',title='Nav2 closed loop: synthetic odometry, static obstacle');ax.set_aspect('equal');ax.legend();fig.savefig(out/'trajectory.png',dpi=160)

if collisions:
 raise SystemExit(1)

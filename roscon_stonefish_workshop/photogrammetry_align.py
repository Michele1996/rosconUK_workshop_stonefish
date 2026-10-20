#!/usr/bin/env python3
"""Estimate metric COLMAP->Stonefish similarity using matched image filenames.

Requires colmap model_converter --output_type TXT; no ground truth is exposed to
SfM itself. The local manifest records diagnostic odom poses for evaluation.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np

CAMERA_OFFSET_BODY_M=np.array([.2,0.,.3])


def body_camera_centres(manifest):
    from scipy.spatial.transform import Rotation
    positions={}
    with Path(manifest).open(newline='') as f:
        for row in csv.DictReader(f):
            try:
                pos=np.array([float(row['odom_'+a+'_m']) for a in 'xyz'])
                q=np.array([float(row['odom_q'+a]) for a in 'xyzw'])
                if not(np.isfinite(pos).all() and np.isfinite(q).all()) or np.linalg.norm(q)<.5:
                    continue
                q=q/np.linalg.norm(q)
                positions[Path(row['image']).name]=pos+Rotation.from_quat(q).apply(CAMERA_OFFSET_BODY_M)
            except (ValueError,KeyError,TypeError):continue
    return positions


def colmap_camera_centres(images_txt):
    from scipy.spatial.transform import Rotation
    centres={}
    with Path(images_txt).open() as f:
        for line in f:
            if not line.strip() or line.startswith('#'):continue
            items=line.split()
            if len(items)<10 or not items[9].lower().endswith(('.png','.jpg','.jpeg')):
                continue
            try:
                q=np.array([float(items[i]) for i in (2,3,4,1)]) # COLMAP qw qx qy qz -> xyzw
                t=np.array([float(items[i]) for i in (5,6,7)])
                if not np.isfinite(q).all() or not np.isfinite(t).all():continue
                c=-Rotation.from_quat(q).inv().apply(t)
                centres[Path(items[9]).name]=c
            except (ValueError,IndexError):continue
    return centres


def umeyama(source,target):
    x=np.asarray(source,dtype=float);y=np.asarray(target,dtype=float)
    if x.shape!=y.shape or x.ndim!=2 or x.shape[1]!=3 or len(x)<3:
        raise ValueError('need at least 3 paired 3D positions')
    xm=x.mean(0);ym=y.mean(0)
    xc=x-xm;yc=y-ym
    var=np.mean(np.sum(xc**2,axis=1))
    if var<1e-10:raise ValueError('camera centres too similar to estimate scale')
    U,d,Vt=np.linalg.svd(yc.T@xc/len(x))
    s=np.eye(3)
    if np.linalg.det(U@Vt)<0:s[-1,-1]=-1
    rot=U@s@Vt;scale=float(np.sum(d*np.diag(s))/var)
    if scale<=0 or not np.isfinite(scale):raise ValueError('invalid scale estimate')
    t=ym-scale*rot@xm
    return scale,rot,t


def fit_alignment(source,target,iterations=200,threshold_m=.65):
    x=np.asarray(source);y=np.asarray(target)
    if len(x)<6:raise ValueError('at least 6 registered images required to align safely')
    rng=np.random.default_rng(2026)
    best=None
    for _ in range(iterations):
        ids=rng.choice(len(x),3,replace=False)
        try:scale,R,t=umeyama(x[ids],y[ids])
        except (ValueError,np.linalg.LinAlgError):continue
        d=np.linalg.norm(scale*(x@R.T)+t-y,axis=1)
        inliers=d<threshold_m
        quality=(int(sum(inliers)),-float(np.mean(np.clip(d,0,5))))
        if best is None or quality>best[0]:best=(quality,inliers)
    if best is None or sum(best[1])<6:raise ValueError('Too few consistent camera correspondences (check scene/time/odometry)')
    scale,R,t=umeyama(x[best[1]],y[best[1]])
    residual=np.linalg.norm(scale*(x@R.T)+t-y,axis=1)
    H=np.eye(4);H[:3,:3]=scale*R;H[:3,3]=t
    return H,{'matches':len(x),'inliers':int(sum(best[1])),
              'scale':scale,'median_position_residual_m':float(np.median(residual[best[1]])),
              'max_position_residual_m':float(np.max(residual[best[1]]))}


def main():
    p=argparse.ArgumentParser(description='Align COLMAP reconstruction to Stonefish world using the captured image manifest')
    p.add_argument('--manifest',required=True)
    p.add_argument('--colmap-images',required=True,help='images.txt converted from COLMAP sparse model')
    p.add_argument('--output',default='colmap_to_world.json')
    p.add_argument('--max-residual',type=float,default=.65,help='Robust pose correspondence threshold in metres')
    a=p.parse_args()
    if not np.isfinite(a.max_residual) or a.max_residual<=0:p.error('--max-residual must be positive')
    world=body_camera_centres(a.manifest);sfm=colmap_camera_centres(a.colmap_images)
    matches=sorted(set(world)&set(sfm))
    if len(matches)<6:raise SystemExit('At least six matching image filenames and odometry poses are needed; found '+str(len(matches)))
    H,stats=fit_alignment([sfm[n] for n in matches],[world[n] for n in matches],threshold_m=a.max_residual)
    Path(a.output).parent.mkdir(parents=True,exist_ok=True)
    Path(a.output).write_text(json.dumps(H.tolist(),indent=2)+'\n')
    stat_path=Path(a.output).with_suffix('.stats.json')
    stat_path.write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps(stats,indent=2));print('World-alignment transform:',a.output)
    if stats['median_position_residual_m']>.3:
        print('WARNING: large alignment residual; visually inspect camera tracks before trusting mesh distances.')

if __name__=='__main__':main()

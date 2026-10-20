#!/usr/bin/env python3
"""Optional geometric metrics on an *explicitly aligned* reconstruction.

Requires optional pip packages: trimesh and scipy, and numpy. The submitted PLY
must already be expressed in world metres (NED) and cropped to the named target.
No automatic registration is fabricated from simulator ground truth.
"""
import argparse
import json
from pathlib import Path
import numpy as np

REFERENCE = {
 'cat': ('cat_textured.obj',(-10.,-6.,8.95),.10),
 'bust':('bust_textured.obj',(-10.,5.,8.95),.10),
 'wreck':('wr_00.obj',(19.,6.,9.36),.25),
}

def compute_metrics(reference_points, reconstructed_points, threshold):
    from scipy.spatial import cKDTree
    ref=np.asarray(reference_points,dtype=float)
    rec=np.asarray(reconstructed_points,dtype=float)
    if ref.ndim!=2 or rec.ndim!=2 or ref.shape[1]!=3 or rec.shape[1]!=3 or len(ref)<3 or len(rec)<3:
        raise ValueError('Need at least 3 finite XYZ points in each cloud')
    if not (np.isfinite(ref).all() and np.isfinite(rec).all()):
        raise ValueError('Nonfinite point found')
    rec_to_ref=cKDTree(ref).query(rec,k=1)[0]
    ref_to_rec=cKDTree(rec).query(ref,k=1)[0]
    precision=float(np.mean(rec_to_ref<=threshold))
    recall=float(np.mean(ref_to_rec<=threshold))
    f=2*precision*recall/(precision+recall) if precision+recall else 0.
    return {'distance_threshold_m':threshold,'mean_accuracy_distance_m':float(np.mean(rec_to_ref)),
            'mean_completeness_distance_m':float(np.mean(ref_to_rec)),
            'precision':round(precision,5),'completeness':round(recall,5),
            'f_score':round(f,5),'reconstructed_points':len(rec),'reference_sample_points':len(ref)}


def load_xyz(path,max_points=50000):
    import trimesh
    p=Path(path)
    if p.suffix.lower() in ('.xyz','.txt'):
        pts=np.loadtxt(p,usecols=(0,1,2),ndmin=2)
    else:
        cloud=trimesh.load(str(p),process=False)
        if hasattr(cloud,'vertices'):
            pts=np.asarray(cloud.vertices)
        else:raise ValueError('Expected PLY/XYZ point cloud, not multi-object geometry')
    if len(pts)>max_points:
        rng=np.random.default_rng(210)
        pts=pts[rng.choice(len(pts),max_points,replace=False)]
    return pts


def evaluate(target,ply,data_dir,alignment=None,threshold=None,auto_crop=False):
    import trimesh
    name,xyz,default_threshold=REFERENCE[target]
    if target=='wreck':
        parts=[]
        for part in sorted(Path(data_dir).glob('wr_[0-9][0-9].obj')):
            part_mesh=trimesh.load(str(part),process=False)
            if isinstance(part_mesh,trimesh.Trimesh):parts.append(part_mesh)
        if not parts:raise FileNotFoundError('No wreck mesh parts in '+str(data_dir))
        mesh=trimesh.util.concatenate(parts)
        name='wr_00..wr_14.obj (all available components)'
    else:
        mesh=trimesh.load(str(Path(data_dir)/name),process=False)
        if not isinstance(mesh,trimesh.Trimesh):raise ValueError('Reference mesh must be a trimesh mesh')
    ref=trimesh.sample.sample_surface(mesh,30000,seed=210)[0] + np.array(xyz)
    rec=load_xyz(ply)
    if alignment is not None:
        t=np.asarray(json.loads(Path(alignment).read_text()),dtype=float)
        if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]):
            raise ValueError('Alignment must be a finite 4x4 homogeneous transform (reconstruction -> world metres)')
        rec=(t[:3,:3]@rec.T).T+t[:3,3]
    if auto_crop:
        bounds=np.stack([ref.min(0),ref.max(0)])
        # 0.4m padding: avoids counting the majority of irrelevant seabed.
        keep=np.all((rec>=bounds[0]-.4)&(rec<=bounds[1]+.4),axis=1)
        rec=rec[keep]
        if len(rec)<3:raise ValueError('No reconstructed points after reference bounding-box crop; check alignment')
    # Must explicitly acknowledge that the point cloud is scaled, cropped, aligned.
    return {'target':target,'registration_requirement':'reconstruction prealigned in metric NED world coordinates',
            'reference_mesh':name,
            **compute_metrics(ref,rec,threshold or default_threshold)}


def main():
    p=argparse.ArgumentParser(description='Compare one aligned cloud to the real simulation mesh')
    p.add_argument('--target',choices=REFERENCE,required=True)
    p.add_argument('--cloud',required=True,help='PLY or XYZ, CROPPED to target, metric scale')
    p.add_argument('--auto-crop',action='store_true',help='Crop aligned cloud to target reference mesh bounding box (+0.4m)')
    p.add_argument('--alignment',help='Optional 4x4 transform JSON from cloud frame to NED metres')
    p.add_argument('--reference-dir',default=str(Path(__file__).resolve().parents[1]/'data/photogrammetry'))
    p.add_argument('--threshold',type=float)
    p.add_argument('--output',default='')
    a=p.parse_args()
    if a.threshold is not None and (not np.isfinite(a.threshold) or a.threshold<=0):
        p.error('--threshold must be a positive finite distance in metres')
    result=evaluate(a.target,a.cloud,a.reference_dir,a.alignment,a.threshold,a.auto_crop)
    s=json.dumps(result,indent=2)
    print(s)
    if a.output:
        Path(a.output).parent.mkdir(parents=True,exist_ok=True)
        Path(a.output).write_text(s+'\n')

if __name__=='__main__':main()

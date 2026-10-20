#!/usr/bin/env python3
"""Offline acquisition benchmark for independent statue-pair and wreck surveys.

This assesses camera *sampling geometry* inferred from the capture manifest, not
true visibility or reconstructed surface completeness. Geometry must be assessed
separately from aligned reconstruction points.
"""
import argparse
import csv
import json
import math
from pathlib import Path

CAT = (-10.0, -6.0)
BUST = (-10.0, 5.0)
WRECK = (19.0, 6.0)


def read_manifest(path, image_root=None):
    path = Path(path)
    if path.is_dir():
        path = path / 'manifest.csv'
    root = Path(image_root) if image_root else path.parent / 'images'
    rows = []
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            try:
                x,y,z = (float(row['odom_'+k+'_m']) for k in ('x','y','z'))
                stamp = float(row['stamp_sec']) + float(row['stamp_nanosec']) * 1e-9
                im = root / Path(row['image']).name
                if all(map(math.isfinite,(x,y,z,stamp))) and im.is_file():
                    rows.append({'image':im.name,'xyz':(x,y,z),'time':stamp})
            except (ValueError,KeyError,TypeError):
                continue
    return rows


def _clamp(v):
    return max(0.0,min(1.0,float(v)))


def _result(name, images, views, required, levels, max_levels=2, geometry_label='viewpoint coverage proxy'):
    count_score = 25 * _clamp(len(images)/required)
    view_score = 60 * _clamp(len(views[0])/views[1])
    height_score = 15 * _clamp(len(levels)/max_levels)
    return {'target':name,'images_in_evaluation_region':len(images),
            'viewpoint_cells_visited':len(views[0]),'viewpoint_cells_total':views[1],
            'camera_height_bands_visited':sorted(levels),
            'image_count_points':round(count_score,2),
            'viewpoint_diversity_points':round(view_score,2),
            'vertical_diversity_points':round(height_score,2),
            'score_out_of_100':round(count_score+view_score+height_score,2),
            'interpretation':geometry_label}


def statue_score(rows, name, center):
    eligible=[];sectors=set();heights=set()
    for r in rows:
        x,y,z=r['xyz'];dx=x-center[0];dy=y-center[1]
        rho=math.hypot(dx,dy)
        if not 1.6<=rho<=5.5 or not 3.5<=z<=9.5:
            continue
        eligible.append(r)
        sectors.add(int((math.atan2(dy,dx)%(2*math.pi))/(2*math.pi)*12)%12)
        heights.add('upper' if z<7.0 else 'lower') # NED: smaller z means higher
    return _result(name, eligible, (sectors,12),24,heights)


def wreck_score(rows):
    eligible=[];cells=set(); heights=set()
    # The wreck extends approx x=16..22, y=-3.3..15.3 in the scene.
    # Use offset lanes outside the hull rather than proximity to its centre.
    for r in rows:
        x,y,z=r['xyz']
        if not 12<=x<=26 or not -8<=y<=20 or not 3.5<=z<=9.5:
            continue
        if 16 <= x <= 22:  # over structure: useful oblique/top views
            side='above'
        else:
            side='west' if x<16 else 'east'
        section=max(0,min(5,int((y+3.3)/18.6*6)))
        if not -4 <= y <= 16:
            continue
        eligible.append(r)
        cells.add((side,section))
        heights.add('upper' if z<7 else 'lower')
    # Three observation lanes times six longitudinal stations.
    return _result('shipwreck',eligible,(cells,18),72,heights)


def evaluate(rows, challenge='both'):
    cats=statue_score(rows,'concrete_cat',CAT)
    bust=statue_score(rows,'marble_bust',BUST)
    wreck=wreck_score(rows)
    data={'schema':'roscon-photogrammetry-acquisition-v1',
          'metric_type':'camera-viewpoint sampling proxy; not reconstruction quality',
          'input_images_with_valid_odom':len(rows),
          'statues':{'cat':cats,'bust':bust,
                     'combined_score_out_of_100':round((cats['score_out_of_100']+bust['score_out_of_100'])/2,2)},
          'shipwreck':wreck,
          'limitations':[
              'Image capture locations alone cannot establish that the target appears in a frame.',
              'No occlusion, surface visibility, SfM registration, or reconstructed geometry is assessed.',
              'Odometry in manifest is diagnostic simulation data, not available to image-only SfM.',
              'Independent official geometric scoring requires an aligned reconstruction and reference mesh.']}
    if challenge=='statues':data.pop('shipwreck')
    if challenge=='wreck':data.pop('statues')
    return data


def main():
    p=argparse.ArgumentParser(description='Evaluate cat+bust and shipwreck photo-collection missions independently')
    p.add_argument('--survey',default=str(Path.home()/'roscon_photogrammetry'/'survey'))
    p.add_argument('--challenge',choices=['statues','wreck','both'],default='both')
    p.add_argument('--output',default='')
    args=p.parse_args()
    report=evaluate(read_manifest(args.survey),args.challenge)
    out=Path(args.output) if args.output else Path(args.survey)/'acquisition_scores.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));print('Saved:',out)

if __name__=='__main__':main()

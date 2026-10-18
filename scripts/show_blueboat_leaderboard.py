#!/usr/bin/env python3
"""Rank locally submitted BlueBoat race ZIPs by best finished adjusted time."""
import argparse
import json
from pathlib import Path
from zipfile import ZipFile, BadZipFile


def main():
    parser=argparse.ArgumentParser(description='Combine independently submitted ROSCon results')
    parser.add_argument('submissions_dir', nargs='?', default='~/roscon_submissions')
    args=parser.parse_args()
    folder=Path(args.submissions_dir).expanduser()
    rows=[]
    for zip_path in sorted(folder.glob('*.zip')):
        try:
            with ZipFile(zip_path) as zf:
                summary=json.loads(zf.read('submission_summary.json'))
            score=summary.get('best_adjusted_time_s')
            if score is None: continue
            rows.append((float(score),summary.get('team','?'),zip_path.name,
                         summary.get('best_attempt_number','?'),
                         summary.get('course_sha256_prefix','?')))
        except (OSError,KeyError,ValueError,TypeError,BadZipFile):
            print('IGNORED invalid submission:',zip_path)
    if not rows:
        print('No completed race submissions in',folder)
        return
    print('Rank  Adjusted(s)  Team  Attempt  Course hash  File')
    for i,(score,team,name,attempt,course) in enumerate(sorted(rows),1):
        print(f'{i:>4}  {score:>11.3f}  {team}  {attempt}  {course}  {name}')
    print('\nLOCAL RESULTS ARE SELF-REPORTED; check trajectories and scene hashes for review.')


if __name__=='__main__': main()

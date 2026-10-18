"""Prepare a reviewable result ZIP from the team's local self-reported logs."""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

from .race_records import DEFAULT_RESULTS, slug


def export(team, results_dir=DEFAULT_RESULTS, destination='.', course_hash=None):
    root = Path(results_dir).expanduser() / slug(team)
    if course_hash:
        folders = [root / course_hash[:12]]
    else:
        folders = sorted(p for p in root.glob('*') if p.is_dir()) if root.exists() else []
    if not folders:
        raise RuntimeError('No official attempts recorded for this team')
    # A package with a single installed course will use a single folder; if multiple
    # course variants are present, export the most recently modified report set.
    folder = max(folders, key=lambda f: f.stat().st_mtime)
    rows = []
    for fp in sorted(folder.glob('attempt_*/report.json')):
        try: rows.append((fp, json.loads(fp.read_text(encoding='utf-8'))))
        except (OSError, ValueError): continue
    valid = [(fp, r) for fp, r in rows if r.get('status') == 'FINISHED'
             and isinstance(r.get('adjusted_time_s'), (float, int))]
    best = min(valid, key=lambda e:e[1]['adjusted_time_s'], default=None)
    outdir = Path(destination).expanduser()
    outdir.mkdir(parents=True, exist_ok=True)
    name = folder.parent.name + '_' + folder.name + '_submission.zip'
    target = outdir / name
    manifest = {
        'schema':'roscon-blueboat-export-v1', 'team':team,
        'course_sha256_prefix':folder.name, 'official_attempts_consumed':len(rows),
        'best_adjusted_time_s':best[1]['adjusted_time_s'] if best else None,
        'best_attempt_number':best[1]['attempt_number'] if best else None,
        'reports':[r for _,r in rows],
        'warning':'Self-reported local results; hash checks are not anti-tamper proof.',
    }
    with ZipFile(target, 'w', compression=ZIP_DEFLATED) as archive:
        for fp, record in rows:
            for candidate in [fp, fp.with_name('trajectory.csv')]:
                if candidate.exists():
                    archive.write(candidate, arcname=f'{fp.parent.name}/{candidate.name}')
        archive.writestr('submission_summary.json', json.dumps(manifest,indent=2))
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    return target, manifest, digest


def main():
    p=argparse.ArgumentParser(description='Export all three official attempts + best time')
    p.add_argument('--team', required=True)
    p.add_argument('--results-dir', default=DEFAULT_RESULTS)
    p.add_argument('--output-dir', default='~/roscon_submissions')
    args=p.parse_args()
    try:
        path, manifest, digest=export(args.team,args.results_dir,args.output_dir)
    except RuntimeError as exc:
        p.error(str(exc))
    print(f'Export: {path}\nOfficial attempts: {manifest["official_attempts_consumed"]}'
          f'\nBest adjusted time: {manifest["best_adjusted_time_s"]}'
          f'\nSHA256: {digest}\nSubmit this ZIP to the organiser.')


if __name__=='__main__':
    main()

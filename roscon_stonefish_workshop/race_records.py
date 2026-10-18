"""Local, ROS-independent evidence and three-attempt ledger for the Grand Prix.

This is an educational self-reported competition, not an anti-tamper system.
All laptops execute their own simulator and can modify their local files.
"""
import hashlib
import json
import os
import re
import uuid
from datetime import datetime, timezone
from pathlib import Path

MAX_OFFICIAL_ATTEMPTS = 3
DEFAULT_RESULTS = '~/.ros/roscon_blueboat_grand_prix'


def slug(team):
    label = re.sub(r'[^a-zA-Z0-9_-]+', '_', team.strip())[:50].strip('_')
    return label or 'Team'


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w', encoding='utf-8') as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write('\n')
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class AttemptStore:
    def __init__(self, root, team, course_version, course_hash, limit=MAX_OFFICIAL_ATTEMPTS):
        self.team = str(team)
        self.course_version = str(course_version)
        self.course_hash = str(course_hash)
        self.limit = int(limit)
        self.root = Path(root).expanduser() / slug(team) / course_hash[:12]
        self.root.mkdir(parents=True, exist_ok=True)
        self.current = None
        self.path = None

    def attempts(self):
        rows = []
        for fp in sorted(self.root.glob('attempt_*/report.json')):
            try:
                record = json.loads(fp.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if record.get('team') == self.team and record.get('course_sha256') == self.course_hash:
                rows.append((fp, record))
        return sorted(rows, key=lambda row: row[1]['attempt_number'])

    @property
    def consumed(self):
        return len(self.attempts())

    @property
    def remaining(self):
        return max(0, self.limit - self.consumed)

    def start(self, sim_time):
        if self.current is not None:
            raise RuntimeError('Attempt already active')
        if not self.remaining:
            raise RuntimeError(f'No official attempts left (limit {self.limit})')
        number = self.consumed + 1
        self.path = self.root / f'attempt_{number:02d}'
        if self.path.exists():
            raise RuntimeError('Attempt directory already exists, refusing overwrite')
        self.path.mkdir(parents=True)
        record = {
            'schema': 'roscon-blueboat-attempt-v1',
            'team': self.team,
            'course_version': self.course_version,
            'course_sha256': self.course_hash,
            'attempt_number': number,
            'attempt_id': uuid.uuid4().hex,
            'created_utc': utc_now(),
            'armed_at_sim_s': float(sim_time),
            'status': 'ARMED',
            'official': True,
            'max_attempts': self.limit,
            'checkpoints': [],
            'trajectory_file': 'trajectory.csv',
            'note': 'Local self-reported simulator result; not cryptographically trusted.'
        }
        save_json(self.path / 'report.json', record)
        (self.path / 'trajectory.csv').write_text(
            'sim_time_s,x_m,y_m,stage,next_gate,offtrack,penalty_s,events\n', encoding='utf-8')
        self.current = record
        return record

    def checkpoint(self, name, timestamp):
        if self.current is None:
            return
        self.current['checkpoints'].append({'gate': str(name), 'sim_time_s': round(float(timestamp), 4)})
        save_json(self.path / 'report.json', self.current)

    def trace(self, timestamp, x, y, status):
        if self.current is None:
            return
        with (self.path / 'trajectory.csv').open('a', newline='', encoding='utf-8') as fh:
            import csv
            csv.writer(fh).writerow([
                f'{timestamp:.4f}', f'{x:.4f}', f'{y:.4f}', status['state'],
                status['next_gate'] or '', int(status['outside']),
                status['penalty_s'], ';'.join(status['events'])
            ])

    def finalize(self, verdict, status=None):
        if self.current is None:
            return None
        if verdict not in ('FINISHED', 'DNF_TIMEOUT', 'DNF_CANCELLED', 'DNF_CLOCK_RESET'):
            raise ValueError('Invalid verdict')
        self.current['status'] = verdict
        self.current['closed_utc'] = utc_now()
        if status:
            self.current.update({
                'race_time_s': status['race_time_s'],
                'penalty_s': status['penalty_s'],
                'adjusted_time_s': status['adjusted_time_s'] if verdict == 'FINISHED' else None,
                'offtrack_excursions': status['offtrack_excursions'],
                'offtrack_duration_s': status['offtrack_duration_s'],
                'gates_passed': status['gates_passed'],
                'gates_total': status['gates_total'],
            })
        save_json(self.path / 'report.json', self.current)
        row = self.current
        self.current, self.path = None, None
        return row


def best_completed(store):
    return min(
        ((fp, r) for fp, r in store.attempts()
         if r.get('status') == 'FINISHED' and isinstance(r.get('adjusted_time_s'), (float, int))),
        key=lambda item: item[1]['adjusted_time_s'], default=None
    )

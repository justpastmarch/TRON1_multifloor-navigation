"""Per-recording run summary written next to each console BAG (<bag>.run.json).

The BAG holds the raw evidence; this file says what the run was, so a run can
be found and compared without opening every BAG: missions started while the
BAG recorded, their results, first stair faults, console events (operator
actions), operator notes / experiment conditions, code commit and the stair
config hash. Recording only: nothing here commands the robot or gates a run.
"""
import json
import os
import subprocess
import threading
import time
from pathlib import Path

SCHEMA = 'tron-run/1'
RESULT_NAMES = {0: 'OK', 1: 'BUSY', 2: 'INVALID_GOAL', 3: 'CAPABILITY_DISABLED', 4: 'NAVIGATION_FAILED',
                5: 'STAIR_FAILED', 6: 'LOCALIZATION_FAILED', 7: 'SCAN_FAILED', 8: 'COMMUNICATION_LOST',
                9: 'MISSION_ABORT'}
NOTE_MAX = 500
_REPO = Path(__file__).resolve().parents[4]


def code_version(repo=_REPO):
    """Commit and dirty flag of the running checkout; None fields when unknown."""
    def git(*args):
        try:
            result = subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        out = getattr(result, 'stdout', None)
        return out.strip() if getattr(result, 'returncode', 1) == 0 and isinstance(out, str) else None
    commit = git('rev-parse', 'HEAD')
    status = git('status', '--porcelain', '--untracked-files=no')
    return dict(commit=commit, branch=git('rev-parse', '--abbrev-ref', 'HEAD'),
                dirty=None if status is None else bool(status))


# The running code does not change under a live console: read it once at import.
_CODE_VERSION = code_version()


def _iso(t):
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t)) + '%+03d:00' % (-time.timezone // 3600)


class RunRecord:
    def __init__(self, clock=time.time, version=lambda: dict(_CODE_VERSION)):
        self.lock = threading.RLock()
        self.clock, self.version = clock, version
        self.events_provider = None   # callable -> console events (dicts with time/kind/detail)
        self.current = None
        self.last_path = None
        self.last_record = None

    def begin(self, bag_path, raw_path=None):
        with self.lock:
            now = self.clock()
            self.current = dict(schema=SCHEMA, bag=str(bag_path), raw_bag=str(raw_path) if raw_path else None,
                                started_at=_iso(now), started_unix=now, code=self.version(),
                                config_sha256=None, missions=[], results=[], first_faults=[], notes=[])

    def active(self):
        with self.lock:
            return self.current is not None

    def mission(self, **fields):
        self._append('missions', fields)

    def result(self, status, result_code, reason, mission_id=''):
        self._append('results', dict(status=status, result_code=result_code,
                                     result_name=RESULT_NAMES.get(result_code, str(result_code)),
                                     reason=reason, mission_id=mission_id))

    def control(self, debug):
        """Feed /stair_supervisor/control_debug; keeps the config hash and each new first fault."""
        if not isinstance(debug, dict):
            return
        with self.lock:
            if self.current is None:
                return
            if debug.get('config_sha256'):
                self.current['config_sha256'] = debug['config_sha256']
            fault = debug.get('first_fault')
            if isinstance(fault, dict) and fault.get('at') not in {f.get('at') for f in self.current['first_faults']}:
                self.current['first_faults'].append(dict(
                    t=_iso(self.clock()), at=fault.get('at'), route_id=debug.get('route_id'),
                    phase=fault.get('phase'), tracking_state=fault.get('tracking_state'),
                    reason=fault.get('reason'), clearance_m=fault.get('clearance_m')))

    def note(self, text):
        """Add an operator note to the running record, else to the last written file."""
        if not isinstance(text, str) or not text.strip():
            raise ValueError('메모를 입력하세요.')
        text = text.strip()[:NOTE_MAX]
        with self.lock:
            entry = dict(t=_iso(self.clock()), text=text)
            if self.current is not None:
                self.current['notes'].append(entry)
                return dict(saved_to='current', path=self.current['bag'])
            if self.last_path is None or not Path(self.last_path).exists():
                raise ValueError('메모를 붙일 녹화가 없습니다. BAG 기록을 켜고 주행하세요.')
            data = json.loads(Path(self.last_path).read_text())
            data.setdefault('notes', []).append(entry)
            self._write(Path(self.last_path), data)
            self.last_record = data
            return dict(saved_to='last', path=data.get('bag'))

    def snapshot(self):
        with self.lock:
            source = self.current if self.current is not None else self.last_record
            if source is None:
                return dict(active=False, notes=[], missions=0, summary_path=None)
            return dict(active=self.current is not None, notes=list(source.get('notes', [])),
                        missions=len(source.get('missions', [])), results=list(source.get('results', []))[-3:],
                        summary_path=None if self.current is not None else self.last_path)

    def finish(self, saved, error=''):
        """Write <bag>.run.json once the local BAG is finalized (saved or not)."""
        with self.lock:
            record, self.current = self.current, None
            if record is None:
                return None
            now = self.clock()
            start = record.pop('started_unix')
            events = []
            if self.events_provider is not None:
                try:
                    events = [dict(t=_iso(e['time']), kind=e.get('kind'), detail=e.get('detail'))
                              for e in self.events_provider() if e.get('time', 0) >= start - 1]
                except Exception:  # noqa: BROAD_EXCEPT_OK - a summary must never block BAG finalization
                    events = []
            record.update(ended_at=_iso(now), duration_s=round(now - start, 1), bag_saved=bool(saved),
                          bag_error=error or '', console_events=events)
            path = Path(record['bag']).with_suffix('.run.json')
            try:
                self._write(path, record)
            except OSError:
                return None
            self.last_path, self.last_record = str(path), record
            return str(path)

    def _append(self, key, fields):
        with self.lock:
            if self.current is not None:
                self.current[key].append(dict(t=_iso(self.clock()), **fields))

    @staticmethod
    def _write(path, data):
        temporary = path.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=1)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)

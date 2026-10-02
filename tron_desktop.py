#!/usr/bin/env python3
"""Desktop entry for the existing stack; never submits a mission."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.request

ROOT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8765'


def console_running():
    try:
        with urllib.request.urlopen(URL + '/api/state', timeout=1) as response:
            state = json.load(response)
        return state.get('mode') == 'live' and isinstance(state.get('locations'), list)
    except (OSError, ValueError):
        return False


def open_console():
    result = subprocess.run(['xdg-open', URL], check=False)
    if result.returncode:
        print('브라우저에서 이 주소를 여세요: ' + URL, flush=True)


def pause(message):
    print(message, flush=True)
    try:
        input('Enter를 누르면 이 창을 닫습니다. ')
    except (EOFError, KeyboardInterrupt):
        pass


def main():
    if console_running():
        open_console()
        return 0
    state_dir = Path.home() / '.local/state/tron1'
    state_dir.mkdir(parents=True, exist_ok=True)
    with (state_dir / 'desktop.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('이미 시작 중입니다. 제어 화면 연결을 기다립니다.', flush=True)
            for _ in range(180):
                if console_running():
                    open_console()
                    return 0
                time.sleep(1)
            pause('시작이 아직 완료되지 않았습니다. 먼저 열린 TRON 실행 창을 확인하세요.')
            return 1
        # Another caller may have completed startup while we acquired the lock.
        if console_running():
            open_console()
            return 0
        env = dict(os.environ, MISSION_CONSOLE_PORT='8765', STAIR_RECORD='0')
        log_path = state_dir / ('startup-' + time.strftime('%Y%m%d-%H%M%S') + '.log')
        print('TRON 스택을 시작합니다. UI 서버가 연결되면 제어 화면이 열립니다. 위치 확인 상태는 화면에서 확인하세요.\n'
              '이 창은 실행 중 유지하세요. 종료는 Ctrl+C입니다.\n'
              '주행은 UI에서 직접 시작해야 합니다.\n로그: ' + str(log_path), flush=True)
        with log_path.open('w', buffering=1) as log:
            child = subprocess.Popen(['/bin/bash', str(ROOT / 'run.sh')], cwd=str(ROOT),
                                     env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1, start_new_session=True)
            def read_output():
                for line in child.stdout:
                    log.write(line)
                    print(line, end='', flush=True)
            reader = threading.Thread(target=read_output, daemon=True)
            reader.start()
            def stop(*_):
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, stop)
            opened = False
            while child.poll() is None:
                if not opened and console_running():
                    open_console()
                    opened = True
                time.sleep(.5)
            reader.join(timeout=5)
        if child.returncode not in (0, 130, 143):
            pause('시작 또는 실행이 중단됐습니다. 위 오류와 로그를 확인하세요.\n' + str(log_path))
        return child.returncode


if __name__ == '__main__':
    raise SystemExit(main())

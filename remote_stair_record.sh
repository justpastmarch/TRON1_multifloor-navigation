#!/usr/bin/env bash
# Start/stop one MiniPC raw-sensor bag and copy the completed file to the 1TB disk.
set -Eeuo pipefail
[[ $# -ge 2 && "$2" =~ ^[A-Za-z0-9_]+$ ]] || { echo 'Usage: remote_stair_record.sh start NAME | seal NAME | copy NAME DEST | stop NAME DEST | abort NAME' >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${root}/config.env"
target="${MINI_PC_USER}@${MINI_PC_HOST}"
export RSYNC_RSH="ssh -o BatchMode=yes -o ConnectTimeout=5"
base="/home/${MINI_PC_USER}/.local/share/tron1-stair-supervisor"
unit="tron1-rawbag-$2.service"
case "$1" in
    start)
        ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
            "systemd-run --user --collect --unit=${unit%.service} --property=KillSignal=SIGINT --property=TimeoutStopSec=60s '${base}/record_raw.sh' '$2' >/dev/null"
        if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
            "for i in \$(seq 1 80); do test -f '${base}/bags/$2.bag.active' && exit 0; sleep .1; done; exit 1"; then
            ssh -o BatchMode=yes "$target" "systemctl --user stop '$unit'" || true
            echo 'MiniPC raw LiDAR recording did not become ready.' >&2
            exit 1
        fi
        ;;
    seal)
        ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
            "systemctl --user stop '$unit' >/dev/null 2>&1 || true; test -f '${base}/bags/$2.bag'; test ! -e '${base}/bags/$2.bag.active'"
        ;;
    copy)
        [[ $# == 3 ]] || exit 2
        ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
            "test -f '${base}/bags/$2.bag'; test ! -e '${base}/bags/$2.bag.active'"
        mkdir -p "$(dirname "$3")"
        rsync -a --partial --timeout=60 "${target}:${base}/bags/$2.bag" "$3"
        ssh -o BatchMode=yes "$target" "rm -f '${base}/bags/$2.bag'"
        ;;
    stop)
        [[ $# == 3 ]] || exit 2
        "$0" seal "$2"
        "$0" copy "$2" "$3"
        ;;
    abort)
        ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
            "systemctl --user stop '$unit'"
        ;;
    *) echo 'Expected start, stop, or abort.' >&2; exit 2 ;;
esac

#!/usr/bin/env python3
"""Detached map-save client. SUCCEEDED means a complete server-published bundle."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import yaml

BUNDLE = ('GlobalMap.pcd', 'SurfMap.pcd', 'filterGlobalMap.pcd',
          'mapping_metadata.yaml',
          'trajectory.pcd', 'transformations.pcd')


def incomplete_bundle(destination):
    required = list(BUNDLE)
    try:
        metadata = yaml.safe_load((destination / 'mapping_metadata.yaml').read_text())
        if not isinstance(metadata, dict):
            raise ValueError('metadata must be a mapping')
        version = metadata.get('format_version')
        if version == 1:
            required.append('observed_occupancy.pcd')  # Legacy bundles always include it.
        elif version == 2 and type(metadata.get('occupancy_exported')) is bool:
            if metadata['occupancy_exported']:
                required.append('observed_occupancy.pcd')
        else:
            raise ValueError('unsupported format or missing occupancy declaration')
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return ['mapping_metadata.yaml: ' + str(exc)]
    return [name for name in required if not (destination / name).is_file()
            or (destination / name).stat().st_size == 0]


def write_status(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(str(temporary), str(path))


def perform_save(destination, status_path, call, read_progress, interval=1.0):
    destination = Path(destination)
    status = dict(state='SAVING', destination=str(destination), started=time.time())
    write_status(status_path, status)
    completed = threading.Event()
    outcome = {}

    def invoke():
        try:
            outcome['success'] = bool(call())
        except Exception as exc:
            outcome['error'] = str(exc)
        finally:
            completed.set()

    threading.Thread(target=invoke, daemon=True).start()
    while not completed.wait(interval):
        try:
            progress = read_progress()
            if progress.get('destination') == str(destination):
                status['progress'] = progress
        except Exception:
            pass  # Older servers have no progress; never invent a percentage.
        status['elapsed_seconds'] = round(time.time() - status['started'], 1)
        write_status(status_path, status)
    if 'error' in outcome:
        status.update(state='UNKNOWN', message=outcome['error'] + '; inspect server/map before retrying')
        result = 2
    elif not outcome['success']:
        status.update(state='FAILED', message='Save service rejected or failed; see mapping terminal')
        result = 1
    else:
        missing = incomplete_bundle(destination)
        status.update(state='FAILED' if missing else 'SUCCEEDED',
                      message='Incomplete bundle: ' + ', '.join(missing) if missing else 'Complete map saved')
        result = 1 if missing else 0
    status['elapsed_seconds'] = round(time.time() - status['started'], 1)
    write_status(status_path, status)
    return result


def read_status(job):
    job = Path(job)
    status = json.loads((job / 'status.json').read_text(encoding='utf-8'))
    if status['state'] in ('SUBMITTED', 'SAVING') and (job / 'process.json').exists():
        identity = json.loads((job / 'process.json').read_text())
        try:
            fields = Path('/proc/{}/stat'.format(identity['pid'])).read_text().rsplit(')', 1)[1].split()
            alive = fields[19] == identity['start_ticks'] and fields[0] not in ('Z', 'X')
        except (OSError, IndexError):
            alive = False
        if not alive:
            # The worker may have committed its final status after our first
            # read. Re-read before reporting an incomplete client exit.
            status = json.loads((job / 'status.json').read_text(encoding='utf-8'))
            if status['state'] in ('SUBMITTED', 'SAVING'):
                status.update(state='UNKNOWN', message='Background client exited; inspect worker.log and map before retrying')
    return status


def wait_for_save(job, interval=2.0):
    try:
        while True:
            status = read_status(job)
            state = status['state']
            if state == 'SUCCEEDED':
                print('保存完成：' + str(Path(status['destination']) / 'GlobalMap.pcd'), flush=True)
                return 0
            if state not in ('SUBMITTED', 'SAVING'):
                print(state + ': ' + status.get('message', '检查建图终端和 worker.log'), flush=True)
                return 1 if state == 'FAILED' else 2
            progress = status.get('progress', {})
            print('{}  已用时 {} 秒  {}'.format(state, status.get('elapsed_seconds', 0),
                  json.dumps(progress, ensure_ascii=False) if progress else '等待建图节点进度'), flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print('\n已退出进度显示，后台保存继续。保持建图运行；用 ./save_map.sh --status 查询。')
        return 130


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['start', 'status', 'wait', '_worker'])
    parser.add_argument('--destination')
    parser.add_argument('--resolution', type=float, default=0,
                        help='Output voxel size in metres; 0 keeps native static-map detail (default 0.05 m)')
    parser.add_argument('--job', help='Job directory printed by start')
    parser.add_argument('--log-root', default='/home/nrc/catkin_ws/logs/map-save-jobs')
    parser.add_argument('--lock-fd', type=int)
    parser.add_argument('--wait', action='store_true', help='Show progress after submitting; Ctrl+C leaves save running')
    args = parser.parse_args()
    master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    root = Path(args.log_root) / hashlib.sha256(master.encode()).hexdigest()[:12]
    if args.command in ('status', 'wait'):
        if not args.job and not (root / 'latest.json').is_file():
            parser.error('no map-save job found for this ROS master')
        job = Path(args.job) if args.job else Path(json.loads((root / 'latest.json').read_text())['job'])
        if args.command == 'wait':
            return wait_for_save(job)
        status = read_status(job)
        print(json.dumps(status, ensure_ascii=False, indent=2))
        return 0
    if not args.destination or not math.isfinite(args.resolution) or not 0 <= args.resolution <= 1:
        parser.error('--destination and a finite resolution in [0, 1] are required')
    destination = Path(args.destination).expanduser().resolve()
    if args.command == '_worker':
        import rospy
        from fast_lio_sam.srv import save_map
        # Inherited flock remains held until this process exits, including RPC wait.
        os.fstat(args.lock_fd)
        rospy.init_node('map_save_client', anonymous=True, disable_signals=True)
        def call():
            rospy.wait_for_service('/ducted/mapping/save_map', timeout=10)
            return rospy.ServiceProxy('/ducted/mapping/save_map', save_map)(args.resolution, str(destination)).success
        return perform_save(destination, Path(args.job) / 'status.json', call,
                            lambda: rospy.get_param('/ducted/mapping/save_progress', {}))
    if os.path.lexists(str(destination)):
        parser.error('destination already exists; choose a new directory (do not mkdir it)')
    import fcntl
    root.mkdir(parents=True, exist_ok=True)
    lock = open(root / 'save.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another background save is active; use status and wait')
    job = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d-%H%M%S-'), dir=str(root)))
    write_status(job / 'status.json', dict(state='SUBMITTED', destination=str(destination)))
    try:
        with open(job / 'worker.log', 'ab', buffering=0) as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '_worker',
                '--destination', str(destination), '--resolution', str(args.resolution),
                '--job', str(job), '--lock-fd', str(lock.fileno())],
                stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                start_new_session=True, pass_fds=(lock.fileno(),))
        write_status(root / 'latest.json', dict(job=str(job), pid=process.pid))
        ticks = Path('/proc/{}/stat'.format(process.pid)).read_text().rsplit(')', 1)[1].split()[19]
        write_status(job / 'process.json', dict(pid=process.pid, start_ticks=ticks))
    except Exception as exc:
        write_status(job / 'status.json', dict(state='FAILED', message=str(exc)))
        raise
    finally:
        lock.close()  # Do not LOCK_UN: child owns this same open file description.
    print('Save submitted (not yet complete). Keep mapping/base running and aircraft still.')
    print('Job: ' + str(job))
    print('Check: rosrun ducted_bringup save_map.py status')
    return wait_for_save(job) if args.wait else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('Map save error: ' + str(exc), file=sys.stderr)
        sys.exit(2)

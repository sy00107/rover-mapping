"""Read-only IMU sender preflight. Does not open SPI or send motor commands."""
import argparse
import json
from pathlib import Path


def bridge_processes(socket_path,proc_root=Path('/proc')):
    result=[]
    for folder in proc_root.iterdir():
        if not folder.name.isdigit():continue
        try:
            args=folder.joinpath('cmdline').read_bytes().decode(errors='replace').split('\x00')
        except (OSError,PermissionError):continue
        for i,arg in enumerate(args):
            if ((arg=='--telemetry-socket' and i+1<len(args) and args[i+1]==socket_path)
                    or arg=='--telemetry-socket='+socket_path):
                result.append(int(folder.name));break
    return sorted(result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',default='/dev/shm/rover-mapping-imu.sock')
    parser.add_argument('--require-source',action='store_true')
    args=parser.parse_args();pids=bridge_processes(args.socket)
    if args.require_source and not pids:
        parser.exit(2,'IMU 브리지가 실행 중이 아닙니다. 노트북 터미널 1에서 '
                    'bash tools/mapping_imu_bridge.sh pi@raspberrypi.local 로 Pi 독립 브리지를 시작하세요.\n')
    if not args.require_source:
        print(json.dumps(dict(bridge_pids=pids,bridge_running=bool(pids),
            receiver_socket_exists=Path(args.socket).exists(),socket=args.socket),ensure_ascii=False))


if __name__=='__main__':main()

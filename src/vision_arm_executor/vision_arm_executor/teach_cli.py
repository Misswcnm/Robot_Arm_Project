"""Local operator CLI for the stateful ICP A/B drag-teaching workflow."""

import argparse
import json
import os
import sys

from .nx_gateway import VisionRpcClient


def _show(result):
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return result.get('status') == 'succeeded'


def _send(rpc, command, frames, identity=None):
    params = {'command': command, 'point_type': 1}
    if str(command).lower() in {'0', 'start'} and identity:
        params.update(identity)
    if str(command).lower() in {'1', 'a', 'record_a'}:
        params['frames'] = int(frames)
    result = rpc.run(
        'vision_point_teach', params=params, timeout_sec=30.0,
        dry_run=False)
    return _show(result)


def _interactive(rpc, frames, identity):
    print('ICP A/B 本地拖拽示教')
    print('  A: 记录点云模板和当前法兰位姿')
    print('  B: 记录戳点法兰位姿、保存 A→B，并自动退出拖拽。')
    print('  command 3 仅用于中途取消/异常恢复。')
    answer = input('确认机械臂周围安全，开始拖拽示教？[y/N] ').strip().lower()
    if answer not in {'y', 'yes'}:
        print('已取消。')
        return 0
    active = False
    try:
        if not _send(rpc, 'start', frames, identity):
            return 1
        active = True
        print('已进入拖拽：移动到 A 后按 1，移动到戳点 B 后按 2。')
        while True:
            command = input(
                'icp-teach [1=A, 2=B并完成, 3=取消, s=状态, q=退出] > '
            ).strip().lower()
            if command == '1':
                _send(rpc, 1, frames)
            elif command == '2':
                if _send(rpc, 2, frames):
                    active = False
                    print(
                        '当前点已落盘并退出拖拽。继续示教下一个 P 点时，'
                        '重新运行本命令即可。')
                    return 0
            elif command == '3':
                if _send(rpc, 3, frames):
                    active = False
                    print('示教已取消并恢复机械臂使能。')
                    return 0
            elif command in {'s', 'status'}:
                _send(rpc, 'status', frames)
            elif command in {'q', 'quit', 'exit'}:
                return 0
            else:
                print('请输入 1、2、3、s 或 q。')
    except (EOFError, KeyboardInterrupt):
        print('\n收到退出请求。')
        return 130
    finally:
        if active:
            print('正在退出拖拽并恢复 Enable/User0/Tool0...')
            _send(rpc, 'finish', frames)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='通过本机 vision RPC 执行 ICP A/B 拖拽示教')
    parser.add_argument(
        '--host', default=os.environ.get('VISION_RPC_HOST', '127.0.0.1'))
    parser.add_argument(
        '--port', type=int,
        default=int(os.environ.get('VISION_RPC_PORT', '17881')))
    parser.add_argument(
        '--auth-token', default=os.environ.get('VISION_RPC_AUTH_TOKEN', ''))
    parser.add_argument('--frames', type=int, default=5)
    parser.add_argument('--mapid', help='小车地图 ID')
    parser.add_argument('--poseid', help='小车导航点 ID')
    parser.add_argument(
        '--label',
        help='该导航点内的动作标签，例如 P1；省略时自动分配下一个 P 序号')
    parser.add_argument(
        '--command',
        help=(
            '单步调用：start/0、record_a/1、record_b/2（自动完成）、'
            'finish/3（取消恢复）、status/4'))
    args = parser.parse_args(argv)
    rpc = VisionRpcClient(
        args.host, args.port, auth_token=args.auth_token,
        socket_timeout=5.0)
    identity = {
        'mapid': args.mapid,
        'poseid': args.poseid,
        'label': args.label or '',
    }
    if args.command is not None:
        if str(args.command).lower() in {'0', 'start'}:
            if not args.mapid or not args.poseid:
                parser.error('--mapid and --poseid are required for start')
        return 0 if _send(
            rpc, args.command, args.frames, identity) else 1
    if not args.mapid or not args.poseid:
        parser.error('--mapid and --poseid are required in interactive mode')
    return _interactive(rpc, args.frames, identity)


if __name__ == '__main__':
    sys.exit(main())

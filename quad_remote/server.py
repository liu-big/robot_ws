#!/usr/bin/env python3
"""QUAD mobile remote: aiohttp WebSocket + ROS2 latest-value bridge."""
import argparse
import asyncio
from contextlib import suppress
import json
import math
from pathlib import Path
import time

from aiohttp import web, WSMsgType
from core import Latest, MODES, finite

ROOT = Path(__file__).resolve().parent
SPEED_SCALE_MAX = 1.5
LATEST = web.AppKey('latest', Latest)
CLIENTS = web.AppKey('clients', dict)
CONTROLLER = web.AppKey('controller', object)
DEMO = web.AppKey('demo', bool)


async def socket_handler(request):
    # One control page at a time; additional pages are status viewers.
    ws = web.WebSocketResponse(heartbeat=30, max_msg_size=2048, compress=False)
    await ws.prepare(request)
    app = request.app
    latest = app[LATEST]
    app[CLIENTS][ws] = None
    app[CONTROLLER] = ws
    await ws.send_json(dict(type='hello', limits=latest.limits,
                            base_limits=app.get('base_limits', latest.limits),
                            control_hz=50, state_hz=5, demo=app[DEMO],
                            speed_min=0.4, speed_max=SPEED_SCALE_MAX,
                            min_body=0.4, min_yaw=0.4))

    tick = 0

    async def send_pong(t, rtt):
        if ws.closed:
            return
        try:
            await ws.send_json(dict(type='pong', t=t))
            if finite(rtt):
                app[CLIENTS][ws] = max(0, min(10000, round(rtt)))
        except (ConnectionError, RuntimeError):
            pass

    async def send_state():
        nonlocal tick
        while not ws.closed:
            tick += 1
            state = latest.snapshot(full=tick % 20 == 0)
            state.update(can_control=app[CONTROLLER] is ws, ping=app[CLIENTS].get(ws))
            try:
                await ws.send_str(json.dumps(state, separators=(',', ':'), allow_nan=False))
            except (ConnectionError, RuntimeError):
                break
            await asyncio.sleep(0.2)

    sender = asyncio.create_task(send_state())

    def sender_done(task):
        if not task.cancelled() and task.exception():
            asyncio.create_task(ws.close(code=1013, message=b'State transport stalled'))
    sender.add_done_callback(sender_done)
    try:
        async for message in ws:
            if message.type != WSMsgType.TEXT:
                if message.type == WSMsgType.ERROR:
                    break
                continue
            try:
                data = json.loads(message.data)
                if not isinstance(data, dict):
                    raise ValueError('expected object')
                kind = data.get('type')
                if kind == 'ping' and finite(data.get('t')):
                    asyncio.create_task(send_pong(data['t'], data.get('rtt')))
                elif kind == 'stop':
                    if app[CONTROLLER] is ws:
                        latest.stop()
                elif kind == 'odom_reset':
                    if app[CONTROLLER] is ws:
                        latest.request_odom_reset()
                elif kind in ('drive', 'pose'):
                    if app[CONTROLLER] is ws:
                        if kind == 'drive':
                            latest.set_drive(data)
                        else:
                            latest.pose(data.get('mode'))
                else:
                    raise ValueError('unknown message type')
            except (ValueError, TypeError, OverflowError):
                if app[CONTROLLER] is ws:
                    latest.stop()
                await ws.close(code=1007, message=b'Invalid control message')
                break
    finally:
        sender.cancel()
        with suppress(asyncio.CancelledError, TimeoutError, ConnectionError, RuntimeError):
            await sender
        app[CLIENTS].pop(ws, None)
        if app[CONTROLLER] is ws:
            latest.stop()
            app[CONTROLLER] = next(iter(app[CLIENTS]), None)
    return ws


async def demo_loop(app):
    """Explicit demo mode. No rclpy import and no hardware access."""
    from types import SimpleNamespace
    latest = app[LATEST]
    latest.set_mode('neutral')
    latest.set_graph(True)
    odom_pose = dict(x=0.0, y=0.0, yaw=0.0)
    while True:
        drive_out, pose = latest.tick()
        if pose:
            latest.set_mode(pose)
        state = latest.published_drive
        latest.feedback('joints', SimpleNamespace(
            name=[f'leg{leg}_{axis}' for leg in range(1, 5) for axis in ('lift', 'crouch')],
            position=[0.12, 0.28] * 4, velocity=[0.0] * 8, effort=[0.18, 0.12] * 4))
        latest.feedback('wheels', SimpleNamespace(
            name=[f'leg{leg}_wheel' for leg in range(1, 5)], position=[0.0] * 4,
            velocity=[state.vx - state.vy - state.wz * .25,
                      state.vx + state.vy + state.wz * .25,
                      state.vx + state.vy - state.wz * .25,
                      state.vx - state.vy + state.wz * .25], effort=[0.0] * 4))
        dt = 0.02
        odom_pose['yaw'] += state.wz * dt
        odom_pose['x'] += (state.vx * math.cos(odom_pose['yaw']) - state.vy * math.sin(odom_pose['yaw'])) * dt
        odom_pose['y'] += (state.vx * math.sin(odom_pose['yaw']) + state.vy * math.cos(odom_pose['yaw'])) * dt
        half_yaw = odom_pose['yaw'] / 2.0
        latest.set_odom(SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=odom_pose['x'], y=odom_pose['y'], z=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw)),
                )),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=state.vx, y=state.vy, z=0.0),
                    angular=SimpleNamespace(x=0.0, y=0.0, z=state.wz),
                )),
        ))
        await asyncio.sleep(0.02)


def make_app(latest=None, demo=False, use_ros=True, base_limits=None):
    app = web.Application()
    app[LATEST] = latest or Latest()
    app['base_limits'] = base_limits or dict((latest or Latest()).limits)
    app[CLIENTS] = {}
    app[CONTROLLER] = None
    app[DEMO] = demo

    async def lifetime(app):
        runtime = None
        task = None
        if demo:
            task = asyncio.create_task(demo_loop(app))
        elif use_ros:
            from ros_bridge import RosRuntime
            runtime = RosRuntime(app[LATEST])
        yield
        app[LATEST].stop()
        if task:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        if runtime:
            runtime.close()

    async def close_clients(app):
        for client in list(app[CLIENTS]):
            await client.close(code=1001, message=b'Bridge shutdown')

    async def asset(request):
        filename = request.match_info.get('asset', 'index.html')
        if filename not in ('index.html', 'app.js', 'style.css'):
            raise web.HTTPNotFound()
        return web.FileResponse(ROOT / 'static' / filename, headers={'Cache-Control': 'no-cache'})

    async def health(request):
        return web.json_response(dict(ok=True, demo=demo, **app[LATEST].snapshot()))

    app.cleanup_ctx.append(lifetime)
    app.on_shutdown.append(close_clients)
    app.router.add_get('/ws', socket_handler)
    app.router.add_get('/health', health)
    app.router.add_get('/', asset)
    app.router.add_get('/{asset}', asset)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--workspace', type=Path, default=Path('/home/a/robot_ws'))
    parser.add_argument('--demo', action='store_true', help='simulated state; never loads ROS')
    args = parser.parse_args()
    limits, base_limits, active = dict(vx=0.5, vy=0.5, wz=1.0), dict(vx=0.5, vy=0.5, wz=1.0), [1, 2]
    if not args.demo:
        import yaml
        with (args.workspace / 'config/quad_teleop_params.yaml').open() as stream:
            config = yaml.safe_load(stream)['quad_teleop_node']['ros__parameters']
        for target, source in [('vx', 'max_vx'), ('vy', 'max_vy'), ('wz', 'max_omega')]:
            value = config.get(source, limits[target])
            if not finite(value) or value <= 0:
                parser.error('invalid velocity limit: ' + source)
            base_limits[target] = value
            limits[target] = value * SPEED_SCALE_MAX
        with (args.workspace / 'config/c620_quad_params.yaml').open() as stream:
            active = yaml.safe_load(stream)['c620_quad_node']['ros__parameters']['active_wheels']
    print('QUAD remote · ' + ('DEMO (no hardware)' if args.demo else 'ROS2 /cmd_vel bridge'), flush=True)
    web.run_app(make_app(Latest(limits, active), demo=args.demo, base_limits=base_limits),
                host=args.host, port=args.port,
                access_log=None, shutdown_timeout=2)


if __name__ == '__main__':
    main()

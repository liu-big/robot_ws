import asyncio
import time
import unittest
from aiohttp import ClientSession, WSMsgType
from aiohttp.test_utils import TestServer
from async_timeout import timeout
from server import make_app, LATEST


class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.app = make_app(demo=True)
        self.server = TestServer(self.app)
        await self.server.start_server()
        self.session = ClientSession()

    async def asyncTearDown(self):
        await self.session.close()
        await self.server.close()

    async def state(self, ws, predicate=lambda state: True):
        async with timeout(3):
            while True:
                value = await ws.receive_json()
                if value.get('type') == 'state' and predicate(value):
                    return value

    async def test_static_routes_and_no_source_exposure(self):
        for path in ('/', '/app.js', '/style.css', '/health'):
            async with self.session.get(self.server.make_url(path)) as response:
                self.assertEqual(response.status, 200)
        async with self.session.get(self.server.make_url('/server.py')) as response:
            self.assertEqual(response.status, 404)

    async def test_state_rate_pose_and_watchdog(self):
        ws = await self.session.ws_connect(self.server.make_url('/ws'))
        hello = await ws.receive_json()
        self.assertEqual(hello['control_hz'], 50)
        await self.state(ws)
        stamps = []
        for _ in range(21):
            await self.state(ws); stamps.append(time.monotonic())
        hz = 20 / (stamps[-1] - stamps[0])
        self.assertTrue(4 < hz < 7, hz)
        await ws.send_json(dict(type='drive', vx=0, vy=0, wz=.3))
        moving = await self.state(ws, lambda s: s['wz'] == .3)
        self.assertAlmostEqual(moving['wz'], .3)
        await self.state(ws, lambda s: s['wz'] == 0)
        await ws.send_json(dict(type='pose', mode='stand'))
        await self.state(ws, lambda s: s['mode'] == 'stand')
        await ws.close()
        print(f'WebSocket telemetry: {hz:.1f} Hz; pose and stale-command stop verified')

    async def test_disconnect_stops_and_returns_control(self):
        first = await self.session.ws_connect(self.server.make_url('/ws'))
        self.assertTrue((await self.state(first))['can_control'])
        await first.send_json(dict(type='drive', vx=0, vy=0, wz=-.5))
        await self.state(first, lambda s: s['wz'] == -.5)
        await first.close()
        second = await self.session.ws_connect(self.server.make_url('/ws'))
        await self.state(second, lambda s: s['can_control'] and s['wz'] == 0)
        await second.close()

    async def test_invalid_control_closes_socket_and_stops(self):
        ws = await self.session.ws_connect(self.server.make_url('/ws'))
        await self.state(ws)
        await ws.send_json(dict(type='drive', vx=0, vy=0, wz=.2))
        await self.state(ws, lambda s: s['wz'] == .2)
        await ws.send_str('{"type":"drive","vx":NaN,"vy":0,"wz":0}')
        while (await ws.receive()).type not in (WSMsgType.CLOSE, WSMsgType.CLOSED):
            pass
        await asyncio.sleep(.05)
        self.assertEqual(self.app[LATEST].snapshot()['wz'], 0)


if __name__ == '__main__':
    unittest.main()

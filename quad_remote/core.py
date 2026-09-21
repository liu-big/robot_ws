"""Latest-value storage shared by the WebSocket loop and ROS executor."""
from dataclasses import dataclass
import math
import threading
import time

MODES = ('stand', 'neutral', 'crouch')
WHEELS = ('FL', 'FR', 'RR', 'RL')


@dataclass(frozen=True)
class DriveCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def as_dict(self):
        return dict(vx=self.vx, vy=self.vy, wz=self.wz)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class Latest:
    def __init__(self, limits=None, active_wheels=(1, 2), timeout=1.2, clock=time.monotonic):
        self.lock = threading.Lock()
        self.clock = clock
        defaults = dict(vx=0.8, vy=0.8, wz=1.5)
        self.limits = {**defaults, **(limits or {})}
        self.active_wheels = set(active_wheels)
        self.timeout = timeout
        self.latest_drive = None
        self.drive_at = 0.0
        self.stop_pending = False
        self.pending_pose = None
        self.published_drive = DriveCommand()
        self.mode = 'unknown'
        self.graph_ready = False
        self.joint_values = {}
        self.wheel_values = {}
        self.joint_fb_at = self.wheel_fb_at = self.odom_fb_at = float('-inf')
        self.odom = None
        self.odom_reset_pending = False
        self.publish_count = 0
        self.receive_count = 0

    def set_drive(self, data):
        if not all(finite(data.get(k)) for k in ('vx', 'vy', 'wz')):
            raise ValueError('drive requires finite numeric vx, vy and wz')
        command = DriveCommand(**{
            k: max(-self.limits[k], min(self.limits[k], data[k])) for k in ('vx', 'vy', 'wz')
        })
        with self.lock:
            if command == DriveCommand() and self.latest_drive is None:
                return
            self.latest_drive = command
            self.drive_at = self.clock()
            self.receive_count += 1

    def pose(self, name):
        if name not in MODES:
            raise ValueError('unknown pose')
        with self.lock:
            self.pending_pose = name
            self.mode = name
            self.stop_pending |= self.latest_drive is not None
            self.latest_drive = None

    def stop(self):
        with self.lock:
            self.stop_pending |= self.latest_drive is not None or self.published_drive != DriveCommand()
            self.latest_drive = None
            self.pending_pose = None

    def tick(self):
        with self.lock:
            pose, self.pending_pose = self.pending_pose, None
            drive = self.latest_drive
            timed_out = False
            if drive is not None and self.clock() - self.drive_at > self.timeout:
                drive = None
                self.latest_drive = None
                timed_out = True

            drive_out = None
            if drive is not None:
                self.published_drive = drive
                drive_out = drive
                self.publish_count += 1
            elif timed_out or (self.stop_pending and self.published_drive != DriveCommand()):
                self.published_drive = DriveCommand()
                drive_out = DriveCommand()
                self.publish_count += 1

            if self.stop_pending:
                self.stop_pending = False

            return drive_out, pose

    def set_mode(self, mode):
        if mode in (*MODES, 'estop'):
            with self.lock:
                self.mode = mode
            if mode == 'estop':
                self.stop()

    def set_graph(self, ready):
        with self.lock:
            self.graph_ready = ready

    def set_odom(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        x = pose.position.x
        y = pose.position.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        if not all(finite(v) for v in (x, y, qz, qw)):
            return
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        dist = math.hypot(x, y)
        with self.lock:
            self.odom = dict(
                x=round(x, 3), y=round(y, 3), yaw=round(yaw, 3), dist=round(dist, 3),
                vx=round(twist.linear.x, 3) if finite(twist.linear.x) else 0.0,
                vy=round(twist.linear.y, 3) if finite(twist.linear.y) else 0.0,
                wz=round(twist.angular.z, 3) if finite(twist.angular.z) else 0.0,
            )
            self.odom_fb_at = self.clock()

    def request_odom_reset(self):
        with self.lock:
            self.odom_reset_pending = True

    def consume_odom_reset(self):
        with self.lock:
            pending = self.odom_reset_pending
            self.odom_reset_pending = False
            return pending

    def feedback(self, kind, msg):
        values = {}
        for index, name in enumerate(msg.name):
            item = {'name': name}
            for field in ('position', 'velocity', 'effort'):
                source = getattr(msg, field)
                value = source[index] if index < len(source) else None
                item[field] = round(value, 5) if finite(value) else None
            values[name] = item
        with self.lock:
            if kind == 'joints':
                self.joint_values = values
                self.joint_fb_at = self.clock()
            elif kind == 'wheels':
                self.wheel_values = values
                self.wheel_fb_at = self.clock()

    def snapshot(self, full=False):
        with self.lock:
            now = self.clock()
            joint_fresh = now - self.joint_fb_at < 1.0
            wheel_fresh = now - self.wheel_fb_at < 1.0
            odom_fresh = now - self.odom_fb_at < 1.0
            state = dict(
                type='state', mode=self.mode, **self.published_drive.as_dict(),
                ros_connected=self.graph_ready, teleop_connected=self.graph_ready,
                odom_ok=odom_fresh and self.odom is not None)
            if self.odom:
                state.update(self.odom)
            if not full:
                return state
            joints, wheels = [], []
            for leg in range(1, 5):
                for axis in ('lift', 'crouch'):
                    name = f'leg{leg}_{axis}'
                    value = self.joint_values.get(name)
                    joints.append(dict(name=name, label=f'L{leg} {axis.title()}',
                                       position=value['position'] if value else None,
                                       velocity=value['velocity'] if value else None,
                                       effort=value['effort'] if value else None,
                                       status='receiving' if value and joint_fresh else 'stale' if value else 'unavailable'))
                name = f'leg{leg}_wheel'
                value = self.wheel_values.get(name)
                status = ('receiving' if wheel_fresh else 'stale') if value else ('unavailable' if leg in self.active_wheels else 'disabled')
                wheels.append(dict(name=name, label=WHEELS[leg - 1],
                                   position=value['position'] if value else None,
                                   velocity=value['velocity'] if value else None,
                                   effort=value['effort'] if value else None, status=status))
            state['joints'] = joints
            state['wheels'] = wheels
            return state

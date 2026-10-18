#!/usr/bin/env python3
"""
KSRC 스워브 로버용 화면 마우스 조이스틱 텔레옵.

패드 둘:
  왼쪽  (마우스 드래그) -> 병진 (vx, vy)
  오른쪽(마우스 드래그) -> 회전 (omega), 가로축만

키보드 / 화면 8방향 패드:
  ↑ 또는 Space: 전진, ↓: 후진, ← 또는 Q: 좌회전, → 또는 E: 우회전.
  전후 + 좌우를 함께 누르면 차체 회전 없이 45° 방향으로 평행 이동한다.
  좌우 단독 입력은 제자리 회전이다. 대각선에서도 전체 이동 속도는 선택값을 유지한다.
  화면의 8방향 버튼도 누르는 동안 같은 명령을 보낸다.
  1/2/3 또는 화면 속도 버튼으로 전후 속도 배율을 선택한다.
  회전은 --max-ang의 50%다. 키보드는 마우스 패드보다 우선한다.
  키/버튼을 떼면 해당 명령이 해제된다. 포커스를 잃으면 모두 정지하며,
  돌아와도 새 키 입력이나 클릭이 있어야 움직인다.

와이어 포맷 두 가지 (--format):
  ascii  -> "V <vx> <vy> <omega>\\n" 줄. STM32 펌웨어의 터미널 파서가 그대로
            이해한다 (펌웨어 수정 불필요). 실제 로버는 이걸 쓴다.
  binary -> common/nucleo_link.py 의 프레임 프로토콜. 시뮬레이터의 가상
            Nucleo 가 쓰는 포맷.

  udp    -> (--udp HOST) PC 에서 창을 띄우고 vx, vy, ω 를 UDP 로 라파에 보낸다. 라파의
            pi/common/udp_spi_bridge.py 가 받아서 SPI 로 Nucleo 에 넘긴다. 라파가 헤드리스라
            이게 실주행 기본 경로 (ssh X 포워딩은 SDL MIT-SHM 오류로 창이 안 뜸).
            --monitor 는 라파가 돌려주는 텔레메트리 요약을 찍는다.
  spi    -> (--spi) 이 프로그램 자체를 라파에서 돌려 SPI 로 직접 보낸다 (라파에 화면이 있을 때).

사용법:
  # 라파: python3 pi/common/udp_spi_bridge.py
  python3 teleop_joystick.py --udp raspberrypi.local --rate 50 --max-lin 0.15 --max-ang 0.8 --monitor
  python3 teleop_joystick.py --port /dev/ttyACM0                 # 실제 로버 (USB 시리얼)
  python3 teleop_joystick.py --port /dev/pts/N --format binary   # 시뮬레이터
  python3 teleop_joystick.py --ros --max-lin 0.25 --max-ang 0.8 # ROS2 대회맵
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # pi/ -> import common.*
from common.nucleo_link import (TLM_FLAG_GATE_PENDING, TLM_FLAG_WATCHDOG,
                                encode_velocity_cmd)
from joystick_math import (apply_deadzone, clamp_to_unit_circle, left_pad_to_linear,
                           lock_low_speed_direction, right_pad_to_angular, snap_to_axes)

WIDTH, HEIGHT = 940, 450
PAD_RADIUS = 100
LEFT_CENTER = (170, 190)
RIGHT_CENTER = (470, 190)
GRAB_SLACK = 1.4  # click within radius*GRAB_SLACK of center still grabs the pad
ROTATION_SPEED_SCALE = 0.5  # 모든 회전 입력은 같은 50% 제한
FORWARD_SPEED_CHOICES = (1, 2, 3)
SPEED_BUTTON_RECTS = ((16, 334, 192, 32), (224, 334, 192, 32), (432, 334, 192, 32))

# (label, forward, side): side is left/right translation in a diagonal, yaw when alone.
DIRECTION_BUTTONS = (
    ("F+L", 1, 1), ("FWD", 1, 0), ("F+R", 1, -1),
    ("LEFT", 0, 1), ("STOP", 0, 0), ("RIGHT", 0, -1),
    ("B+L", -1, 1), ("BACK", -1, 0), ("B+R", -1, -1),
)
DIRECTION_RECTS = tuple((650 + col*90, 105 + row*62, 84, 56)
                        for row in range(3) for col in range(3))


def drive_direction(forward, side, linear_speed, angular_speed):
    """Diagonal = 45-degree crab translation; left/right alone = in-place yaw."""
    if forward:
        scale = linear_speed/math.hypot(forward, side)
        return forward*scale, side*scale, 0.0
    return 0.0, 0.0, side*angular_speed*ROTATION_SPEED_SCALE


class Pad:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.dragging = False
        self.nx = 0.0
        self.ny = 0.0

    def contains(self, pos):
        dx = pos[0] - self.center[0]
        dy = pos[1] - self.center[1]
        return math.hypot(dx, dy) <= self.radius * GRAB_SLACK

    def start_drag(self, pos):
        self.dragging = True
        self.update(pos)

    def update(self, pos):
        dx = pos[0] - self.center[0]
        dy = pos[1] - self.center[1]
        self.nx, self.ny = clamp_to_unit_circle(dx, dy, self.radius)

    def release(self):
        self.dragging = False
        self.nx = 0.0
        self.ny = 0.0

    def handle_pixel_pos(self):
        return (self.center[0] + self.nx * self.radius, self.center[1] + self.ny * self.radius)


def make_frame_encoder(fmt):
    """Returns encode(vx, vy, omega) -> bytes for the chosen wire format."""
    if fmt == "ascii":
        # + 0.0 : 패드 중앙의 -0.0 이 "V -0.000" 으로 찍혀 헷갈리지 않게
        return lambda vx, vy, omega: f"V {vx + 0.0:.3f} {vy + 0.0:.3f} {omega + 0.0:.3f}\n".encode("ascii")
    return encode_velocity_cmd


class RosSink:
    """Publish the same body twist as the serial teleop to the MuJoCo bridge."""

    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import Twist

        # Ctrl+C 를 rclpy 가 가로채면 컨텍스트가 먼저 닫혀서, 종료 시 보내는 정지
        # 명령이 "publisher's context is invalid" 로 실패한다 (로버가 마지막 명령으로
        # 계속 감). 신호 처리를 파이썬에 맡겨 finally 의 정지 명령이 나가게 한다.
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self.rclpy = rclpy
        self.Twist = Twist
        self.node = rclpy.create_node("ksrc_teleop_joystick")
        self.pub = self.node.create_publisher(Twist, topic, 10)

    def send(self, vx, vy, omega):
        if not self.rclpy.ok():
            return
        msg = self.Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)
        self.pub.publish(msg)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def run(args):
    import pygame  # local import so this module stays importable headlessly for tooling

    # sim/test_e2e_teleop.py 는 이 필드 없이 자기 Args 객체를 만들어 가상
    # Nucleo 를 구동하는데, 그쪽은 바이너리 프로토콜을 쓴다. 따라서 속성이
    # 없으면 "binary" 를 뜻해야 한다.
    demo_mode = getattr(args, "demo", False)
    ros_mode = getattr(args, "ros", False) and not demo_mode
    spi_mode = getattr(args, "spi", False) and not demo_mode
    last_tlm = None   # --spi: 가장 최근 텔레메트리
    n_spi_bad = 0
    if demo_mode:
        send = lambda vx, vy, omega: None
    elif ros_mode:
        sink = RosSink(getattr(args, "topic", "/cmd_vel"))
        send = sink.send
    elif getattr(args, "udp", None):
        import json
        import socket
        import struct
        host, _, port = args.udp.partition(":")
        dest = (socket.gethostbyname(host), int(port or 5005))
        usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        usock.setblocking(False)
        pkt = struct.Struct("<4sIfff")
        useq = 0
        print(f"[udp] -> {dest[0]}:{dest[1]} (라파에서 udp_spi_bridge.py 실행 중이어야 함)")

        def send(vx, vy, omega):
            nonlocal useq, last_tlm
            useq = (useq + 1) & 0xFFFFFFFF
            try:
                usock.sendto(pkt.pack(b"KSRC", useq, vx, vy, omega), dest)
            except OSError:
                pass
            while True:   # 라파가 돌려준 텔레메트리 요약 (5Hz)
                try:
                    data, _ = usock.recvfrom(2048)
                except (BlockingIOError, OSError):
                    break
                try:
                    last_tlm = json.loads(data)
                except ValueError:
                    pass
    elif spi_mode:
        from common.nucleo_link import SpiLink
        link = SpiLink(args.spi_bus, args.spi_dev, args.spi_speed)

        def send(vx, vy, omega):
            nonlocal last_tlm, n_spi_bad
            tlm = link.exchange(vx, vy, omega)
            if tlm is None:
                n_spi_bad += 1
            else:
                last_tlm = tlm
    else:
        import serial
        encode = make_frame_encoder(getattr(args, "format", "binary"))
        ser = serial.Serial(args.port, args.baud, timeout=0)
        send = lambda vx, vy, omega: ser.write(encode(vx, vy, omega))

    # --monitor: STM 이 받은 값을 되돌려 찍게 하고(CTRL ON), 보낸 값과 나란히 출력한다.
    udp_mode = bool(getattr(args, "udp", None)) and not demo_mode
    tlm_mode = spi_mode or udp_mode   # 텔레메트리를 직접 받는 모드 (시리얼 CTRL 불필요)
    monitor = getattr(args, "monitor", False) and not ros_mode and not demo_mode
    rx_buf = b""
    next_mon = time.monotonic()
    if monitor and tlm_mode:
        print("[monitor] PI > 는 보낸 값, STM> 는 STM 이 실제 적용한 명령·조향·duty (SPI 텔레메트리, 5Hz)")
    elif monitor:
        ser.write(b"CTRL ON\n")
        print("[monitor] PC > 는 보낸 값, STM> 는 펌웨어가 받은 값 (텔레옵 시작 후 5Hz)")

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("KSRC Controller — DEMO (no robot output)" if demo_mode else "KSRC swerve teleop (joystick)")
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()

    left_pad = Pad(LEFT_CENTER, PAD_RADIUS)
    right_pad = Pad(RIGHT_CENTER, PAD_RADIUS)
    speed_buttons = [pygame.Rect(rect) for rect in SPEED_BUTTON_RECTS]
    speed_keys = {pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3,
                  pygame.K_KP1: 1, pygame.K_KP2: 2, pygame.K_KP3: 3}

    period = 1.0 / args.rate
    next_send = time.monotonic()
    last_dir = None   # 직전에 내보낸 병진 방향 (저속 방향 고정용)
    running = True
    held_keys = set()
    motion_keys = {pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT,
                   pygame.K_SPACE, pygame.K_q, pygame.K_e}
    direction_buttons = [pygame.Rect(rect) for rect in DIRECTION_RECTS]
    mouse_direction = None
    controls_focused = True
    forward_multiplier = 1

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in speed_keys:
                    if controls_focused:
                        forward_multiplier = speed_keys[event.key]
                elif event.type == pygame.KEYDOWN and event.key in motion_keys:
                    if controls_focused and not getattr(event, "repeat", False):
                        held_keys.add(event.key)
                        if event.key in {pygame.K_UP, pygame.K_SPACE, pygame.K_DOWN}:
                            left_pad.release()
                        right_pad.release()
                        mouse_direction = None
                        last_dir = None
                elif event.type == pygame.KEYUP and event.key in motion_keys:
                    held_keys.discard(event.key)
                elif (event.type == pygame.MOUSEBUTTONDOWN and event.button == 1
                      and controls_focused):
                    selected = next((mult for mult, rect in zip(FORWARD_SPEED_CHOICES, speed_buttons)
                                     if rect.collidepoint(event.pos)), None)
                    if selected is not None:
                        forward_multiplier = selected
                    elif any(rect.collidepoint(event.pos) for rect in direction_buttons):
                        index = next(i for i, rect in enumerate(direction_buttons) if rect.collidepoint(event.pos))
                        mouse_direction = DIRECTION_BUTTONS[index][1:]
                        held_keys.clear()
                        left_pad.release()
                        right_pad.release()
                        last_dir = None
                    elif left_pad.contains(event.pos):
                        mouse_direction = None
                        left_pad.start_drag(event.pos)
                    elif right_pad.contains(event.pos):
                        mouse_direction = None
                        right_pad.start_drag(event.pos)
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    mouse_direction = None
                    left_pad.release()
                    right_pad.release()
                elif event.type == pygame.MOUSEMOTION and controls_focused:
                    if left_pad.dragging:
                        left_pad.update(event.pos)
                    if right_pad.dragging:
                        right_pad.update(event.pos)
                elif (event.type == pygame.WINDOWFOCUSLOST or
                      (event.type == pygame.ACTIVEEVENT and not event.gain and
                       event.state & (pygame.APPINPUTFOCUS | pygame.APPACTIVE))):
                    controls_focused = False
                    held_keys.clear()
                    mouse_direction = None
                    left_pad.release()
                    right_pad.release()
                    last_dir = None
                elif (event.type == pygame.WINDOWFOCUSGAINED or
                      (event.type == pygame.ACTIVEEVENT and event.gain and
                       event.state & (pygame.APPINPUTFOCUS | pygame.APPACTIVE))):
                    controls_focused = True

            if held_keys & {pygame.K_UP, pygame.K_SPACE, pygame.K_DOWN}:
                left_pad.release()
                right_pad.release()
                last_dir = None
            elif held_keys:
                right_pad.release()

            dz = getattr(args, "deadzone", 0.0)
            lnx, lny = apply_deadzone(left_pad.nx, left_pad.ny, dz)
            lnx, lny = snap_to_axes(lnx, lny, getattr(args, "snap_deg", 0.0))
            lnx, lny, last_dir = lock_low_speed_direction(
                lnx, lny, last_dir, getattr(args, "hold_mag", 0.0),
                getattr(args, "hold_deg", 180.0))
            vx, vy = left_pad_to_linear(lnx, lny, args.max_lin)
            if right_pad.dragging:
                rnx, _ = apply_deadzone(right_pad.nx, 0.0, dz)
                omega = right_pad_to_angular(rnx, args.max_ang)
            else:
                omega = 0.0
            omega *= ROTATION_SPEED_SCALE
            if held_keys:
                forward = int(bool(held_keys & {pygame.K_UP, pygame.K_SPACE})) - int(pygame.K_DOWN in held_keys)
                turn = int(bool(held_keys & {pygame.K_LEFT, pygame.K_q})) - int(bool(held_keys & {pygame.K_RIGHT, pygame.K_e}))
                if held_keys & {pygame.K_UP, pygame.K_SPACE, pygame.K_DOWN}:
                    vx, vy, omega = drive_direction(forward, turn, args.max_lin*forward_multiplier, args.max_ang)
                else:
                    # Keep the original mouse translation + Q/E rotation controls.
                    omega = args.max_ang * ROTATION_SPEED_SCALE * turn
            elif mouse_direction is not None:
                forward, side = mouse_direction
                vx, vy, omega = drive_direction(forward, side, args.max_lin*forward_multiplier, args.max_ang)
            if not controls_focused or not running:
                vx = vy = omega = 0.0

            now = time.monotonic()
            if now >= next_send:
                send(vx, vy, omega)
                next_send = now + period

            if monitor and tlm_mode:
                if now >= next_mon:
                    print(f"PI > v {vx + 0.0:+.3f} {vy + 0.0:+.3f} {omega + 0.0:+.3f}")
                    if last_tlm is not None:
                        t = last_tlm
                        f = t["flags"]
                        tags = ("W" if f & TLM_FLAG_WATCHDOG else "-") + \
                               ("G" if f & TLM_FLAG_GATE_PENDING else "-")
                        degs = t["deg"] if "deg" in t else [math.degrees(a) for a in t["steer_cmd_rad"]]
                        deg = " ".join(f"{a:+6.1f}" for a in degs)
                        duty = " ".join(f"{d:+4d}" for d in t["duty"])
                        print(f"STM> v {t['cmd'][0] + 0.0:+.3f} {t['cmd'][1] + 0.0:+.3f} {t['cmd'][2] + 0.0:+.3f} "
                              f"| deg {deg} | duty {duty} [{tags}] "
                              f"crc_bad={t.get('crc_bad', n_spi_bad)}{' STALE' if t.get('stale') else ''}")
                    else:
                        print(f"STM> (아직 유효 프레임 없음, crc_bad={n_spi_bad})")
                    next_mon = now + 0.2
            elif monitor:
                rx_buf += ser.read(4096)
                while b"\n" in rx_buf:
                    line, rx_buf = rx_buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").strip()
                    if text:
                        print(f"STM> {text}")
                if now >= next_mon:
                    print(f"PC > v {vx + 0.0:+.3f} {vy + 0.0:+.3f} {omega + 0.0:+.3f}")
                    next_mon = now + 0.2

            screen.fill((24, 24, 28))
            for pad, label in ((left_pad, "translate"), (right_pad, "rotate")):
                pygame.draw.circle(screen, (70, 70, 80), pad.center, pad.radius, 2)
                pygame.draw.circle(screen, (60, 60, 68), pad.center, pad.radius, 0)
                color = (90, 200, 255) if pad.dragging else (140, 140, 150)
                pygame.draw.circle(screen, color, [int(c) for c in pad.handle_pixel_pos()], 18)
                text = font.render(label, True, (150, 150, 160))
                screen.blit(text, (pad.center[0] - text.get_width() // 2, pad.center[1] + pad.radius + 10))

            readout = f"vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  omega={omega:+.2f} rad/s"
            help_text = "Arrows: diagonal = 45deg crab | Space = Up | Q/E = Left/Right | Esc quit"
            screen.blit(font.render(readout, True, (220, 220, 230)), (16, 16))
            screen.blit(font.render(help_text, True, (140, 140, 150)), (16, 44))
            if demo_mode:
                screen.blit(font.render("DEMO ONLY - no network / serial / motor commands", True, (245, 190, 90)), (16, 68))
            screen.blit(font.render("45deg crab / hold", True, (160, 210, 235)), (650, 76))
            for (label, forward, turn), rect in zip(DIRECTION_BUTTONS, direction_buttons):
                selected = mouse_direction == (forward, turn)
                pygame.draw.rect(screen, (35, 115, 155) if selected else (55, 55, 65), rect, border_radius=5)
                text = font.render(label, True, (230, 230, 240))
                screen.blit(text, text.get_rect(center=rect.center))
            for mult, rect in zip(FORWARD_SPEED_CHOICES, speed_buttons):
                selected = mult == forward_multiplier
                pygame.draw.rect(screen, (35, 115, 155) if selected else (55, 55, 65), rect,
                                 border_radius=5)
                label = font.render(f"{mult}x   {args.max_lin * mult:.2f} m/s", True,
                                    (245, 245, 250) if selected else (170, 170, 180))
                screen.blit(label, label.get_rect(center=rect.center))
            speed_text = (f"Drive speed: {forward_multiplier}x ({args.max_lin * forward_multiplier:.2f} m/s)"
                          " | 1 / 2 / 3 or click")
            screen.blit(font.render(speed_text, True, (160, 210, 235)), (16, 380))
            if tlm_mode:
                if last_tlm is not None:
                    c = last_tlm["cmd"]
                    stm = (f"STM got vx={c[0]:+.2f} vy={c[1]:+.2f} omega={c[2]:+.2f}  "
                           f"crc_bad={last_tlm.get('crc_bad', n_spi_bad)}")
                else:
                    stm = f"STM: no valid frame yet  crc_bad={n_spi_bad}"
                screen.blit(font.render(stm, True, (120, 220, 140)), (16, HEIGHT - 30))

            pygame.display.flip()
            clock.tick(60)
    finally:
        for _ in range(5):
            send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        if demo_mode:
            pass
        elif ros_mode:
            sink.close()
        elif getattr(args, "udp", None):
            usock.close()
        elif spi_mode:
            link.close()
        else:
            ser.close()
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="화면과 입력만 시연 · 네트워크/시리얼/SPI/ROS 출력 없음")
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial device to the Nucleo (ST-Link VCP is usually /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp", metavar="HOST[:PORT]",
                    help="PC 창 -> UDP -> 라파 udp_spi_bridge.py -> SPI (기본 포트 5005)")
    ap.add_argument("--spi", action="store_true",
                    help="시리얼 대신 Pi SPI0 -> Nucleo SPI2 로 보낸다 (라파에서 실행)")
    ap.add_argument("--spi-bus", type=int, default=0)
    ap.add_argument("--spi-dev", type=int, default=0)
    ap.add_argument("--spi-speed", type=int, default=1_000_000)
    ap.add_argument("--rate", type=float, default=20.0,
                    help="command send rate, Hz. Above ~30 the firmware's line parser "
                         "starts dropping bytes while it services a command.")
    ap.add_argument("--format", choices=("ascii", "binary"), default="ascii",
                    help="ascii: 'V vx vy omega' lines for the STM32 terminal parser "
                         "(real rover). binary: framed protocol for the simulator.")
    ap.add_argument("--ros", action="store_true",
                    help="serial 대신 ROS2 Twist 를 /cmd_vel 로 발행")
    ap.add_argument("--topic", default="/cmd_vel", help="--ros 발행 토픽")
    ap.add_argument("--max-lin", type=float, default=0.5,
                    help="pad limit and 1x keyboard/button speed in m/s; selector allows up to 3x")
    ap.add_argument("--max-ang", type=float, default=2.0,
                    help="rotation reference in rad/s; all controls send 50%% of this value")
    ap.add_argument("--deadzone", type=float, default=0.1,
                    help="패드 중심 데드존 (반경 비율). 중심 근처 손떨림이 바퀴 방향을 "
                         "홱홱 돌리지 않게 한다. 0 이면 끔")
    ap.add_argument("--monitor", action="store_true",
                    help="(직렬 모드) STM 에 CTRL ON 을 보내고, 보낸 vx vy w 와 STM 이 받은 값을 "
                         "터미널에 나란히 출력")
    ap.add_argument("--snap-deg", type=float, default=10.0,
                    help="앞·뒤·좌·우 축에서 이 각도 이내면 축으로 맞춤 (횡걸음 잔떨림 제거). 0 이면 끔")
    ap.add_argument("--hold-mag", type=float, default=0.35,
                    help="이 크기(반경 비율) 미만에서는 바퀴 축을 크게 바꾸는 방향을 0 으로 "
                         "(우->좌 전환 중 중앙을 스쳐도 조향이 돌지 않음). 0 이면 끔")
    ap.add_argument("--hold-deg", type=float, default=20.0,
                    help="--hold-mag 미만에서 허용하는 바퀴 축 변화 (도). 정반대 방향은 항상 허용")
    args = ap.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        pass   # 정지 명령은 run() 의 finally 가 이미 보냈다


if __name__ == "__main__":
    main()

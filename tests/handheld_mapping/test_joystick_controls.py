"""Exercise pygame's command loop with an in-memory sink, never robot I/O."""
import importlib.util
import itertools
import math
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def joystick(monkeypatch):
    monkeypatch.setenv('SDL_VIDEODRIVER', 'dummy')
    monkeypatch.setenv('SDL_AUDIODRIVER', 'dummy')
    directory = Path(__file__).resolve().parents[2] / 'teleop/ksrc_snapshot/pi/teleop'
    monkeypatch.syspath_prepend(str(directory))
    spec = importlib.util.spec_from_file_location('mapping_joystick', directory/'teleop_joystick.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_frames(joystick, monkeypatch, frames, snapshot_path=None):
    import pygame

    commands, closed = [], []
    class Sink:
        def __init__(self, topic):
            pass
        def send(self, vx, vy, omega):
            commands.append((vx, vy, omega))
        def close(self):
            closed.append(True)

    sequence = iter(frames)
    pressed = set()
    def events():
        nonlocal pressed
        batch, pressed = next(sequence)
        return [pygame.event.Event(kind, attrs) for kind, attrs in batch]

    monkeypatch.setattr(joystick, 'RosSink', Sink)
    monkeypatch.setattr(pygame.event, 'get', events)
    monkeypatch.setattr(pygame.key, 'get_pressed', lambda: defaultdict(bool, {k: True for k in pressed}))
    if snapshot_path is not None:
        flip = pygame.display.flip
        def save_screen():
            pygame.image.save(pygame.display.get_surface(), str(snapshot_path))
            flip()
        monkeypatch.setattr(pygame.display, 'flip', save_screen)
    ticks = itertools.count()
    monkeypatch.setattr(joystick.time, 'monotonic', lambda: float(next(ticks)))
    monkeypatch.setattr(joystick.time, 'sleep', lambda _: None)
    joystick.run(SimpleNamespace(ros=True, max_lin=.15, max_ang=.8, rate=50))
    assert closed == [True]
    assert commands[-5:] == [(0., 0., 0.)]*5
    return commands[:-5]


def test_diagonal_is_crab_and_releasing_forward_keeps_held_turn(joystick, monkeypatch):
    import pygame as p
    frames = [
        ([], set()),
        ([(p.KEYDOWN, {'key': p.K_SPACE}), (p.KEYDOWN, {'key': p.K_q})], {p.K_SPACE, p.K_q}),
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (570, 190)})], {p.K_SPACE, p.K_q}),
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (270, 190)})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], {p.K_q}),
        ([(p.KEYUP, {'key': p.K_q})], set()),
        ([(p.QUIT, {})], set()),
    ]
    assert run_frames(joystick, monkeypatch, frames) == [
        (0., 0., 0.), (.15/math.sqrt(2), .15/math.sqrt(2), 0.), (.15/math.sqrt(2), .15/math.sqrt(2), 0.), (.15/math.sqrt(2), .15/math.sqrt(2), 0.),
        (0., 0., .4), (0., 0., 0.), (0., 0., 0.),
    ]


@pytest.mark.parametrize('legacy', [False, True])
def test_focus_loss_stops_and_requires_fresh_press(joystick, monkeypatch, legacy):
    import pygame as p
    lost = (p.ACTIVEEVENT, {'gain': 0, 'state': p.APPINPUTFOCUS}) if legacy else (p.WINDOWFOCUSLOST, {})
    gained = (p.ACTIVEEVENT, {'gain': 1, 'state': p.APPINPUTFOCUS}) if legacy else (p.WINDOWFOCUSGAINED, {})
    frames = [
        ([(p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([lost], {p.K_SPACE, p.K_q}),
        ([(p.KEYDOWN, {'key': p.K_SPACE}),
          (p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (170, 90)})], {p.K_SPACE}),
        ([gained], {p.K_SPACE}),
        ([(p.KEYDOWN, {'key': p.K_SPACE, 'repeat': True})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], set()),
        ([(p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([(p.KEYDOWN, {'key': p.K_ESCAPE})], {p.K_SPACE}),
    ]
    assert run_frames(joystick, monkeypatch, frames) == [
        (.15, 0., 0.), *((0., 0., 0.),)*5, (.15, 0., 0.), (0., 0., 0.),
    ]


def test_existing_mouse_and_qe_rotation_still_work(joystick, monkeypatch):
    import pygame as p
    frames = [
        ([(p.KEYDOWN, {'key': p.K_q}), (p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (170, 90)})], {p.K_q}),
        ([(p.MOUSEBUTTONUP, {'button': 1}), (p.KEYUP, {'key': p.K_q}), (p.KEYDOWN, {'key': p.K_e})], {p.K_e}),
        ([(p.QUIT, {})], {p.K_e}),
    ]
    assert run_frames(joystick, monkeypatch, frames) == [
        (.15, 0., .4), (0., 0., -.4), (0., 0., 0.),
    ]


@pytest.mark.parametrize('keys', [('K_2', 'K_3', 'K_1'), ('K_KP2', 'K_KP3', 'K_KP1')])
def test_keyboard_speed_selection_and_live_change(joystick, monkeypatch, keys):
    import pygame as p
    two, three, one = (getattr(p, key) for key in keys)
    frames = [
        ([(p.KEYDOWN, {'key': two})], {two}),
        ([(p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([(p.KEYDOWN, {'key': three})], {p.K_SPACE}),
        ([(p.KEYDOWN, {'key': one})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], set()),
        ([(p.QUIT, {})], set()),
    ]
    commands = run_frames(joystick, monkeypatch, frames)
    assert [cmd[0] for cmd in commands] == pytest.approx([0., .30, .45, .15, 0., 0.])
    assert all(cmd[1:] == (0., 0.) for cmd in commands)


def test_click_speed_buttons_and_stop_at_three_times_speed(joystick, monkeypatch):
    import pygame as p
    frames = [
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (528, 350)})], set()),
        ([(p.MOUSEBUTTONUP, {'button': 1})], set()),
        ([(p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (320, 350)})], {p.K_SPACE}),
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (112, 350)})], {p.K_SPACE}),
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (528, 350)})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], set()),
        ([(p.QUIT, {})], set()),
    ]
    commands = run_frames(joystick, monkeypatch, frames)
    assert [cmd[0] for cmd in commands] == pytest.approx([0., 0., .45, .30, .15, .45, 0., 0.])
    assert all(cmd[1:] == (0., 0.) for cmd in commands)


def test_three_times_speed_focus_loss_and_mouse_speed_stays_original(joystick, monkeypatch):
    import pygame as p
    frames = [
        ([(p.KEYDOWN, {'key': p.K_3}),
          (p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([(p.WINDOWFOCUSLOST, {})], {p.K_SPACE}),
        ([(p.KEYDOWN, {'key': p.K_1})], {p.K_SPACE}),
        ([(p.WINDOWFOCUSGAINED, {})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], set()),
        ([(p.KEYDOWN, {'key': p.K_SPACE})], {p.K_SPACE}),
        ([(p.KEYUP, {'key': p.K_SPACE})], set()),
        ([(p.KEYDOWN, {'key': p.K_q}), (p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (170, 90)})], {p.K_q}),
        ([(p.QUIT, {})], set()),
    ]
    commands = run_frames(joystick, monkeypatch, frames)
    assert [cmd[0] for cmd in commands] == pytest.approx([.45, 0., 0., 0., 0., .45, 0., .15, 0.])
    assert commands[7][1:] == (0., .4)


@pytest.mark.parametrize('names, expected', [
    (('K_UP',), (.15, 0., 0.)), (('K_DOWN',), (-.15, 0., 0.)),
    (('K_LEFT',), (0., 0., .4)), (('K_RIGHT',), (0., 0., -.4)),
    (('K_UP', 'K_LEFT'), (.15/math.sqrt(2), .15/math.sqrt(2), 0.)),
    (('K_UP', 'K_RIGHT'), (.15/math.sqrt(2), -.15/math.sqrt(2), 0.)),
    (('K_DOWN', 'K_LEFT'), (-.15/math.sqrt(2), .15/math.sqrt(2), 0.)),
    (('K_DOWN', 'K_RIGHT'), (-.15/math.sqrt(2), -.15/math.sqrt(2), 0.)),
    (('K_SPACE', 'K_q'), (.15/math.sqrt(2), .15/math.sqrt(2), 0.)),
    (('K_SPACE', 'K_e'), (.15/math.sqrt(2), -.15/math.sqrt(2), 0.)),
    (('K_UP', 'K_SPACE'), (.15, 0., 0.)),
    (('K_UP', 'K_DOWN', 'K_LEFT', 'K_RIGHT'), (0., 0., 0.)),
])
def test_eight_directions_aliases_and_opposites(joystick, monkeypatch, names, expected):
    import pygame as p
    keys = {getattr(p, name) for name in names}
    commands = run_frames(joystick, monkeypatch, [
        ([(p.KEYDOWN, {'key': key}) for key in keys], keys),
        ([(p.KEYUP, {'key': key}) for key in keys], set()),
        ([(p.QUIT, {})], set()),
    ])
    assert commands == [expected, (0., 0., 0.), (0., 0., 0.)]


@pytest.mark.parametrize('index', range(9))
def test_screen_direction_buttons_release_to_stop(joystick, monkeypatch, index):
    import pygame as p
    x, y, w, h = joystick.DIRECTION_RECTS[index]
    _, forward, turn = joystick.DIRECTION_BUTTONS[index]
    commands = run_frames(joystick, monkeypatch, [
        ([(p.MOUSEBUTTONDOWN, {'button': 1, 'pos': (x+w//2, y+h//2)})], set()),
        ([], set()),
        ([(p.MOUSEBUTTONUP, {'button': 1})], set()),
        ([(p.QUIT, {})], set()),
    ])
    if forward and turn:
        expected = (.15/math.sqrt(2)*forward, .15/math.sqrt(2)*turn, 0.)
    else:
        expected = (.15*forward, 0., .4*turn)
    assert commands == [expected, expected, (0., 0., 0.), (0., 0., 0.)]


@pytest.mark.parametrize('key_name', ['K_UP', 'K_DOWN', 'K_LEFT', 'K_RIGHT', 'K_q', 'K_e'])
def test_all_keyboard_directions_require_fresh_press_after_focus_loss(joystick, monkeypatch, key_name):
    import pygame as p
    key = getattr(p, key_name)
    commands = run_frames(joystick, monkeypatch, [
        ([(p.KEYDOWN, {'key': key})], {key}),
        ([(p.WINDOWFOCUSLOST, {})], {key}),
        ([(p.WINDOWFOCUSGAINED, {})], {key}),
        ([(p.KEYDOWN, {'key': key, 'repeat': True})], {key}),
        ([(p.KEYUP, {'key': key})], set()),
        ([(p.KEYDOWN, {'key': key})], {key}),
        ([(p.QUIT, {})], set()),
    ])
    assert commands[0] != (0., 0., 0.)
    assert commands[1:5] == [(0., 0., 0.)]*4
    assert commands[5] == commands[0]


def test_demo_never_initializes_robot_outputs_even_with_transport_flags(joystick,monkeypatch):
    import pygame as p
    import socket
    def forbidden(*args,**kwargs):raise AssertionError('demo attempted robot I/O')
    monkeypatch.setattr(joystick,'RosSink',forbidden)
    monkeypatch.setattr(socket,'socket',forbidden)
    frames=iter([
        [p.event.Event(p.KEYDOWN,key=p.K_UP)],
        [p.event.Event(p.KEYDOWN,key=p.K_LEFT)],
        [p.event.Event(p.KEYUP,key=p.K_UP),p.event.Event(p.KEYUP,key=p.K_LEFT)],
        [p.event.Event(p.QUIT)],
    ])
    monkeypatch.setattr(p.event,'get',lambda:next(frames))
    monkeypatch.setattr(joystick.time,'sleep',lambda value:None)
    joystick.run(SimpleNamespace(demo=True,ros=True,spi=True,udp='invalid-host',monitor=True,
                                 max_lin=.15,max_ang=.8,rate=50))


@pytest.mark.parametrize('forward,side',[(1,1),(1,-1),(-1,1),(-1,-1)])
@pytest.mark.parametrize('multiplier',[1,2,3])
def test_crab_steering_is_45_degrees_without_yaw_at_each_speed(joystick,forward,side,multiplier):
    vx,vy,omega=joystick.drive_direction(forward,side,.15*multiplier,.8)
    assert omega==0.
    assert vx*forward>0 and vy*side>0
    assert abs(vx)==pytest.approx(abs(vy))
    assert math.hypot(vx,vy)==pytest.approx(.15*multiplier)
    assert math.degrees(math.atan(abs(vy/vx)))==pytest.approx(45.)

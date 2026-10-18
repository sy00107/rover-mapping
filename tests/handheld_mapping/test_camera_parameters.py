# ================================================================
# test_camera_parameters.py
# 설명: 선택 depth_units가 없는 실제 드라이버에서도 projector 검증을 유지하는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_camera_parameters.py -q
# ================================================================
from types import SimpleNamespace

import pytest


def test_optional_depth_units_missing_does_not_break_required_emitter_query():
    from experiments.handheld_mapping.pi_worker import depth_parameters
    calls = []
    def fetch(names: list[str]) -> list:
        calls.append(names)
        if names == ['depth_module.emitter_enabled']:
            return [SimpleNamespace(type=2, integer_value=0)]
        return []
    values = depth_parameters(fetch)
    assert values == {'emitter_enabled': 0, 'sensor_depth_units_m': None}
    assert calls == [['depth_module.emitter_enabled'], ['depth_module.depth_units']]


@pytest.mark.parametrize('value', [[], [SimpleNamespace(type=0, integer_value=0)],
                                  [SimpleNamespace(type=3, integer_value=0)],
                                  [SimpleNamespace(type=2, integer_value=1)]])
def test_unknown_or_enabled_projector_fails_closed(value: list):
    from experiments.handheld_mapping.pi_worker import depth_parameters
    with pytest.raises(RuntimeError, match='projector OFF'):
        depth_parameters(lambda names: value)


def test_available_sensor_depth_units_are_metadata():
    from experiments.handheld_mapping.pi_worker import depth_parameters
    def fetch(names: list[str]) -> list:
        return ([SimpleNamespace(type=2, integer_value=0)] if 'emitter' in names[0]
                else [SimpleNamespace(type=3, double_value=.001)])
    assert depth_parameters(fetch)['sensor_depth_units_m'] == .001


def test_startup_rate_uses_recent_window_without_hiding_sustained_slow_camera():
    from experiments.handheld_mapping.pi_worker import recent_camera_rates
    # An initial USB restart gap expires only after a whole healthy window.
    values=[0.,.066,3.]+[4.+i/15 for i in range(15)]
    assert recent_camera_rates({'left':values})['left']==pytest.approx(15.)
    assert recent_camera_rates({'left':[i/5 for i in range(20)]})['left']==pytest.approx(5.)
    assert recent_camera_rates({'left':[0.,.1]})['left']==0.

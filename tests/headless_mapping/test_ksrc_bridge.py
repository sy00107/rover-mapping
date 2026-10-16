# ================================================================
# test_ksrc_bridge.py
# 설명: 실제 SPI를 열지 않고 baseline과 patch의 명령·stale·종료 0 명령을 비교한다.
# 사용법: KSRC_REVIEW_ROOT=/path/to/ksrc /usr/bin/python3 -m pytest tests/headless_mapping/test_ksrc_bridge.py
# ================================================================
import importlib.util
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import types
import pytest

ROOT=Path(__file__).resolve().parents[2]
BASE=Path(os.environ.get('KSRC_REVIEW_ROOT','/tmp/rover-ksrc-review-fIZ5v1/ksrc-4bd158b1125cee32c5851f74ecc0c8f3d71013b1'))

@pytest.mark.parametrize('consumer',['absent','slow'])
@pytest.mark.parametrize('linear_limit', [.32, .45])
def test_bridge_command_equivalence(monkeypatch,tmp_path,consumer,linear_limit):
    if not (BASE/'pi/common/udp_spi_bridge.py').is_file(): pytest.skip('KSRC fixed review source required')
    target=tmp_path/'patched'; (target/'pi/common').mkdir(parents=True)
    shutil.copyfile(BASE/'pi/common/udp_spi_bridge.py',target/'pi/common/udp_spi_bridge.py')
    patch=ROOT/'ros2_ws/src/orbslam3_ros2/patches/ksrc_telemetry_tee.patch'
    subprocess.run(['patch','--batch','--fuzz=0','-p1','-d',str(target),'-i',str(patch)],check=True,capture_output=True)
    path=tmp_path/'consumer.sock'; receiver=None
    if consumer=='slow':
        receiver=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM); receiver.bind(str(path))
    results=[]
    for source,enabled in [(BASE/'pi/common/udp_spi_bridge.py',False),(target/'pi/common/udp_spi_bridge.py',True)]:
        reserved=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); reserved.bind(('127.0.0.1',0)); port=reserved.getsockname()[1]; reserved.close()
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); calls=[]
        fake=types.ModuleType('nucleo_link'); fake.TLM_FLAG_GATE_PENDING=1; fake.TLM_FLAG_WATCHDOG=2
        class FakeSpi:
            """명령만 보존하며 실제 SPI 장치를 열지 않는다."""
            def __init__(self,*args): self.iterations=0; self.stopping=False
            def exchange(self,*cmd,**kw):
                calls.append((cmd,kw))
                if self.iterations==0: sender.sendto(module.PKT.pack(b'KSRC',1,1.,-.5,3.),('127.0.0.1',port))
                self.iterations+=1
                if self.iterations==30 and not self.stopping:
                    self.stopping=True; raise KeyboardInterrupt
                if self.iterations%3==0: return None
                return {'cmd':cmd,'steer_cmd_rad':[0]*4,'duty':[0]*4,'flags':16,'tick':self.iterations%256,
                        't_ms':self.iterations*20,'gyro_dps':[0,0,0],'acc_g':[0,0,1]}
            def close(self): pass
        fake.SpiLink=FakeSpi; monkeypatch.setitem(sys.modules,'nucleo_link',fake)
        spec=importlib.util.spec_from_file_location('mapping_bridge_fixture',source)
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        class FakeTime:
            """50 Hz loop의 시간만 진행시켜 watchdog을 결정적으로 시험한다."""
            value=0.
            def monotonic(self): return self.value
            def monotonic_ns(self): return int(self.value*1e9)
            def time_ns(self): return 10**18+int(self.value*1e9)
            def sleep(self,seconds): self.value+=seconds
        module.time=FakeTime()
        argv=['bridge','--port',str(port),'--max-lin',str(linear_limit)]
        if enabled: argv+=['--telemetry-socket',str(path),'--run-id','fixture']
        monkeypatch.setattr(sys,'argv',argv); module.main(); sender.close(); results.append(calls)
    if receiver: receiver.close()
    assert results[0]==results[1]
    commands=[cmd for cmd,kw in results[1]]
    assert commands[1] == (linear_limit,-linear_limit,2.)
    assert commands[20] == (0.,0.,0.)
    assert commands[-5:] == [(0.,0.,0.)]*5

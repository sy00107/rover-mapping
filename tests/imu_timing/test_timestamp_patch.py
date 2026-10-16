# ================================================================
# test_timestamp_patch.py
# 설명: portable C 시각 누적기와 실제 KSRC 패치의 wire 호환성을 검사한다.
# 사용법: KSRC_IMU_TIMING_BASELINE=PATH .venv-mapping/bin/python -m pytest tests/imu_timing -q
# ================================================================
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[2]


def build_timing_probe(folder):
    binary=folder/'timing_probe'
    result=subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=undefined',
        '-I',str(ROOT/'experiments/imu_timing_patch'),str(ROOT/'tests/imu_timing/test_mean_time.c'),'-o',str(binary)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return binary


@pytest.mark.parametrize('case',['regular','irregular','wrap','range','empty_reset','overflow'])
def test_mean_read_time_arithmetic(tmp_path,case):
    timing_probe=build_timing_probe(tmp_path)
    subprocess.run([str(timing_probe),case],check=True)


@pytest.fixture(scope='module')
def patched_ksrc(tmp_path_factory):
    source=os.environ.get('KSRC_IMU_TIMING_BASELINE')
    if not source: pytest.skip('KSRC_IMU_TIMING_BASELINE is required for integration')
    target=tmp_path_factory.mktemp('ksrc_patch')/'ksrc'
    shutil.copytree(source,target)
    patch=ROOT/'experiments/imu_timing_patch/ksrc_imu_mean_time.patch'
    with patch.open() as f:
        result=subprocess.run(['patch','--batch','--fuzz=0','-p1'],stdin=f,cwd=target,text=True,capture_output=True)
    assert result.returncode==0,result.stdout+result.stderr
    return target


def decoder(path):
    spec=importlib.util.spec_from_file_location('nucleo_timing_fixture',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def telemetry_probe(patched_ksrc):
    source=patched_ksrc/'probe.c'
    source.write_text('''/* SPI wire loopback probe: pytest fixture로 빌드·실행한다. */
#include "spi_link.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc,char **argv) {
 if(argc!=3) return 2;
 spi_link_telemetry_t t={0}; unsigned char frame[SPI_LINK_FRAME_LEN];
 t.flags=(uint16_t)strtoul(argv[1],0,10);t.t_ms=100;t.imu_n=4;
 t.imu_mean_read_offset_us=(int16_t)strtol(argv[2],0,10);
 spi_link_pack_telemetry(&t,frame);
 spi_link_telemetry_t parsed={0};
 if(spi_link_parse_telemetry(frame,&parsed)!=SPI_LINK_OK || parsed.imu_mean_read_offset_us!=t.imu_mean_read_offset_us) return 3;
 return fwrite(frame,1,SPI_LINK_FRAME_LEN,stdout)==144?0:4;
}''')
    binary=patched_ksrc/'probe'
    subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-I',str(patched_ksrc/'firmware/common'),
        str(source),str(patched_ksrc/'firmware/common/spi_link.c'),'-o',str(binary)],check=True)
    return binary


@pytest.mark.parametrize('flags,offset,valid',[(4112,-7000,True),(4112,1200,True),(16,-7000,False),(4096,-7000,False)])
def test_new_c_frame_decodes_with_both_new_and_legacy_python(patched_ksrc,telemetry_probe,flags,offset,valid):
    frame=subprocess.check_output([str(telemetry_probe),str(flags),str(offset)])
    assert len(frame)==144
    current=decoder(patched_ksrc/'pi/common/nucleo_link.py')
    old=decoder(Path(os.environ['KSRC_IMU_TIMING_BASELINE'])/'pi/common/nucleo_link.py')
    legacy=old.decode_spi_telemetry(frame);new=current.decode_spi_telemetry(frame)
    assert legacy is not None and new is not None
    assert all(new[k]==v for k,v in legacy.items())
    assert new['imu_timing_valid'] is valid
    assert new['imu_mean_read_offset_us']==(offset if valid else None)
    corrupted=bytearray(frame);corrupted[140]^=1
    assert current.decode_spi_telemetry(bytes(corrupted)) is None
    assert current.encode_spi_cmd(.1,-.2,.3,7)==old.encode_spi_cmd(.1,-.2,.3,7)


def test_actual_driver_mean_and_fallback_keep_adc_values_and_read_time(patched_ksrc):
    source=(patched_ksrc/'firmware/src2026-rover-firmware/Core/Src/imu_ism330.c').read_text()
    functions=[]
    for name in ('Imu_Poll','Imu_TakeMean','Imu_MeanReadOffsetUs'):
        begin=source.index('int '+name+'(')
        brace=source.index('{',begin)
        depth=1;end=brace+1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        functions.append(source[begin:end])
    harness=patched_ksrc/'driver_probe.c'
    harness.write_text('''/* 실제 driver 세 함수에 외부 HAL/SPI 읽기만 대역으로 제공한다. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "imu_mean_time.h"
#define REG_STATUS 0
static int32_t s_sum[6]; static uint8_t s_n, s_valid=1;
static imu_time_accumulator_t s_read_times;
static imu_mean_time_t s_last_mean_time;
static uint32_t fake_ms; static int16_t fake_value; static int fake_ok=1;
static uint32_t HAL_GetTick(void) { return fake_ms; }
static int reg_read(int address,uint8_t *out,int size) {
 (void)address;(void)size;*out=3;return 1;
}
static int Imu_Read(int16_t out[6]) {
 for(int i=0;i<6;i++) out[i]=fake_value;
 return fake_ok;
}
'''+ '\n'.join(functions)+'''
int main(void) {
 fake_ms=100;fake_value=6;assert(Imu_Poll(0));
 fake_ms=105;fake_value=12;assert(Imu_Poll(0));
 fake_ms=119;fake_value=24;assert(Imu_Poll(0));
 int16_t out[6],offset;uint8_t n;
 assert(Imu_TakeMean(out,&n) && n==3);
 for(int i=0;i<6;i++) assert(out[i]==14);
 assert(Imu_MeanReadOffsetUs(120,&offset) && offset==-12000);
 fake_ms=125;fake_value=30;
 assert(Imu_TakeMean(out,&n) && n==0 && out[0]==30);
 assert(Imu_MeanReadOffsetUs(124,&offset) && offset==1000);
 fake_ok=0;
 assert(!Imu_TakeMean(out,&n));
 assert(!Imu_MeanReadOffsetUs(124,&offset) && offset==0);
 return 0;
}
''')
    binary=patched_ksrc/'driver_probe'
    subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=undefined',
        '-I',str(patched_ksrc/'firmware/src2026-rover-firmware/Core/Inc'),str(harness),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

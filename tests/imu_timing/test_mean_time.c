/* ================================================================
 * test_mean_time.c
 * 설명: 평균 표본 시각·wrap·무효 범위를 알려진 정수 예제로 검증한다.
 * 사용법: cc -std=c99 -I experiments/imu_timing_patch test_mean_time.c -o probe
 * ================================================================ */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "imu_mean_time.h"

int main(int argc, char **argv)
{
    assert(argc == 2);
    imu_time_accumulator_t a = {0};
    int16_t offset = 123;
    if (!strcmp(argv[1], "regular")) {
        imu_time_add(&a,100); imu_time_add(&a,105); imu_time_add(&a,110);
        imu_mean_time_t mean=imu_time_take(&a);
        assert(mean.count==3 && mean.first_ms==100 && mean.last_ms==110);
        assert(imu_mean_offset_us(&mean,112,&offset) && offset==-7000);
        assert(!imu_time_take(&a).valid);
    } else if (!strcmp(argv[1], "irregular")) {
        imu_time_add(&a,100); imu_time_add(&a,101); imu_time_add(&a,119);
        imu_mean_time_t mean=imu_time_take(&a);
        assert(imu_mean_offset_us(&mean,120,&offset) && offset==-13333);
    } else if (!strcmp(argv[1], "wrap")) {
        imu_time_add(&a,UINT32_MAX-1); imu_time_add(&a,1); imu_time_add(&a,4);
        imu_mean_time_t mean=imu_time_take(&a);
        assert(imu_mean_offset_us(&mean,6,&offset) && offset==-5000);
    } else if (!strcmp(argv[1], "range")) {
        imu_time_add(&a,100); imu_mean_time_t mean=imu_time_take(&a);
        assert(!imu_mean_offset_us(&mean,133,&offset) && offset==0);
        assert(imu_mean_offset_us(&mean,68,&offset) && offset==32000);
    } else if (!strcmp(argv[1], "empty_reset")) {
        imu_mean_time_t mean=imu_time_take(&a);
        assert(!imu_mean_offset_us(&mean,100,&offset) && offset==0);
        imu_time_add(&a,100); imu_time_add(&a,90); mean=imu_time_take(&a);
        assert(!mean.valid && !imu_mean_offset_us(&mean,100,&offset));
    } else if (!strcmp(argv[1], "overflow")) {
        for (unsigned i=0;i<256;i++) imu_time_add(&a,100+i);
        assert(!imu_time_take(&a).valid);
    } else assert(0);
    return 0;
}

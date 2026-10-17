# ORB 위치 + RTAB RGB-D 지도 시각화

촬영을 다시 하지 않고 RUN `20261016_174639_b1d8ed`의 최종 ORB offline camera keyframe712개를 RTAB-Map0.23.13에 외부 pose로 입력했다. 각 keyframe에 동일 시각의 원본 left IR과 측정 depth를 함께 넣었다. 현재 구성은 offline role composition이며 joint estimator나 두 궤적의 평균이 아니다.

- ORB: 최종 camera-to-world pose. ORB의 기존 최적화/loop closure 결과를 포함한다.
- RTAB: 외부 pose 기반 RGB-D node/neighbor graph와 IR/depth database 저장. RTAB odometry는 실행하지 않는다.
- 시각화 mesh: 저장된 RTAB graph pose와 원본 depth를 기존 Open3D TSDF / GroundGrid / 표면 보정 파이프라인으로 처리한다. 이 실험에서는 RTAB 자체 cloud exporter로 TSDF를 생성하지 않는다.

RTAB 추가 보정은 끈다: `Rtabmap/LoopThr=1`, `RGBD/ProximityBySpace=false`, `RGBD/ProximityByTime=false`, `RGBD/NeighborLinkRefining=false`, `Optimizer/Iterations=0`. 키프레임을 보존하기 위해 `RGBD/LinearUpdate=0`, `RGBD/AngularUpdate=0`, `Mem/RehearsalSimilarity=1`을 사용한다. ORB export에는 covariance가 없어서 nominal1e-4 diagonal을 넣으며 실측 불확실성으로 해석하지 않는다. 추가 graph 보정이 꺼져 있으므로 그 가중치로 궤적을 수정하지 않는다.

실제 DB/graph 검증: input712 / retained712, neighbor link1422(양방향), 다른 type의 constraint0. Imported ORB 대비 RTAB graph pose의 최대 위치차8.17e-7m / 방향차0.000102°. `fusion_verification.json`에 기록하고, 허용범위를 넘는 pose 변경이나 non-neighbor link가 있으면 생성 단계에서 실패시킨다.

결과 폴더: `outputs/handheld_mapping/comparisons/orb_rtab_fusion_20261016_174639`

원본 depth 0.25–2m, 최종 camera pose, TSDF1cm, 원거리 depth 보정·다중시점 확인, calibrated stationary IMU/archived URDF를 이용한 provisional Z-up alignment. 동일 filtered depth를 GroundGrid에 누적하고 초록=지면, 주황=장애물, 회색=미분류로 표시한다. 이후 관측된 초록 지면의 짧은 틈 재연결과 지면 음영 안정화를 적용한다. 실제 경사·단차와 큰 미관측 영역을 강제로 평면화하지 않는다. 표면 연결 일부는 추론이며 새 관측이나 통행 가능 판정이 아니다.

보기:

```bash
cd ~/rover-mapping
bash tools/view_orb_rtabmap.sh
# 보정 전 mesh
bash tools/view_orb_rtabmap.sh --raw
# 색상 점군
bash tools/view_orb_rtabmap.sh --cloud
```

새 output으로 전체 재생성:

```bash
bash tools/build_orb_rtabmap.sh \
  outputs/handheld_mapping/sessions/20261016_174639_b1d8ed \
  outputs/handheld_mapping/comparisons/orb_rtab_fusion_new
```

기존 ORB, RTAB 단독, raw capture 결과는 수정하지 않는다. 현 구성은 ORB 위치를 보존하므로 ORB 단독보다 정확도가 개선되었다는 주장은 할 수 없다. RTAB 추가 loop 보정이나 실시간 ORB pose jump 처리, 공동 uncertainty 추정은 이번 시연에 포함하지 않는다.

검증: native Release build, 실제712-node DB/pose/constraint 검사, 외부pose 변경·추가loop 거부/입력timestamp·출력보존/지면재연결을 포함한 관련11개 테스트 통과. `git diff --check` 통과.

최종 시각화 결과: filtered depth712개 모두 융합. 지면233,872 / 장애물111,257 / 미분류49,180 vertices. 초록 지면 보정184,453 vertices, 최대변위4.75mm, 비지면 좌표와 색 불변. 지면 재연결 후 nonmanifold edge0. native screenshot을 확인했고 창 제목을 `ORB pose + RTAB RGB-D`로 구분했다. screenshot은 `surface_refined_v4/native_view.png`다.

이동 경로 표시: `view_orb_rtabmap.sh`는 기본 `--trajectory`로 저장된 최종712 camera pose를 파란 선으로 연결하고 초록 시작구·빨강 종료구를 표시한다. source pose checksum을 확인하고 mesh summary의 `gravity.world_from_slam`을 똑같이 적용한다. 메시에 경로를 굽거나 지면으로 투영하지 않으며 실제 로봇 중심/encoder 경로가 아닌 camera 추정 경로다. mesh/분류/원본은 바뀌지 않는다. 좌표변환 및 source 변경 거부를 포함한 관련5개 테스트 통과, native screenshot 확인.

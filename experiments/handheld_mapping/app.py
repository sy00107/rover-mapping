# ================================================================
# app.py
# 설명: 노트북에서 시작·종료 버튼으로 D435 기록과 TSDF 지도 생성을 제어한다.
# 사용법: bash tools/handheld_mapping.sh [--demo]
# ================================================================
import argparse
import json
import os
from pathlib import Path
import sys
import signal
import subprocess

from PyQt5.QtCore import Qt, QTimer, QUrl, QProcess
from PyQt5.QtGui import QDesktopServices, QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog, QGridLayout,
                            QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy, QVBoxLayout, QWidget)

from experiments.handheld_mapping.controller import ACTIVE, ROOT, Session
from experiments.handheld_mapping.live_preview import CoverageView


class MappingWindow(QWidget):
    """촬영 상태를 표시하고 blocking 작업을 session thread에 맡기는 노트북 제어창."""

    def __init__(self, session: Session, output_root: Path | None = None, demo: bool = False,
                 imu_calibration: Path | None = None, inertial_fusion: bool = False) -> None:
        super().__init__()
        self.session = session
        self.close_pending = False
        self.opened_transfer_attempts = set()
        self.transfer_terminal_process = None
        self.setWindowTitle("D435 · Handheld Mapping")
        self.resize(1200, 900)
        self.setStyleSheet("""
            QWidget { background:#111820; color:#e8edf2; font-size:14px; }
            QLineEdit,QComboBox { background:#1b2733; border:1px solid #344556; border-radius:6px; padding:9px; }
            QPushButton { background:#243445; border:1px solid #41566a; border-radius:7px; padding:8px 18px; }
            QPushButton:hover { background:#334d64; }
            QPushButton:disabled { color:#647382; background:#19232d; border-color:#253340; }
            QPushButton#start { background:#176951; border-color:#259b79; font-weight:bold; }
            QPushButton#stop { background:#713b33; border-color:#a46458; font-weight:bold; }
            QLabel#heading { font-size:26px; font-weight:bold; }
            QLabel#status { color:#80d5bb; font-size:20px; padding:10px 0; }
            QLabel#preview { background:#090e14; border:1px solid #2c3a48; border-radius:8px; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(26, 12, 26, 12)
        layout.setSpacing(4)
        heading = QLabel("D435  /  HANDHELD MAPPING")
        heading.setObjectName("heading")
        layout.addWidget(heading)
        layout.addWidget(QLabel("Pi 원본 저장 · 실시간 미리보기 · 종료 후 원본 전송과 지도 생성"))
        fields = QGridLayout()
        fields.setVerticalSpacing(2)
        self.host = QLineEdit("pi@raspberrypi.local")
        self.output = QLineEdit(str(output_root or ROOT / "outputs/handheld_mapping/sessions"))
        # Capture policy is fixed; synthetic/fusion experiments remain CLI-only.
        self.demo = demo
        self.inertial_fusion = inertial_fusion and not demo
        self.point_ready = (ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare').is_file()
        calibration = imu_calibration or ROOT / "outputs/handheld_mapping/calibration/imu_calibration.json"
        self.imu_calibration = QLineEdit(str(calibration))
        self.imu_zero_button = QPushButton('평지 영점')
        self.imu_zero_button.setToolTip('알려진 평지에서 3초 정지한 뒤 누르세요. IMU 참고 각도의 표시 영점만 설정하며 지도와 센서 보정값은 바꾸지 않습니다.')
        self.imu_zero_button.clicked.connect(self.zero_imu_display)
        fields.addWidget(QLabel("Pi 연결"), 0, 0)
        fields.addWidget(self.host, 0, 1)
        fields.addWidget(self.imu_zero_button, 0, 2)
        fields.setColumnStretch(1, 1)
        layout.addLayout(fields)
        policy = QLabel("영상 + IMU 자동 기록  ·  종료 후 PC에서 지도 생성  ·  Pi는 검증 후 최신 촬영 보관")
        policy.setStyleSheet("color:#80d5bb; padding:6px 0;")
        layout.addWidget(policy)
        layout.addWidget(QLabel("촬영 시작 후 평지에서 3초간 멈춰 주세요."))
        self.settings_button = QPushButton("세부 설정 ▸")
        self.settings_button.setCheckable(True)
        layout.addWidget(self.settings_button, alignment=Qt.AlignLeft)
        self.settings_panel = QWidget()
        settings = QVBoxLayout(self.settings_panel)
        settings.setContentsMargins(0, 4, 0, 8)
        paths = QGridLayout()
        paths.addWidget(QLabel("저장 폴더"), 0, 0)
        paths.addWidget(self.output, 0, 1)
        choose = QPushButton("선택")
        choose.clicked.connect(self.choose_output)
        self.choose_button = choose
        paths.addWidget(choose, 0, 2)
        paths.addWidget(QLabel("IMU 보정 파일"), 1, 0)
        paths.addWidget(self.imu_calibration, 1, 1, 1, 2)
        settings.addLayout(paths)
        self.settings_panel.hide()
        self.settings_button.toggled.connect(self.toggle_settings)
        layout.addWidget(self.settings_panel)
        self.status_label = QLabel("준비 · 시작 버튼을 누르세요")
        self.status_label.setObjectName("status")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.metrics = QLabel("촬영 시간 00:00   |   저장 프레임 0   |   추적 대기")
        layout.addWidget(self.metrics)
        self.imu_status = QLabel("IMU · 촬영 시작 대기")
        self.imu_status.setWordWrap(True)
        layout.addWidget(self.imu_status)
        self.preview = QLabel("카메라 미리보기\n촬영을 시작하면 좌측 IR 영상이 표시됩니다")
        self.preview.setObjectName("preview")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumHeight(100)
        self.preview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.overlay_preview = QLabel('촬영하면 관측 횟수 색이 영상 위에 표시됩니다')
        self.overlay_preview.setObjectName('preview')
        self.overlay_preview.setAlignment(Qt.AlignCenter)
        self.overlay_preview.setMinimumHeight(100)
        self.overlay_preview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.overlay_note = QLabel('관측 횟수 오버레이 · 노랑 적음 / 초록 여러 시점')
        self.overlay_note.setWordWrap(True)
        cameras = QVBoxLayout()
        cameras.setSpacing(4)
        cameras.addWidget(QLabel('원본 카메라 영상'))
        cameras.addWidget(self.preview,1)
        cameras.addWidget(self.overlay_note)
        cameras.addWidget(self.overlay_preview,1)
        views = QHBoxLayout()
        views.addLayout(cameras,1)
        self.coverage = CoverageView()
        views.addWidget(self.coverage,1)
        layout.addLayout(views,1)
        self.path_label = QLabel("매번 새 날짜별 폴더를 만들며 기존 기록을 덮어쓰지 않습니다")
        self.path_label.setWordWrap(True)
        layout.addWidget(self.path_label)
        self.retention_status = QLabel()
        self.retention_status.setWordWrap(True)
        self.retention_status.hide()
        layout.addWidget(self.retention_status)
        viewing = QHBoxLayout()
        viewing.addWidget(QLabel('3D 보기 설정'))
        self.view_backend = QComboBox()
        self.view_backend.addItem('기본 카메라 지도 (우선)', 'auto')
        self.view_backend.addItem('IMU 융합 시험 지도', 'orb_inertial')
        self.view_backend.addItem('점+선 비교 지도', 'orbline')
        self.view_backend.setToolTip('기본은 전체 카메라 지도입니다. IMU 시험 지도는 일부 구간만 있을 수 있습니다.')
        viewing.addWidget(self.view_backend)
        self.view_depth = QComboBox()
        for label, value in [('2m', 2.), ('1m', 1.), ('3m', 3.), ('저장된 기본 지도', None)]:
            self.view_depth.addItem(label, value)
        self.view_depth.setToolTip('원본에서 거리별 비교본을 생성합니다. 첫 생성은 시간이 걸리며 이후에는 재사용합니다.')
        viewing.addWidget(self.view_depth)
        self.view_cleanup = QCheckBox('표면 보정')
        self.view_cleanup.setChecked(True)
        self.view_cleanup.setToolTip('선택한 거리의 깊이와 표면을 보정합니다. 원본과 기존 기본 지도는 보존합니다.')
        viewing.addWidget(self.view_cleanup)
        self.view_measurements = QCheckBox('거리·각도 표시')
        self.view_measurements.setChecked(False)
        self.view_measurements.setToolTip('체크하면 경로 옆에 구간 거리와 좌우 회전각을 표시합니다. 하늘색 경로는 항상 유지합니다.')
        viewing.addWidget(self.view_measurements)
        viewing.addStretch()
        self.view_depth.currentIndexChanged.connect(
            lambda _: self.view_cleanup.setEnabled(self.view_depth.currentData() is not None))
        settings.addLayout(viewing)
        controls = QHBoxLayout()
        self.start_button, self.stop_button = QPushButton("매핑 시작"), QPushButton("종료 및 지도 생성")
        self.start_button.setObjectName("start")
        self.stop_button.setObjectName("stop")
        self.open_button, self.rebuild_button = QPushButton("저장 폴더 열기"), QPushButton("기록에서 다시 생성")
        self.view_button = QPushButton('3D 지도 보기')
        self.export_process = None
        self.joined_export_button = QPushButton('평면 보정 STL 내보내기')
        self.joined_export_button.setToolTip('중력 유지 · 평지/경사를 평면으로 보정 · 장애물 표면 보존 · STL(m/mm)')
        self.joined_export_button.clicked.connect(self.export_joined_stl)
        self.terrain_button = QPushButton('높이·경사 보기')
        for button in (self.start_button, self.stop_button, self.view_button, self.terrain_button, self.open_button, self.rebuild_button):
            controls.addWidget(button)
        layout.addLayout(controls)
        recovery_controls = QHBoxLayout()
        self.pause_button = QPushButton('전송 보류 · 나중에 이어받기')
        self.resume_button = QPushButton('중단된 원본 이어받기')
        self.transfer_terminal_button = QPushButton('전송 창 열기')
        self.transfer_terminal_button.clicked.connect(self.open_transfer_terminal)
        self.pause_button.clicked.connect(session.pause_transfer)
        self.resume_button.clicked.connect(self.resume_transfer)
        recovery_controls.addWidget(self.pause_button)
        recovery_controls.addWidget(self.resume_button)
        recovery_controls.addWidget(self.transfer_terminal_button)
        recovery_controls.addWidget(self.joined_export_button)
        layout.addLayout(recovery_controls)
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(session.stop)
        self.open_button.clicked.connect(self.open_folder)
        self.rebuild_button.clicked.connect(self.rebuild)
        self.view_button.clicked.connect(self.view_map)
        self.terrain_button.clicked.connect(self.view_terrain)
        settings.addWidget(QLabel("15 FPS · 1cm 지도 격자 · 원본과 생성 지도는 별도 보관"))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.refresh()

    def toggle_settings(self, expanded: bool) -> None:
        self.settings_panel.setVisible(expanded)
        self.settings_button.setText("세부 설정 ▾" if expanded else "세부 설정 ▸")

    def choose_output(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "노트북 저장 폴더", self.output.text())
        if selected:
            self.output.setText(selected)

    def start(self) -> None:
        try:
            self.session.start({"host": self.host.text().strip(), "output_root": self.output.text(),
                                "demo": self.demo, "compare_lines": False,
                                "sensors_only": True,
                                "live_mapping": self.point_ready,
                                "imu_enabled": not self.demo,
                                "gravity_enabled": not self.demo,
                                "inertial_fusion": self.inertial_fusion,
                                "imu_calibration": self.imu_calibration.text().strip()})
        except Exception as error:
            self.status_label.setText(str(error))
        self.refresh()

    def open_folder(self) -> None:
        path = self.session.snapshot()["run_dir"]
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def open_transfer_terminal(self) -> None:
        from experiments.handheld_mapping.transfer_monitor import launch_terminal
        run_dir = self.session.snapshot()['run_dir']
        if not run_dir:
            return
        try:
            self.transfer_terminal_process = launch_terminal(Path(run_dir))
            self.session.update(transfer_terminal_warning=None)
        except OSError as error:
            self.session.update(transfer_terminal_warning=str(error))

    def resume_transfer(self) -> None:
        current = self.session.snapshot()
        selected = current['run_dir'] if current['state'] == 'paused' else QFileDialog.getExistingDirectory(
            self, '이어받을 기존 촬영 폴더', self.output.text())
        if selected:
            try:
                self.session.resume_transfer(Path(selected))
            except Exception as error:
                self.status_label.setText(str(error))

    def zero_imu_display(self) -> None:
        try:
            self.session.zero_imu_display()
        except (ValueError,OSError) as error:
            self.session.update(message=str(error))
        self.refresh()

    def rebuild(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "원본 기록 폴더 선택", self.output.text())
        if selected:
            self.session.rebuild(Path(selected))
            self.refresh()

    def view_map(self) -> None:
        from experiments.handheld_mapping.viewer import FOLDERS, select_maps
        selected = self.session.snapshot()['run_dir'] or QFileDialog.getExistingDirectory(
            self,'3D 지도가 저장된 촬영 폴더 선택',self.output.text())
        if not selected:
            return
        try:
            run = Path(selected)
            backend = self.view_backend.currentData()
            selected_maps = select_maps(run, backend=backend)
            # Pin the displayed result's backend before any depth reconstruction.
            backend = next(key for key, folder in FOLDERS.items()
                           if selected_maps[0][1].parent.name == folder)
            command = [sys.executable, '-m', 'experiments.handheld_mapping.viewer', str(run),
                       '--backend', backend]
            if not self.view_measurements.isChecked():
                command.append('--no-path-measurements')
            depth = self.view_depth.currentData()
            derived = (run/'depth_variant.json').is_file() or (run/'merge.json').is_file()
            if depth is not None and not derived:
                command += ['--depth-max', f'{depth:g}']
                if self.view_cleanup.isChecked():
                    command.append('--surface-cleanup')
                message = f'{depth:g}m'+(' · 표면 보정' if self.view_cleanup.isChecked() else '')
                message += ' 지도 준비 중 · 첫 생성에는 시간이 걸립니다 · map_viewer.log 확인'
            else:
                message = ('저장된 통합·거리별 지도 열기 · 해당 결과의 기존 거리·보정 설정을 사용합니다'
                           if derived else '저장된 기본 지도 열기')
            message += ' · '+('거리·각도 표시' if self.view_measurements.isChecked() else '거리·각도 숨김')
            environment = dict(os.environ)
            if environment.get('DISPLAY'):
                environment.pop('WAYLAND_DISPLAY',None)
                environment['XDG_SESSION_TYPE'] = 'x11'
            with (run/'map_viewer.log').open('ab') as log:
                subprocess.Popen(command,
                                 cwd=ROOT,env=environment,stdout=log,stderr=log,start_new_session=True)
            self.session.update(message=message)
            self.refresh()
        except (ValueError,OSError) as error:
            self.session.update(message=str(error))
            self.refresh()

    def export_joined_stl(self) -> None:
        from datetime import datetime
        selected = self.session.snapshot()['run_dir'] or QFileDialog.getExistingDirectory(
            self, 'STL로 내보낼 촬영 폴더 선택', self.output.text())
        if not selected:
            return
        run = Path(selected)
        output = run/'exports'/('planar_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        command = ['-m', 'experiments.handheld_mapping.joined_export', str(run),
                   '--output', str(output), '--backend', self.view_backend.currentData()]
        depth = self.view_depth.currentData()
        if depth is not None and not ((run/'depth_variant.json').is_file() or (run/'merge.json').is_file()):
            command += ['--depth-max', str(depth)]
        process = QProcess(self)
        self.export_process = process
        process.setWorkingDirectory(str(ROOT))
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.setStandardOutputFile(str(run/'joined_export.log'))
        def finished(code, *args):
            self.export_process = None
            if code == 0:
                self.session.update(message=f'평면 보정 STL 저장 완료: {output}')
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(output)))
            else:
                self.session.update(message=f'STL 생성 실패 · {run/"joined_export.log"} 확인')
            process.deleteLater()
            self.refresh()
        process.finished.connect(finished)
        process.errorOccurred.connect(lambda error: finished(-1) if error == QProcess.FailedToStart else None)
        process.start(sys.executable, command)
        self.session.update(message='평면 보정 STL 생성 중 · 완료 후 결과 폴더가 열립니다')
        self.refresh()

    def view_terrain(self) -> None:
        selected = self.session.snapshot()['run_dir'] or QFileDialog.getExistingDirectory(
            self,'지형 지도가 저장된 촬영 폴더 선택',self.output.text())
        if not selected:
            return
        terrain = self.session.snapshot().get('terrain',{})
        paths = [Path(row['output_dir'])/'preview.png' for row in terrain.get('maps',{}).values()
                 if row.get('output_dir') and (Path(row['output_dir'])/'preview.png').is_file()]
        if not paths:
            paths = sorted((Path(selected)/'terrain').glob('*/map_*/preview.png'))
        if not paths:
            self.session.update(message='저장된 지형 지도가 없습니다. 중력 기준과 정지 IMU 구간을 확인하세요.')
            self.refresh()
            return
        for path in paths:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def refresh(self) -> None:
        status = self.session.snapshot()
        active = status["state"] in ACTIVE or (self.session.thread is not None and self.session.thread.is_alive())
        active = active or self.export_process is not None
        progress_path = Path(status['run_dir'])/'transfer_status.json' if status['run_dir'] else None
        self.transfer_terminal_button.setEnabled(bool(progress_path and progress_path.is_file()))
        if (status.get('local_recording') and progress_path and progress_path.is_file()):
            try:
                progress = json.loads(progress_path.read_text())
                attempt = (status['run_dir'], progress['attempt'])
                if (progress.get('phase') in {'transferring', 'importing', 'verifying', 'complete', 'paused', 'failed'}
                        and attempt not in self.opened_transfer_attempts):
                    self.opened_transfer_attempts.add(attempt)
                    self.open_transfer_terminal()
            except (OSError, ValueError, KeyError):
                pass
        if self.transfer_terminal_process is not None and self.transfer_terminal_process.poll() is not None:
            if self.transfer_terminal_process.returncode:
                self.session.update(transfer_terminal_warning='전송 터미널을 열지 못했습니다. 앱의 전송률 또는 transfer_terminal.log를 확인하세요.')
            self.transfer_terminal_process = None
        self.joined_export_button.setEnabled(not active)
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(status["state"] in {"starting", "recording"})
        self.pause_button.setEnabled(bool(status.get('local_recording')) and
            status['state'] in {'stopping','finalizing','transferring'})
        self.resume_button.setEnabled(not active)
        self.rebuild_button.setEnabled(not active)
        self.view_button.setEnabled(not active)
        self.terrain_button.setEnabled(not active)
        self.open_button.setEnabled(bool(status["run_dir"]))
        for widget in (self.host, self.output, self.choose_button):
            widget.setEnabled(not active)
        self.imu_calibration.setEnabled(not active and not self.demo)
        imu = status.get("imu")
        self.imu_zero_button.setEnabled(status['state']=='recording' and bool(imu) and not imu.get('stale',True))
        imu_text = "IMU · 촬영 시작 시 자동 수집" if not self.demo else "합성 입력 · IMU 없음"
        if imu:
            imu_text = (f"IMU 기록 {imu.get('state', '대기')} · 유효 {imu.get('valid', 0)}"
                        f" · 고유 {imu.get('unique', 0)} / {imu.get('hz', 0):.1f} Hz")
            if imu.get("roll_deg") is not None and imu.get("pitch_deg") is not None:
                roll = imu.get('display_roll_deg',imu['roll_deg'])
                pitch = imu.get('display_pitch_deg',imu['pitch_deg'])
                imu_text += f" · 가속도 참고 roll {roll:.1f}° / pitch {pitch:.1f}°"
                if imu.get('display_zero_set'):imu_text += ' (평지 영점 기준)'
            if imu.get("stale"):
                imu_text += " · 수신 지연"
            if imu.get('source_gap_events',0):
                imu_text += f" · 기록 중 표본 공백 {imu['source_gap_events']}회"
            if imu.get("error"):
                imu_text += " · 오류: "+str(imu["error"])
        gravity = status.get('gravity')
        if gravity:
            imu_text += f" · 중력 기준 {len(gravity.get('maps',{}))}개 지도 / {gravity['status']}"
        elif not self.demo:
            imu_text += ' · 중력 기준 준비: 촬영 시작 후 3초간 정지'
        fusion_text = 'ORB 융합 안 함'
        if status.get('inertial_warning'):
            fusion_text = status['inertial_warning']
        elif status.get('inertial_dense', {}).get('status') == 'complete':
            fusion_text = 'IMU 융합 지도 저장'
        elif self.inertial_fusion:
            fusion_text = '종료 후 IMU 융합 시험'
        self.imu_status.setText(imu_text)
        self.imu_status.setToolTip(fusion_text+' · 영상과 IMU의 시계 동기화는 미검증')
        live = self.session.live_snapshot()
        if live and live.get('body_attitude') and not live.get('stale') and live.get('tracking_state') == 2:
            body = live['body_attitude']
            self.imu_status.setText(self.imu_status.text()+f" · 중력 기준 차체 기울기 {body['tilt_deg']:.1f}°"
                                    + (' · 기울어진 구간' if body['body_tilted'] else ''))
        self.coverage.set_snapshot(live)
        label = status["message"]
        if self.close_pending and active:
            label = "저장이 끝나면 창을 닫습니다 · "+label
        if self.demo:
            label = "[합성 시험] "+label
        if status.get('transfer_terminal_warning'):
            label += ' · '+status['transfer_terminal_warning']
        self.status_label.setText(label)
        elapsed = int(status["elapsed_s"])
        counts = status.get("counts", {})
        recovery = '기존 지도 복구 중 · 정지해서 이전 장면 보기'
        tracking_labels = {2:'정상',3:recovery,4:'손실'}
        tracking = tracking_labels.get(status.get("tracking_state"), "대기")
        if status.get('capture_mode') == 'sensors_only':
            tracking = (tracking_labels.get(live.get('tracking_state'),'준비')
                        if live and not live.get('error') else '노트북 후처리')
        self.metrics.setText(f"시간 {elapsed//60:02d}:{elapsed%60:02d}  |  저장 IR {counts.get('left', 0)}/{counts.get('right', 0)}"
                             f" · depth {counts.get('depth', 0)}  |  수신 {status.get('left_receipt_fps', 0):.1f} FPS"
                             f"  |  추적 {tracking}  |  전송 누락 {status.get('dropped', 0)}")
        if status.get('local_recording'):
            pi = status.get('pi_recording', {})
            fps = pi.get('recording_fps', {}).get('left', 0.)
            self.metrics.setText(self.metrics.text()+f" | Pi 기록 {fps:.1f} FPS · 여유 {pi.get('disk_free_bytes', 0)/1e9:.1f} GB")
        retention = status.get('pi_retention')
        self.retention_status.setVisible(bool(retention))
        if retention:
            if retention['status'] == 'deferred':
                text = 'Pi 원본 정리 보류 · '+retention['reason']
                color = '#e4be7a'
            else:
                extra = {row['run_id'] for key in ('skipped', 'local_skipped', 'unverified_retained')
                         for row in retention.get(key, [])}
                text = f"Pi 과거 원본 {len(retention.get('deleted', []))}개 정리 · 최신 촬영 보관"
                if extra:
                    text += f' · 미검증·이상 촬영 {len(extra)}개 추가 보존'
                color = '#e4be7a' if extra else '#80d5bb'
            self.retention_status.setText(text)
            self.retention_status.setStyleSheet('color:'+color)
            self.retention_status.setToolTip(json.dumps(retention, ensure_ascii=False, indent=2))
        if status["run_dir"]:
            self.path_label.setText(status["run_dir"])
        with self.session.lock:
            image = self.session.preview
            if image is not None:
                image = image.copy()
        overlay = None
        note = '관측 횟수 오버레이 · 노랑 적음 / 초록 여러 시점'
        if live and not live.get('error') and (not live.get('stale') or live.get('finished')):
            if live.get('tracking_state') == 2 and live.get('overlay_original') is not None:
                image = live['overlay_original']
            if live.get('tracking_state') == 2 and not live.get('overlay_error'):
                overlay = live.get('overlay')
            if live.get('overlay_error'):
                note = '오버레이 표시 대기 · '+live['overlay_error']
            elif overlay is None:
                note = '오버레이 색 없음 · 추적 또는 depth 대기'
            elif live.get('finished'):
                note += ' · 촬영 종료 프레임'
        elif live:
            note = '오버레이 색 없음 · '+('실시간 추적 오류' if live.get('error') else '최신 위치 대기')
        if live and not live.get('error') and live.get('tracking_state') == 3:
            note = recovery+' · 이전 촬영 범위 유지'
        self.overlay_note.setText(note)
        if image is not None:
            h, w = image.shape
            qimage = QImage(image.data, w, h, image.strides[0], QImage.Format_Grayscale8).copy()
            self.preview.setPixmap(QPixmap.fromImage(qimage).scaled(self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            if overlay is None:
                overlay_qimage = qimage
            else:
                overlay_qimage = QImage(overlay.data,overlay.shape[1],overlay.shape[0],overlay.strides[0],QImage.Format_RGB888).copy()
            self.overlay_preview.setPixmap(QPixmap.fromImage(overlay_qimage).scaled(
                self.overlay_preview.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        else:
            self.preview.clear()
            self.preview.setText('카메라 수신 대기')
            self.overlay_preview.clear()
            self.overlay_preview.setText('동일 프레임의 관측 오버레이 대기')
        if self.close_pending and not active:
            self.close()

    def closeEvent(self, event) -> None:
        if self.export_process is not None:
            self.session.update(message="STL 생성 완료 후 창을 닫아주세요.")
            event.ignore()
            return
        active = self.session.snapshot()["state"] in ACTIVE or (self.session.thread is not None and self.session.thread.is_alive())
        if active:
            self.close_pending = True
            if self.session.snapshot().get('local_recording') and self.session.snapshot()['state'] in {'stopping','finalizing','transferring'}:
                self.session.pause_transfer()
            else:
                self.session.stop()
            event.ignore()
        else:
            event.accept()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--imu-calibration", type=Path)
    parser.add_argument('--imu-fusion', action='store_true', help='종료 후 별도 IMU 시간 정렬·융합 시험 (IMU 기록 자동 활성화)')
    parser.add_argument("--auto-stop", type=float)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--quit-after-save", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    window = MappingWindow(Session(), args.output_root, args.demo, args.imu_calibration, args.imu_fusion)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    if args.auto_stop:
        window.start()
        QTimer.singleShot(int(args.auto_stop*1000), window.session.stop)
    def check_finished() -> None:
        if window.session.snapshot()["state"] in {"complete", "partial", "failed"} and not window.session.thread.is_alive():
            window.refresh()
            if args.screenshot:
                args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                window.grab().save(str(args.screenshot))
            if args.quit_after_save:
                window.close()
                app.quit()
    finish_timer = QTimer()
    finish_timer.timeout.connect(check_finished)
    finish_timer.start(300)
    app.exec_()


if __name__ == "__main__":
    main()

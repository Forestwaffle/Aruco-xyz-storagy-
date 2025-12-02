# 🤖 Aruco-LiDAR Follower (ROS2 Foxy/Humble)

**ArUco 마커** 기반의 정밀 추적 기능과 **LiDAR** 기반의 실시간 장애물 회피 기능을 통합한 자율 이동 로봇(AMR) 제어 노드입니다.  
특정 마커를 목표(0.6m 거리)로 설정하고, **탐색 → 추적 → 정지 → 회피** 의 단계적 로직을 통해 목표 지점에 안전하고 정확하게 도킹하는 것을 목표로 합니다.

---

## 🌟 주요 기능 및 활용

| 구분 | 내용 |
| :--- | :--- |
| **정밀 추적** | ArUco 마커 기준 **3cm 오차 범위** 내 정지 |
| **안전 주행** | LiDAR 기반 **35cm 이내 장애물 감지 → 긴급 회피** |
| **활용 분야** | 도킹, 물류 로봇, 연구/교육용 센서 퓨전 제어 등 |

---

## 📐 코드 로직 요약

### 1️⃣ 센서 데이터 처리 (콜백 함수)

| 함수 | 센서 | 기능 |
| :--- | :--- | :--- |
| `scan_callback` | LiDAR | 전방 거리 측정 → 장애물 감지 플래그 업데이트 |
| `image_callback` | Camera | ArUco 마커 검출 → `cv2.solvePnP` 자세 추정 → P 제어 기반 추적 속도 계산 |

> X, Y, Z 모두 **0.03m 이내**일 경우 자동 정밀 정지

---

### 2️⃣ 메인 제어 루프 (`timer_callback`)
**50ms (20Hz)** 주기 동작, 최종 속도 명령(`/cmd_vel`) 발행

| 우선순위 | 상태 | 조건 | 명령 |
| :--- | :--- | :--- | :--- |
| **P1(최우선)** | 추적/정지 | 마커 감지 시 | 이미지 기반 P-제어 속도 |
| **P2** | 장애물 회피 | 마커 미검출 + 장애물 | 정지 후 오른쪽 급회전 |
| **P3** | 탐색 | 위 조건 모두 아님 | 저속 전진 |

---

## 🛠 기술 스택

- **ROS2 (Foxy / Humble 지원)**
- Python 3
- OpenCV + ArUco
- `cv_bridge`, `geometry_msgs/Twist`
- `sensor_msgs/Image`, `sensor_msgs/LaserScan`
- Proportional Control(P-Control)

---

## 📦 설치 및 실행

### 🔧 필수 패키지 설치

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# Python 패키지
pip install numpy opencv-python

# ROS 종속 패키지
sudo apt install ros-humble-cv-bridge

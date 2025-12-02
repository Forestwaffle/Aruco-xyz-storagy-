import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoLidarFollowerNode(Node):
    
    def __init__(self):
        super().__init__('aruco_lidar_follower_node')

        # 1. ROS2 설정 및 통신
        self.bridge = CvBridge()  # ROS 이미지 <-> OpenCV 이미지 변환 도구
        self.img_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)  # 컬러 카메라 이미지 수신
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)  # 2D 라이다 데이터 수신
        self.img_publisher = self.create_publisher(Image, '/aruco_result_img', 10)  # 마커 검출 결과를 그린 이미지 발행
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # 로봇 속도 명령 발행 (/cmd_vel)

        # 2. ArUco 및 카메라 설정
        self.TARGET_ID = 0  # 우리가 따라가야 할 특정 마커의 ID (예: ID 0)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)  # 임의의 카메라 내부 파라미터 (fx, fy, cx, cy)
        self.dist_coeffs = np.zeros((5, 1))  # 렌즈 왜곡 계수 없음 (실제 캘리브레이션 필요)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # 4x4 비트, 50개 마커 사전
        self.parameters = cv2.aruco.DetectorParameters()  # 마커 검출 파라미터 기본값
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)  # OpenCV 4.7+ 방식의 검출기
        self.marker_length = 0.05  # 실제 마커 한 변의 길이 = 5cm
        ms = self.marker_length / 2.0
        self.marker_points = np.array([[-ms, ms, 0], [ms, ms, 0], [ms, -ms, 0], [-ms, -ms, 0]], dtype=np.float32)  # 마커의 4개 코너 3D 좌표 (마커 중심이 원점)
 
        # 3. 제어 목표 및 게인
        self.TARGET_DIST = 0.6        # 마커와 유지하고 싶은 이상적인 거리 = 60cm
        self.KP_LINEAR = 0.5          # 거리 오차 → 전진/후진 속도 비례 게인
        self.KP_ANGULAR = 1.5         # 좌우 위치 오차 → 회전 속도 비례 게인
        self.OBSTACLE_DISTANCE_TH = 0.35  # 이 거리(30cm) 이하면 장애물로 판단
        self.MAX_LINEAR_SPEED = 0.2   # 선속도 최대값 제한 (안전)
        self.MAX_ANGULAR_SPEED = 1.0  # 각속도 최대값 제한
        self.EXPLORE_LINEAR_SPEED = 0.15 # 마커를 못 찾았을 때 천천히 전진하는 속도 (탐색용)

        # 4. 상태 플래그 및 데이터
        self.obstale_detect_flag = False  # 범위 내 장애물 여부
        self.target_found = False         # 이번 이미지에서 목표 마커를 찾았는지 여부
        self.latest_linear_x = 0.0        # image_callback에서 계산된 가장 최근 선속도 명령
        self.latest_angular_z = 0.0       # image_callback에서 계산된 가장 최근 각속도 명령
        self.latest_z_pos = 0.0           # 마지막으로 측정된 마커까지의 전방 거리 (디버그용)
        
        # 5. 제어 타이머: 50ms마다 최종 명령을 발행
        timer_period = 0.05  # 20Hz 주기 (50ms)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("ArUco-Lidar Follower Started. Target ID: {}".format(self.TARGET_ID))

    # -----------------------------------------------------------------
    # 센서 콜백 함수
    # -----------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """Lidar 데이터를 받아 장애물 감지 플래그를 업데이트합니다."""
        self.obstale_detect_flag = False  # 기본값: 장애물 없음
        
        setrange = msg.ranges[205:730]
        for distance in setrange:
            
            if distance <= self.OBSTACLE_DISTANCE_TH and distance > 0.02:  # 2cm 이하는 노이즈로 간주
                self.obstale_detect_flag = True
                break  
                
    def image_callback(self, msg: Image):
        """이미지 처리 및 추적 명령 계산, XYZ 시각화를 담당합니다."""
        MARKER_STOP_TH = 0.03 # 0.02m 근처 정지 판단을 위한 임계값
        
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS Image → OpenCV BGR 이미지
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환 (ArUco는 흑백에서 검출)
            corners, ids, rejected = self.detector.detectMarkers(gray)  # ArUco 마커 검출

            # 이번 프레임에서는 목표를 못 찾았다고 초기화
            self.target_found = False
            self.latest_linear_x = 0.0
            self.latest_angular_z = 0.0

            # 마커가 하나라도 검출되었다면
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)  # 모든 검출된 마커를 이미지에 그림
                
                # 우리가 찾는 TARGET_ID가 검출된 마커들 중에 있는지 확인
                if self.TARGET_ID in ids.flatten():
                    index = list(ids.flatten()).index(self.TARGET_ID)  # 해당 ID의 인덱스 찾기
                    target_corners = corners[index].reshape((4, 2))    # 4개의 코너 좌표 (2D 이미지 좌표)
                    self.target_found = True  # 목표 발견!
                    
                    # solvePnP로 마커의 3D 위치와 방향 추정
                    success, rvec, tvec = cv2.solvePnP(self.marker_points, target_corners, self.camera_matrix, self.dist_coeffs)

                    if success:
                        # 카메라 좌표계 기준으로 XYZ축 그리기 (길이 5cm)
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        
                        x_pos = tvec[0][0]   # 카메라 기준 좌우 위치 (오른쪽이 양수)
                        y_pos = tvec[1][0]   # 카메라 기준 상하 위치 (아래쪽이 양수)
                        z_pos = tvec[2][0]   # 카메라 기준 전방 거리 (m)
                        self.latest_z_pos = z_pos 

                        # --- P-Control: 추적 명령 계산 및 저장 ---
                        # 좌우 정렬: 마커가 오른쪽에 있으면 왼쪽으로 회전 (그래서 - 부호)
                        self.latest_angular_z = max(min(-self.KP_ANGULAR * x_pos, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
                        error_distance = z_pos - self.TARGET_DIST  # 거리 오차
                        
                        if z_pos > 0.02:  # 2cm 이하는 정지 (기존 충돌 방지 로직)
                            linear_raw = self.KP_LINEAR * error_distance
                            self.latest_linear_x = max(min(linear_raw, self.MAX_LINEAR_SPEED), -0.1)  # 후진 허용
                        
                        # ----------------------------------------------------------------------
                        # 미세 정지 영역 (X, Y, Z가 0.02m 근처에 있을 때) 로직
                        # ----------------------------------------------------------------------
                        # X, Y, Z 축 모두 MARKER_STOP_TH (0.03m) 이내에 들어왔는지 확인
                            
                        # 1. Z축 조건: 현재 Z 위치가 범위 내인지 확인
                        z_is_close_to_target = abs(error_distance) < MARKER_STOP_TH
                        
                        # 2. X, Y축 조건: X, Y 위치가 중앙(0.0)에 가까운지 확인
                        x_is_centered = abs(x_pos) < MARKER_STOP_TH
                        y_is_centered = abs(y_pos) < MARKER_STOP_TH

                        if (x_is_centered and 
                            y_is_centered and 
                            z_is_close_to_target):

                            # 최종 명령을 0.0으로 덮어씀 (정지)
                            self.latest_linear_x = 0.0
                            self.latest_angular_z = 0.0
                            
                            # 정지 상태를 시각화 텍스트에 표시
                            cv2.putText(cv_image, "TARGET STOP ZONE (0.6m)", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        # ----------------------------------------------------------------------


                        # 시각화 텍스트 (XYZ 좌표 및 상태)
                        status_text = f"ID:{self.TARGET_ID} | Dist:{z_pos:.2f}m | X:{x_pos:.2f}"
                        cv2.putText(cv_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        pose_text_x = f"X: {x_pos:.3f}m (Right+)"
                        pose_text_y = f"Y: {y_pos:.3f}m (Down+)"
                        pose_text_z = f"Z: {z_pos:.3f}m (Forward+)"

                        cv2.putText(cv_image, pose_text_x, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(cv_image, pose_text_y, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(cv_image, pose_text_z, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 목표 마커를 못 찾았을 때 화면에 표시
            if not self.target_found:
                 cv2.putText(cv_image, "Target Not Found (Searching)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 마커 검출 결과가 그려진 이미지 퍼블리시 (rviz에서 확인 가능)
            self.img_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    # -----------------------------------------------------------------
    # 메인 제어 루프 (무조건 오른쪽 회전 탐색 적용)
    # -----------------------------------------------------------------
    def timer_callback(self):
        """
        [로직 우선순위]
        1. 목표 마커 추적 (LiDAR 완전 무시)
        2. LiDAR 장애물 회피 (태그 미감지 시)
        3. 태그 탐색 (장애물/태그 미감지 시) -> 무조건 오른쪽 회전
        """
        cmd_msg = Twist()  # 속도 명령 메시지 생성
        
        # 1. 목표 마커 추적 (P1) - 마커 보이면 최우선으로 추종
        if self.target_found:
            cmd_msg.linear.x = self.latest_linear_x
            cmd_msg.angular.z = self.latest_angular_z
            self.get_logger().info(f"추적 (P1): 태그 감지. X:{cmd_msg.linear.x:.2f}, Z:{cmd_msg.angular.z:.2f}")

        # 2. LiDAR 장애물 회피 (P2) - 마커 안 보일 때만 작동
        elif self.obstale_detect_flag:
            cmd_msg.linear.x = 0.0      # 정지
            cmd_msg.angular.z = -0.8    # 오른쪽으로 급회전해서 회피
            self.get_logger().info("회피 (P2): 태그 미감지 & 장애물 감지.")
            
        # 3. 태그 탐색 (P3) - 마커도 없고 장애물도 없을 때
        else:
            # 탐색 명령 실행: 천천히 전진
            cmd_msg.linear.x = self.EXPLORE_LINEAR_SPEED # 0.15 m/s 전진
            cmd_msg.angular.z = 0.0 
            self.get_logger().info(f"탐색 (P3): 탐색 중. X:{cmd_msg.linear.x:.2f}, Z:{cmd_msg.angular.z:.2f}")

        # 최종 명령 발행
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLidarFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


 
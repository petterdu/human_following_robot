import rospy
import torch
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.5  # 신뢰도 임계값 설정

        # ROS 관련 초기화
        self.bridge = CvBridge()
        self.camera_flip = False
        self.image_sub = rospy.Subscriber("/camera_topRGBD/color/image_raw", Image, self.image_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera_topRGBD/depth/points", PointCloud2, self.point_cloud_callback)
        self.camera_info_sub = rospy.Subscriber("/camera_topRGBD/depth/camera_info", CameraInfo, self.camera_info_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.command_sub = rospy.Subscriber('/gui_command', String, self.command_callback)

        self.distance_pub = rospy.Publisher('/distance', String, queue_size=10)  # String 타입으로 퍼블리시
        self.person_coords_pub = rospy.Publisher('/person_coordinates', String, queue_size=10)  # 좌표 퍼블리시

        # 변수 초기화
        self.selected_person_coords = None
        self.point_cloud = None
        self.camera_info = None
        self.following = False  # 초기에는 Following이 False 상태여야 함
        self.selection_mode = False  # 선택 모드 플래그

    def camera_info_callback(self, data):
        self.camera_info = data

    def command_callback(self, msg):
        command = msg.data
        if command == "start_selection":
            rospy.loginfo("Selection mode activated.")
            self.selection_mode = True
        elif command == "confirm_person":
            rospy.loginfo("Person confirmed.")
            self.selection_mode = False
            # self.following을 설정하지 않음: 사람이 확인되었지만, 따라가는 것은 Human Following 버튼에 의해서만 제어됨
            cv2.destroyAllWindows()  # 선택 완료 후 창 닫기
            if self.selected_person_coords:
                coords_str = ','.join(map(str, self.selected_person_coords))
                self.person_coords_pub.publish(coords_str)  # 좌표 퍼블리시
                rospy.loginfo(f"Published person coordinates: {coords_str}")
        elif command == "reject_person":
            rospy.loginfo("Person rejected.")
            self.selection_mode = True
        elif command == "start_following":
            rospy.loginfo("Starting to follow the person...")
            self.following = True  # 버튼을 눌렀을 때만 following 시작
        elif command == "stop_following":
            rospy.loginfo("Stopped following the person.")
            self.following = False  # Following을 중지

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.camera_flip:
                frame = cv2.flip(frame, 1)

            if self.selection_mode:
                results = self.model(frame[..., ::-1])
                persons = [x for x in results.xyxy[0] if x[-1] == 0 and x[-2] > 0.5]

                if len(persons) > 0:
                    x1, y1, x2, y2 = map(int, persons[0][:4])
                    self.selected_person_coords = (x1, y1, x2, y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cv2.imshow("Selection Mode", frame)
                cv2.waitKey(1)

            if self.following:
                results = self.model(frame[..., ::-1])
                persons = [x for x in results.xyxy[0] if x[-1] == 0 and x[-2] > 0.5]

                if len(persons) > 0:
                    x1, y1, x2, y2 = map(int, persons[0][:4])
                    self.selected_person_coords = (x1, y1, x2, y2)
                    rospy.loginfo(f"Person selected at coordinates: {self.selected_person_coords}")
                else:
                    self.selected_person_coords = None

        except CvBridgeError as e:
            rospy.logerr(f"Could not convert from '{data.encoding}' to 'bgr8'. {e}")

    def point_cloud_callback(self, data):
        self.point_cloud = data
        if self.selected_person_coords is not None and self.following:  # following 상태에서만 거리 계산
            self.calculate_closest_distance()

    def calculate_closest_distance(self):
        if self.point_cloud is None or self.selected_person_coords is None or self.camera_info is None:
            return

        x1, y1, x2, y2 = self.selected_person_coords
        min_distance = float('inf')

        for point in pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # 3D Point를 2D 이미지 좌표로 변환
            img_x = int((x * self.camera_info.K[0] / z) + self.camera_info.K[2])
            img_y = int((y * self.camera_info.K[4] / z) + self.camera_info.K[5])

            if x1 <= img_x <= x2 and y1 <= img_y <= y2:
                if z > 0:  # 유효한 깊이 값
                    min_distance = min(min_distance, z)

        if min_distance != float('inf'):
            self.distance_pub.publish(str(min_distance))  # Distance를 String으로 퍼블리시
            if self.following:  # self.following이 True일 때만 로봇을 제어
                self.control_robot(min_distance, x1, y1, x2, y2)

    def control_robot(self, min_distance, x1, y1, x2, y2):
        if not self.following:
            return  # following이 False인 경우 로봇 제어 중지

        stop_distance = 0.7  # 로봇이 멈추기를 원하는 거리(m)
        max_speed_distance = 2.0  # 최대 속도로 이동할 거리(m)
        min_speed = 0.05  # 최소 이동 속도
        max_speed = 0.2   # 최대 이동 속도

        if min_distance <= stop_distance:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo(f"Person is within {stop_distance} meters, stopping.")
        else:
            error_x = (x1 + x2) / 2 - (self.camera_info.width / 2)
            speed = max(min_speed, max_speed * ((min_distance - stop_distance) / (max_speed_distance - stop_distance)))
            speed = min(speed, max_speed)

            twist = Twist()
            twist.linear.x = speed

            if self.camera_flip:
                twist.angular.z = error_x * 0.002  # 카메라가 반전된 경우 회전 방향을 반대로
            else:
                twist.angular.z = -error_x * 0.002  # 기본 회전 방향

            self.cmd_vel_pub.publish(twist)
            rospy.loginfo(f"Following person: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}, distance={min_distance:.2f} m")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    controller = RobotController()
    controller.run()

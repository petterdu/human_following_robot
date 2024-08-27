import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class LiveVideoFeed:
    def __init__(self):
        # 고유한 노드 이름 사용
        rospy.init_node('live_video_feed_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_topRGBD/color/image_raw", Image, self.image_callback)
        self.person_coords_sub = rospy.Subscriber("/person_coordinates", String, self.person_callback)
        
        self.selected_person_coords = None
        self.frame = None

    def person_callback(self, msg):
        # 수신된 좌표를 로그로 출력
        self.selected_person_coords = tuple(map(int, msg.data.split(',')))
        rospy.loginfo(f"[LiveVideoFeed] Received person coordinates: {self.selected_person_coords}")

    def image_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo(f"[LiveVideoFeed] Received image with shape: {self.frame.shape}")  # 프레임 크기 확인
            self.draw_rectangle(self.frame)
            cv2.imshow("Live Video Feed", self.frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"[LiveVideoFeed] Could not convert from '{data.encoding}' to 'bgr8'. {e}")

    def draw_rectangle(self, frame):
        if self.selected_person_coords:
            x1, y1, x2, y2 = self.selected_person_coords
            rospy.loginfo(f"[LiveVideoFeed] Drawing rectangle at coordinates: {self.selected_person_coords}")  # 로그 추가
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            rospy.loginfo(f"[LiveVideoFeed] Rectangle drawn at: ({x1}, {y1}), ({x2}, {y2})")  # 로그 추가

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    video_feed = LiveVideoFeed()
    video_feed.run()

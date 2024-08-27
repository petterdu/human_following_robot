import sys
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton
from std_msgs.msg import String

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control GUI")
        self.setGeometry(100, 100, 300, 200)

        # 메인 위젯과 레이아웃 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout(central_widget)

        # Distance 라벨 추가
        self.distance_label = QLabel("Distance: N/A", self)
        self.distance_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        self.layout.addWidget(self.distance_label)

        # Person Selection 버튼
        self.person_selection_button = QPushButton("Person Selection", self)
        self.person_selection_button.setStyleSheet("font-size: 16px; padding: 10px;")
        self.person_selection_button.clicked.connect(self.open_selection_window)
        self.layout.addWidget(self.person_selection_button)

        # Human Following 버튼
        self.human_following_button = QPushButton("Human Following: OFF", self)
        self.human_following_button.setCheckable(True)
        self.human_following_button.setStyleSheet("font-size: 16px; padding: 10px; background-color: #f0f0f0;")
        self.human_following_button.clicked.connect(self.toggle_human_following)
        self.layout.addWidget(self.human_following_button)

        # ROS 퍼블리셔 및 구독자 설정
        self.command_pub = rospy.Publisher('/gui_command', String, queue_size=10)
        rospy.Subscriber('/distance', String, self.update_distance)
        rospy.init_node('pyqt_control_gui', anonymous=True)

    def update_distance(self, msg):
        distance = float(msg.data)  # 문자열을 float로 변환
        self.distance_label.setText(f"Distance: {distance:.2f} m")

    def open_selection_window(self):
        self.command_pub.publish("start_selection")
        self.selection_window = PersonSelectionWindow(self)
        self.selection_window.show()

    def toggle_human_following(self):
        if self.human_following_button.isChecked():
            self.human_following_button.setText("Human Following: ON")
            self.human_following_button.setStyleSheet("background-color: #00ff00; font-size: 16px; padding: 10px;")
            self.command_pub.publish("start_following")
        else:
            self.human_following_button.setText("Human Following: OFF")
            self.human_following_button.setStyleSheet("background-color: #f0f0f0; font-size: 16px; padding: 10px;")
            self.command_pub.publish("stop_following")

class PersonSelectionWindow(QMainWindow):
    def __init__(self, main_window):
        super().__init__()
        self.setWindowTitle("Select Person")
        self.setGeometry(200, 200, 300, 200)

        self.main_window = main_window

        self.label = QLabel("Is this the person?", self)
        self.label.setGeometry(10, 10, 280, 40)

        self.yes_button = QPushButton("Yes", self)
        self.yes_button.setGeometry(30, 100, 100, 40)
        self.no_button = QPushButton("No", self)
        self.no_button.setGeometry(170, 100, 100, 40)

        self.yes_button.clicked.connect(self.confirm_person)
        self.no_button.clicked.connect(self.reject_person)

    def confirm_person(self):
        self.main_window.command_pub.publish("confirm_person")
        self.close()

    def reject_person(self):
        self.main_window.command_pub.publish("reject_person")
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

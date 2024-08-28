# Human Following Robot with ROS and YOLOv5

이 프로젝트는 ROS, YOLOv5, 그리고 뎁스 카메라를 사용하여 인간을 따라가는 로봇을 구현합니다. GUI를 통해 선택된 사람을 로봇이 인식하고, 일정 거리를 유지하면서 따라갑니다. 시스템은 ROS와 Python을 기반으로 하며, PyQt를 GUI로, OpenCV를 이미지 처리에 사용합니다.

## Prerequisites

이 환경을 설정하기 전에, Conda와 ROS Noetic이 설치되어 있어야 합니다.

### 필수 플랫폼 및 라이브러리

1. **ROS Noetic** - ROS(로봇 운영 체제)는 로봇 소프트웨어 개발을 위한 오픈 소스 프레임워크입니다.
2. **Conda** - 패키지 관리 및 환경 관리를 위한 툴.
3. **Python 3.8** - 이 프로젝트는 Python 3.8을 사용합니다.

### ROS 패키지 설치

필수 ROS 패키지를 설치합니다:

```bash
sudo apt-get install ros-noetic-ros-base
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-cv-bridge ros-noetic-image-transport
```

### Conda 환경 설정 및 라이브러리 설치

1. **Conda 환경 생성 및 활성화**:

    ```bash
    conda create -n human_following_robot python=3.8
    conda activate human_following_robot
    ```

2. **Python 패키지 설치**:

    ```bash
    pip install PyQt5==5.14.2
    pip install rospkg
    pip install torch==1.9.0+cpu torchvision==0.10.0+cpu torchaudio==0.9.0 --extra-index-url https://download.pytorch.org/whl/cpu
    pip install opencv-python
    pip install numpy
    pip install matplotlib
    ```

3. **추가 시스템 라이브러리 설치**:

    ```bash
    conda install libffi==3.3
    ```

## 코드 실행 방법

1. **ROS Core 실행** (새 터미널에서):

    ```bash
    roscore
    ```

2. **OpenCV 처리 노드 실행**:

    ```bash
    python together_opencv_processing.py
    ```

3. **PyQt GUI 실행**:

    ```bash
    python together_pyqt_gui.py
    ```

4. **로봇 컨트롤러 및 기능 노드 실행**:

    ```bash
    python together_feature.py
    ```

## 기능

- **Person Selection**: GUI를 사용하여 로봇이 따라갈 사람을 선택할 수 있습니다. YOLOv5가 사용되어 사람을 탐지하고 선택합니다.
- **Human Following**: 선택된 사람을 따라가도록 GUI에서 "Human Following" 버튼을 눌러 시작할 수 있습니다. 로봇은 사람과의 안전 거리를 0.7m 유지합니다.
- **Distance Display**: GUI는 실시간으로 선택된 사람과 로봇 간의 거리를 표시합니다.

## Troubleshooting

- 패키지 설치 문제 발생 시 Conda 환경이 올바르게 활성화되었는지 확인하세요.
- ROS 환경 변수가 올바르게 설정되었는지 확인하세요.
```

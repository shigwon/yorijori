
# Robotics


## 📦 Snapd 설치 (로컬 snap 파일로 설치 시)

```bash
snap download snapd --revision=24724
sudo snap ack snapd_24724.assert
sudo snap install snapd_24724.snap
```

---

## 📡 ROS 2 토픽 퍼블리싱 예시

차량 제어용 메시지를 퍼블리싱할 때 사용하는 명령입니다:

```bash
ros2 topic pub /vehicle_control std_msgs/Float32MultiArray "{data: [0, 0]}"
```

> ✅ `Angle`: -60° ~ 60° 범위 내로 입력

---

## 🧪 Isaac ROS 개발 컨테이너 실행

Isaac ROS 패키지의 개발 컨테이너를 실행하려면 다음 명령을 입력하세요:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

---

## ⚙️ NVIDIA 컨테이너 CDI 설정

NVIDIA CDI(Container Device Interface) 구성을 위해 아래 명령을 실행합니다:

```bash
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
```

---

## 🐍 Python 패키지 오류 해결 (setuptools_scm)

`setuptools_scm` 관련 오류가 발생할 경우 다음 명령어로 해결할 수 있습니다:

```bash
pip install setuptools_scm==5.0.0
```

---

## 🛠️ Isaac ROS Visual SLAM 설치 (ROS 2 Humble)

```bash
sudo apt-get install -y ros-humble-isaac-ros-visual-slam
```

---

## 🗺️ SLAM 관련 명령/설정

- Visual SLAM 패키지 설치 후 Realsense 등 센서와 연동하여 SLAM 노드를 실행 가능
- 자세한 사용법은 `isaac_ros_visual_slam` 패키지의 launch 파일 및 공식 문서를 참고

---

## 📁 기타 참고

- ROS 2 환경 설정 파일이 제대로 로드되었는지 확인:  
  ```bash
  source ~/ros2_ws/install/local_setup.bash
  ```

- `cb`와 같은 alias 설정은 `.bashrc`에 추가 가능:
  ```bash
  export ROS2_WS=~/ros2_ws
  alias cb='cd $ROS2_WS && colcon build --symlink-install && source $ROS2_WS/install/local_setup.bash'
  ```

---

## 🔗 참고 문서

- [Isaac ROS 공식 GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [CDI(NVIDIA Container Device Interface) 공식 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#cdi)
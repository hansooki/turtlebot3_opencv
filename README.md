# turtlebot3_opencv

# robot_housewife
자율주행로봇 로봇청소기엄마, 가정관리사, 가정부, 로봇노예, 로봇집사, 로봇가정비서, 자비스

- - -
11/6 
- - -
slam/navi 작동
노트북 캠: ros2 run usb_cam usb_cam_node_exe
rasicam-pub_tb3_pose2d.py-img_compressed2raw.py-aruco_node.py-track_maker2.

- - -
11/7 
- - -
cv_bridge 이용한 ros2 pub,sub 작성 

publising

![Screenshot from 2022-11-18 11-38-49](https://user-images.githubusercontent.com/112480482/202604390-327d1f9c-015f-42d2-871f-b223c8b27095.png)

subsribe

![Screenshot from 2022-11-18 11-39-30](https://user-images.githubusercontent.com/112480482/202604399-17a192cd-c418-4142-a64e-fa17a9695889.png)

video 출력

![video](https://user-images.githubusercontent.com/112480482/202604423-f87d1d74-d445-42a3-a207-0e5c9bc8cff9.png)

- - -
11/8 
- - -
노트북캠에 완성한 pub, sub에 yolo를 추가하여 구동 성공

- - -
11/9 
- - -
인식한 객체를 둘러쌓은 박스의 크기(1m기준)를 기준으로 크기(픽셀)에 비례하여 거리를 측정할려하였으나 박스인식이 유동적이여서 처음 객체를 인식했을 때 정중앙의 좌표를 구해 그 방향으로 터틀봇3의 각도를 돌려놓고 라이다로 거리 측정 후 특정 위치까지 전진 후 매니퓰레이터 작동해야겠다 구상

- - -
11/10 
- - -
터틀봇3와 ros2 sub 구동 성공

    -robot_ws/src/cv_basics/pub,sub 수정

![Screenshot from 2022-11-18 11-57-00](https://user-images.githubusercontent.com/112480482/202607015-10e69a95-898f-4686-a4fd-a681f1a8aeb4.png)

- - -
#11.24
- - -
https://github.com/clydemcqueen/opencv_cam
~ 등등 예제 실행

- - -
#11.25
- - -
https://github.com/clydemcqueen/opencv_cam 예제 활용해 publisher하는 노드(C++)를 통해 기존에 만들어놓은 subscriber(python)을 통해 받으려 시도
compressd_image를 image_raw로 수정
![20221125_124537](https://user-images.githubusercontent.com/112480482/204091952-dabe8bc3-1a5c-4c59-b852-dedbec73640b.jpg)
![20221125_124748](https://user-images.githubusercontent.com/112480482/204091955-f4bfcc37-20d0-4237-9fe9-09733cc3cf83.jpg)
허나 위와 같이 초록색 화면뜸



- - -
#11.27
- - -
opencv_cam node 코드에 right,left 화면이 나오는 simple_camera 코드로 변형하여 opencv_cam을 실행할 수 있게함

- - -
11. 29 
- - -
저장공간 부족으로 포맷
xubuntu_20.04 image 굽기 
터틀봇3 폭시 https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

라즈베리파이 구성
1. 자동 업데이트 설정 파일 변경

	sudo nano /etc/apt/apt.conf.d/20auto-upgrades

	APT::Periodic::Update-Package-Lists "0";
	APT::Periodic::Unattended-Upgrade "0";

2. systemd 시작 시 네트워크가 없더라도 부팅 지연을 방지 설정, systemd 다음 명령을 사용하여 프로세스 마스크를 설정하려면 아래 명령을 실행

	systemctl mask systemd-networkd-wait-online.service

3. 일시 중단 및 최대 절전 모드 비활성화

	sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

-----
Turtlebot3_waffle
-----

4. ROS2 Foxy Fiztroy 설치

	$ sudo apt update
	$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh
	$ chmod 755 ./install_ros2_foxy_rpi.sh
	$ bash ./install_ros2_foxy_rpi.sh

5. ROS 패키지를 설치하고 빌드.

	$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-   essential				     //colcon build system 설치
	$ sudo apt install ros-foxy-hls-lfcd-lds-driver	//레이저 드라이버 설치
	$ sudo apt install ros-foxy-turtlebot3-msgs	//터틀봇3 메세지(토픽,서비스,액션) 설치
	$ sudo apt install ros-foxy-dynamixel-sdk	//다이나믹셀 모터드라이버 설치
	$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src	//ws, src 생성
	$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git	//터틀봇3 전체 소스 받기
	$ cd ~/turtlebot3_ws/src/turtlebot3
	$ rm -r turtlebot3_cartographer turtlebot3_navigation2	//cartographer, navigation2 삭제
	$ cd ~/turtlebot3_ws/
	$ echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc	//bashrc에 setup 파일 추가
	$ source ~/.bashrc	
	$ colcon build --symlink-install --parallel-workers 4	//나누는 코어가 4개여서 1에서 4로 수정
	$ echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
	$ source ~/.bashrc
	
6.OpenCR용 USB 포트 설정	

	$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
	$ sudo udevadm control --reload-rules
	$ sudo udevadm trigger

7. TurtleBot3 의 ID 30으로 설정

	$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
	$ source ~/.bashrc

8. LDS 구성

	$ sudo apt update
	$ sudo apt install libudev-dev
	$ cd ~/turtlebot3_ws/src
	$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git //devel driver 설치
	$ cd ~/turtlebot3_ws && colcon build --symlink-install
	
	
9.LDS 번호 LDS-01 설정	
	$ echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
	$ source ~/.bashrc
	
10.OpenCR 펌웨어 를 업로드하려면 필요한 패키지를 Raspberry Pi에 설치
	$ sudo dpkg --add-architecture armhf
	$ sudo apt update
	$ sudo apt install libc6:armhf

11. OPENCR_MODEL 이름에 waffle
	$ export OPENCR_PORT=/dev/ttyACM0
	$ export OPENCR_MODEL=waffle
	$ rm -rf ./opencr_update.tar.bz2	//기존 opencr 삭제

12. 펌웨어 및 로더를 다운로드한 다음 파일을 추출합니다
	
	$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
	$ tar -xvf ./opencr_update.tar.bz2
	
13. OpenCR에 펌웨어를 업로드합니다.
	
	$ cd ~/opencr_update
	$ ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

#sudo apt install ssh	//해야 ssh 원격접속 가능

14. .bashrc에 export TURTLEBOT3_MODEL=waffle 추가 후 bringup

---------------
Openmanipulator
---------------

15. OpenMANIPULATOR-X용 종속 패키지를 설치 & colcon_ws 생성

	$ sudo apt install ros-foxy-rqt* ros-foxy-joint-state-publisher
	
	$ cd ~/colcon_ws/src/
	$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
	$ git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
	$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
	$ git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
	$ git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
	$ git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git
	$ cd ~/colcon_ws && colcon build --symlink-install

16. open_manipulator_x_controlleer.launch.py에 31번째줄 '/dev/ttyUSB0'을 USB1로 변경

17. 명령어로 제공되는 스크립트로 ROS를 설치
	$ sudo apt update
	$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
	$ chmod 755 ./install_ros2_foxy.sh
	$ bash ./install_ros2_foxy.sh
18. usb 포트 설정 문제
opencr과 manipulator 중 usb 0,1이 분별이 안돼, manipulator의 연결을 끊고 reboot해 기존의 usb0은 opencr로 usb1을 manipulator로 설정함

19. 브링업

	$ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

20. 텔레오피키
	$ ros2 run open_manipulator_x_teleop teleop_keyboard
실행하였으나 manipulator 정상 작동안함. 빨간불들어오며 모터에 힘이 안들어감. 토크 조여주고 u2d2 재부팅 후 bringup 후 실행하니 정상 작동

21. urtlebot3 의 브링업과 open_manipulator 의 브링업이 충돌을일으김. (토픽명, joint_states)
	ros2 launch turtlebot3_bringup robot.launch.py
	ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
	teleop_keyboard.py -> 109 joint_states --->joint_states2
	src/open_manipulator/open_manipulator_x_controller/src/open_manipulator_x_controller.cpp ->87 joint_states --->joint_States2
	topic 이름을 바꾸어서 해결
	!빌드해야 적용 

22. git clone -b foxy https://github.com/ros-perception/image_comon.git 하여 카메라 캘러브레이션 파서 설치
---
opencv
---

23. 기존의 subscriber(webcam_sub.py)에
	self.subscription = self.create_subscription(Image, 
                'image_raw'를 'image_raw1'로 변경
24. from cv_bridge import CvBridge 보다 import cv2를 먼저 import한다. 결과 영상 나옴

# 11/30

1. 기존 opencv_cam 코드의 img_raw1, img_raw2가 동일한 이미지로 나와 opencv_cam 을 opencv_cam1, opencv_cam2로 나눔. 이후 subscriber로 img_raw1,2를 받아 화면에 두 영상을 띠움

2. yolo 넣어 적용 실패 --> TypeError: only integer scalar arrays can be converted to a scalar index 오류가 output_layers = [layer_names[i - 1] for i in YOLO_net.getUnconnectedOutLayers()]에서 발생함, [layer_names[i[0] - 1]을 수정하여 해결.

3. left, right 카메라로 depthmap 예제를 활용해 거리에 따라 차이를 나타냄

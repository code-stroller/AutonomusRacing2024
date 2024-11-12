# AutonomusRacing2024

For growth of technical issue


# Overview
##  1. System Architecrue
- [1.0 Hardware](-hardware)
  -   [computer](-computer)
  -   [ lidar](-lidar)
  -   [ vision](-vision)
  -  [gps](-gps)
  - [obs](-osb)

- [1.1 Software](-software)
  - [OS](-os)
  - [ROS](-total)
  - [catkine workspace](#catkin-worksapce)
  - [control package](#control-package)
  - [lidar package](#lidar-package)
  - [vision package](#vision-package)


## 2. How to Use
- [2.0 How to install package](-install)
- [2.1 How to Build](-build)
- [2.2 How to Run](#-Run)
  - [final ](final)
  - [pre ](pre)
  - [package](pacakge)
  - [util](rviz)
- [2.3 How to develope](#-develope)

## 3. Results

- [final results](final)
- [pre results](pre)
- [rviz examples](pacakge)



## 4. Contributors



# catkin workspace
this is our workspace tree

```
base_ws
└── src
    ├── control
    │   └── README.md
    ├── lidar
    │   └── README.md
    └── vision
        └── README.md

```

# Lidar package

<!--lidar packae에 대한 설명을 담아주세요! -->

# Vision package

<!--vision packae에 대한 설명을 담아주세요! -->

##  spinnaker_camera_driver

spinnaker 의카메라 센서를 키고 조작하기 위한 패키지입니다.

## yolov7_ros

yolov7_ros는 yolo를 사용하며 신호등인식과 표지판인식을 수행하여, 신호등의 값과 표지판의 좌표를 제어프로그램에 전달하기 위한 패키지입니다.

```bash

#yolo와 토픽 발행 모든것을 실행시킵니다.

roslaunch yolov7_ros vision_all

```
# Control package

<!--control  packae에 대한 설명을 담아주세요! -->


# How to Set up

## 1.  build

```

git clone https://github.com/Ajou-Nice/AutonomusRacing2024
cd AutonomusRacing2024
catkin_make


```

## 2.  Run



```
roslaunch example example.launch
```

# How to develope

If you  want add more functions or discriptions following below process. we will follow this [example](https://velog.io/@diduya/git-%ED%9A%A8%EC%9C%A8%EC%A0%81%EC%9D%B8-%ED%98%91%EC%97%85%EC%9D%84-%EC%9C%84%ED%95%9C-Git-Flow-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0-git-branch-repository)

```bash

git clone https://github.com/Ajou-Nice/AutonomusRacing2024
cd AutonomusRacing2024

# develope is our main develope branch
git pull develope

# move branch to develope
git checkout develope

# make your branch
git branch yourbranch

# move your branch
git checkout yourbranch

# revise anything or  add package or someting but don't add large file

git add yourfiles

# commit  your revises
git commit -m "your message"

# push remote store your commits and pull request in the github homepage
git push your branch
```

# Results
# eDriving Kit 제어 시스템

이 프로젝트는 eDriving Kit을 위한 curses 기반 메뉴 제어 시스템입니다.


## Revision History

# 프로젝트 변경 이력

| Rev. 버전 | 주요 변경 사항 |
|-----------|----------------|
| **V1.0.0** | - PCAN-Basic 라이브러리 연동<br>- ROS2 Humble publisher / subscriber, msgs 생성<br>- ROS2 Humble 호환성 확인 |
| **V1.0.1** | - pcan-config.yaml 연동<br>- pcan-node, main_program 구현<br>- PID gain 입력 설계 |
| **V1.1.0** | - pcan-node for VCU 구현<br>- 실행파일 (*.sh) 생성 |
| **V1.1.1** | - README.md 생성 |
| **V1.1.2** | - README.md 수정<br>- 실행파일 (*.sh) 실행 경로 수정<br>- config/ 경로 수정 |
| **V1.2.2** | - 조향각도센서 zero reset 구현<br>- Main_Prog_node.py 수정<br>- KitCmdMsg.msg 수정<br>- ROS2 Foxy CMakeLists 호환성 문제 체크<br>- CMakeLists.txt 수정<br>- edrivingkit_pcan_node 노드명을 for vcu 노드와 통일(yaml 파일) |
| **V1.3.2** | - eDriving.speed CAN tx msg 수정<br>- Main_Prog_node.py에서 SAS reset 변수 초기화<br>- eDriving.deactivate 시 speed=0 처리 |


## 구성 요소

1.0. **edrivingkit_pcan_node** (C++): PCAN 통신을 담당하는 노드
1.1. **edrivingkit_for_vcu_node** (C++): VCU 구매 사용자를 위한 PCAN 통신을 담당하는 노드
# 주의  위의 노드는 하나만 실행할 수 있음

2.0. **Main_Prog_node.py** (Python): curses 기반 사용자 인터페이스를 제공하는 메뉴 제어 프로그램
3.0. **pcan_config.yaml**: PCAN 노드 설정 파일

## 개발자 정보

- 개발: Copyright 2025 ADUS INC., all rights reserved.
- 라이선스: Apache-2.0
- 지원: ROS 2 Foxy
- 플랫폼: Linux (Ubuntu20.04LTS)


## PCAN 라이브러리 설치 방법

1. https://www.peak-system.com/fileadmin/media/linux/index.php 
   peak-system for Linux 홈페이지를 방문하여
   'Download Driver Package' 버튼을 눌러 
   'peak-linux-driver-x.xx.x.tar.gz ' 를 다운로드한다.

```bash
cd ~/Download/
tar -xvf peak-linux-driver-x.xx.x.tar.gz
cd peak-linux-driver-x.xx.x/
sudo apt install libelf-dev libpopt-dev
sudo make clean
sudo make
sudo make install
sudo modprobe pcan
sudo nano /etc/modules
#      pcan               --> modules에 pcan 을 등록한다.
```

2. 디바이스 등록 확인방법
```bash
pcaninfo                  

PCAN driver version: 8.20.0
PCAN-Basic version: 4.10.0.4

  * pcanusbfd32: "PCAN_USBBUS1" (0x051) @ 500 kBit/s, PCAN-USB FD #1, devid=0xFFFFFFFF (/sys/class/pcan/pcanusbfd32)

```


## 빌드 방법

- 빌드 전 선행으로 PCAN Basic 라이브러리를 설치하시오.

```bash
cd ~/Documents/adus_ros2_ws
colcon build --packages-select edrivingkit_pkg
source install/setup.bash
```

## 실행 방법

### 방법:  스크립트 사용

1. **첫 번째 터미널에서 PCAN 노드 실행**:
```bash
cd ~/Documents/adus_ros2_ws/
# VCU를 구매하지 않은 사용자
./start_eDrivingKit_node.sh 
# VCU를 구매한 사용자
./start_eDrivingKit_for_vcu_node.sh
```

2. **두 번째 터미널에서 메인 제어 프로그램 실행**:
```bash
cd ~/Documents/adus_ros2_ws/
./start_main_prog.sh
```


## 메뉴 기능

### 1. Operating eSteering (조향 제어)
#### 1-1. Enable eSteering
- **1. Go to angle of eSteering(degree)**: 조향 각도를 -10° ~ +10° 범위에서 설정
  - 좌측: 음수 값 (예: -20°)
  - 우측: 양수 값 (예: +15°)
- **2. See the information of eSteering value**: 현재 조향 모드와 각도 정보 표시
- **3. Previous menu**: 이전 메뉴로 돌아가기

#### 1-2. See the information of eSteering value
- eSteering mode: 현재 조향 모드 (0: Manual, 1: Auto)
- eSteering angle info: 현재 조향 각도 정보

### 2. Operating eStop (비상정지)
- 비상정지 활성화/해제 기능
- 안전을 위한 즉시 정지 기능
- 1: 비상정지 활성화, 2: 비상정지 해제

### 3. Operating eDrving (메인모터 제어)
- 메인모터 활성화/해제 기능
- 메인모터 RPM 제어 (토크모드)
- 안전을 위한 즉시 정지 기능
- 1: 비상정지 활성화, 2: 비상정지 해제


### 4. Quit program (프로그램 종료)
- 프로그램 안전 종료

## 설정 파일 (pcan_config.yaml)

```yaml
edrivingkit_pcan_node:
  ros__parameters:
    pcan_port: 1                         # PCAN 포트 (1: PCAN_USBBUS1)
    pcan_baud: 500000                    # CAN 통신 속도 (500kbps)
    tx_q_size: 10                        # 송신 큐 크기
    rx_q_size: 20                        # 수신 큐 크기
    eSteering_sensor_dir: 0              # 조향 센서 방향 (0: Right, 1: Opposite)
    eBraking_dir: 0                      # 브레이킹 방향 (0: pulled, 1: pushed)
    eSteering_kp_const: 0.0              # PID Kp 상수 (0: default) ~ 50.0 까지
    eSteering_ki_const: 0.0              # PID Ki 상수 (0: default) ~ 10.0 까지 
    eSteering_kd_const: 0.0              # PID Kd 상수 (0: default) ~ 10.0 까지
    eSteering_min_max_limit_angle: 10.0  # 조향 각도 제한값
```

## PID 상수 수정 방법
- eSteering_kp_const, eSteering_ki_const, eSteering_kd_const 가 0.0일 때 수정하지 않고, default 상황으로 유지
- 수정 예: 
   eSteering_kp_const: 10.0
   eSteering_ki_const: 0.0
   eSteering_kd_const: 5.0

## 조향 각도 제한값 수정 방법
- eSteering_min_max_limit_angle은 사용자 차량의 조향각을 먼저 측정 후 사용하길 권장한다.
- 수정 예:
    eSteering_min_max_limit_angle: 15.0

# 조향센서 장착 위치 따른 수정 값
- 매뉴얼 참조
- 수정 예:
    eSteering_sensor_dir: 1

# 긴급정지 액추에이터 장착 및 stroke Bar에 따른 수정 값
- Stroke bar가 push 된 상태가 default
- 반대 상황이라면 pulled
- 수정 예:
    eBraking_dir: 1


## 메시지 구조

### KitCmdMsg (Command)
```cpp
uint8 cmd_esteering_mode      # 조향 모드 (0: Manual, 1: Auto)
float64 cmd_esteering_angle   # 조향 각도 (도 단위, -10 ~ +10)
uint8 cmd_esteer_kp           # PID Kp 게인
uint8 cmd_esteer_ki           # PID Ki 게인  
uint8 cmd_esteer_kd           # PID Kd 게인
uint8 cmd_edriving_mode       # 주행 모드 (0: Inactive, 1: Active)
float64 cmd_edriving_speed    # 주행 속도 (RPM)
uint8 cmd_estop_braking       # 비상정지 (0: 해제, 1: 활성화)
```

### KitFeedMsg (Feedback)
```cpp
uint8 fd_esteering_mode       # 현재 조향 모드 (0: Manual, 1: Auto)
float64 fd_esteeringle_angle  # 현재 조향 각도 (-100.0 ~ +100.0)
uint8 fd_estop_braking_mode   # 비상정지 모드 (0: release, 1: eStop)
float64 fd_estop_duty         # 비상정지 듀티 (-99.000 ~ 99.000)
uint8 fd_edriving_mode        # 주행 모드 (0: Manual, 1: Auto)
float64 fd_edriving_speed     # 주행 속도 (0 ~ 3000.00 RPM)
uint8 fd_vcu_status           # VCU 상태
uint8 fd_vcu_mode             # VCU 모드
uint8 fd_vcu_alv              # VCU Alive 카운터
```

## Topic 구조

- **edrivingkit_cmd**: 명령 전송 (Main_Prog_node.py → edrivingkit_pcan_node)
- **edrivingkit_feedback**: 피드백 수신 (edrivingkit_pcan_node → Main_Prog_node.py)

## CAN 통신 세부사항

### CAN ID 정의
- **eSTEERING_CMD_ID_**: 0x06800001 (조향 명령)
- **eBRAKING_CMD_ID_**: 0x00000003 (브레이킹 명령) 
- **Setup_PID_ID_**: 0x06900001 (PID 설정)
- **eSTEERING_INFO_STEER_ANGLE_ID_**: 조향 각도 피드백
- **eSTEERING_INFO_MODE_ID_**: 조향 모드 피드백
- **eBRAKING_INFO_DUTY_ID_**: 브레이킹 듀티 피드백

### 통신 주기
- 조향 명령: 20Hz (50ms)
- 브레이킹 명령: 5Hz (200ms)
- 피드백 수신: 실시간

## 주의사항

1. **실행 순서**: PCAN 노드를 먼저 실행한 후 메인 제어 프로그램을 실행하세요.
2. **설정 파일**: pcan_config.yaml 파일을 올바른 경로에서 로드해야 합니다.
3. **PCAN 하드웨어**: PCAN 하드웨어가 올바르게 연결되어 있는지 확인하세요.
4. **터미널 환경**: curses 기반 UI이므로 터미널 환경에서 실행해야 합니다.
5. **안전**: 조향 각도는 -10° ~ +10° 범위에서 먼저 설정하세요.
6. **비상정지**: 안전을 위해 비상정지 기능을 숙지하고 사용하세요.

## 문제 해결

1. **PCAN 초기화 실패**: 
   - PCAN 하드웨어 연결 확인
   - pcan_config.yaml의 pcan_port 설정 확인

2. **파라미터 로드 실패**:
   - 노드 이름이 yaml 파일의 노드명과 일치하는지 확인
   - `--ros-args --params-file` 옵션 사용

3. **Python 스크립트 실행 오류**: 
   - 실행 권한 확인 (`chmod +x`)
   - curses 라이브러리 설치 확인

4. **메시지 통신 오류**: 
   - 두 노드가 모두 실행 중인지 확인
   - `ros2 topic list`로 토픽 확인

5. **컴파일 경고**: 
   - signed/unsigned 비교 경고는 기능상 문제없음
   - 필요시 타입 캐스팅으로 해결

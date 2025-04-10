# Robot Trajectory Simulation

로봇이 목표 지점으로 이동하는 시뮬레이션입니다.
전진(Forward)과 후진(Reverse) 모두 지원하며, 로봇이 목표 위치 및 방향에 정확히 도달할 수 있도록 제어합니다.

## 📂 프로젝트 구조

```
.
├── main.py                 # 여러 대의 로봇을 동시에 시뮬레이션하는 메인 파일
├── single_robot.py         # 단일 로봇만 사용하는 테스트 파일
└── utils
    ├── GoalPlanner.py      # 목표까지의 이동 경로와 속도 계산을 담당
    ├── Robot.py            # 로봇 정보, 이동 업데이트
```

### 1. Robot 클래스 (utils/Robot.py)

    - 로봇의 현재 위치 (x, y) 와 방향 (theta) 을 관리합니다.
    - 주어진 속도(v, w)에 따라 자세(pose)를 업데이트합니다.

### 2. GoalPlanner 클래스 (utils/GoalPlanner.py)

    - 주어진 목표 위치(goal_x, goal_y, goal_theta)로 이동하는 속도(v) 와 회전 속도(w) 를 계산합니다.
    - 전진(Forward) 또는 후진(Reverse) 모드 구현
    - 목표 위치에 도달하면 theta P-Control 수행

### 3. main.py

    - 다수의 로봇을 원형(반지름 5m) 주변에 배치하여 (0, 0) 목표점으로 이동시키는 시뮬레이션을 수행합니다.
    - 각각의 로봇은 개별적인 Robot, GoalPlanner 인스턴스를 사용하여 독립적으로 이동합니다.

### 4. single_robot.py

    - 단일 로봇을 사용하여 기본적인 이동 및 목표 도달 테스트를 수행할 수 있습니다.

## 실행 방법

### 다중 로봇

```
python3 main.py
```

### 단일 로봇

```
python3 single_robot.py
```

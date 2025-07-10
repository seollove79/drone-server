# 드론 에이전트 사용 가이드

## 프로젝트 구조
```
drone-agent/
├── drone_agent.py      # 메인 드론 에이전트 클래스
├── run_agent.py        # 드론 에이전트 실행 스크립트
├── config.py          # 설정 파일
├── test_gcs.py        # 테스트용 GCS 클라이언트
├── requirements.txt   # 필요한 패키지 목록
├── start_agent.bat    # Windows 실행 스크립트
├── start_agent.sh     # Linux/macOS 실행 스크립트
└── README.md         # 사용 가이드
```

## 설치 방법

1. 필요한 패키지 설치:
```bash
pip install -r requirements.txt
```

2. 드론 시뮬레이터 (SITL) 설치 (테스트용):
```bash
pip install MAVProxy
```

## 사용 방법

### 1. 기본 실행
```bash
python run_agent.py
```

### 2. 커스텀 설정으로 실행
```bash
python run_agent.py --name "MyDrone" --connection "udp:127.0.0.1:14550" --server "ws://192.168.1.100:8765"
```

### 3. 명령줄 옵션
- `--name, -n`: 드론 이름 설정
- `--connection, -c`: 드론 연결 문자열
- `--server, -s`: 서버 WebSocket URL
- `--interval, -i`: 상태 업데이트 주기 (초)

### 4. 테스트용 GCS 클라이언트 실행
```bash
python test_gcs.py
```

## 드론 연결 문자열 예시

### SITL (Software In The Loop) 시뮬레이터
```
udp:127.0.0.1:14550
```

### 시리얼 연결
```
COM3                    # Windows
/dev/ttyUSB0           # Linux
/dev/tty.usbserial-*   # macOS
```

### TCP 연결
```
tcp:192.168.1.100:5760
```

### UDP 연결
```
udp:192.168.1.100:14550
```

## SITL 시뮬레이터 실행 방법

1. ArduPilot SITL 다운로드 및 설치:
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

2. SITL 실행:
```bash
cd ArduCopter
sim_vehicle.py --console --map
```

3. 드론 에이전트 실행:
```bash
python run_agent.py
```

## 전체 시스템 테스트 방법

1. 드론 서버 시작:
```bash
cd ../
python drone_server.py
```

2. 드론 에이전트 시작:
```bash
cd drone-agent
python run_agent.py
```

3. GCS 클라이언트 시작:
```bash
python test_gcs.py
```

## 지원하는 드론 명령

- `arm`: 드론 무장
- `disarm`: 드론 무장 해제
- `takeoff`: 이륙 (altitude 매개변수)
- `land`: 착륙
- `goto`: 특정 위치로 이동 (latitude, longitude, altitude 매개변수)
- `set_mode`: 비행 모드 변경 (mode 매개변수)
- `rtl`: Return to Launch (홈으로 복귀)

## 드론 상태 정보

에이전트는 다음과 같은 드론 상태 정보를 수집하고 서버로 전송합니다:

- 연결 상태
- 무장 상태
- 비행 모드
- 배터리 정보 (전압, 전류, 잔량)
- GPS 정보 (위치, 위성 수, 수신 상태)
- 비행 정보 (속도, 고도, 방향)
- 자세 정보 (롤, 피치, 요)

## 문제 해결

### 1. 드론 연결 실패
- 연결 문자열이 올바른지 확인
- 드론/시뮬레이터가 실행 중인지 확인
- 포트가 사용 중이지 않은지 확인

### 2. 서버 연결 실패
- 서버가 실행 중인지 확인
- 서버 URL이 올바른지 확인
- 네트워크 연결 상태 확인

### 3. 패키지 설치 오류
- Python 버전 확인 (3.7 이상 권장)
- pip 업데이트: `pip install --upgrade pip`
- 가상 환경 사용 권장

# 드론 에이전트 설정 파일
# 이 파일을 수정하여 드론 연결 및 서버 설정을 변경할 수 있습니다

# 드론 설정
DRONE_NAME = "Drone_001"                    # 드론 이름
CONNECTION_STRING = "tcp:127.0.0.1:5762"  # 드론 연결 문자열

# 서버 설정
SERVER_URL = "ws://127.0.0.1:8765"         # 서버 WebSocket URL (Docker의 경우 호스트 IP 사용)

# 상태 업데이트 설정
STATUS_UPDATE_INTERVAL = 0.2                # 상태 업데이트 주기 (초)

# 로그 설정
LOG_LEVEL = "INFO"                          # 로그 레벨 (DEBUG, INFO, WARNING, ERROR)

# 연결 문자열 예시:
# - SITL (Software In The Loop): "udp:127.0.0.1:14550"
# - 시리얼 연결: "COM3" (Windows) 또는 "/dev/ttyUSB0" (Linux)
# - TCP 연결: "tcp:192.168.1.100:5760"
# - UDP 연결: "udp:192.168.1.100:14550"

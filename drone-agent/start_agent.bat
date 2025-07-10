@echo off
echo 드론 에이전트 설치 및 실행 스크립트
echo.

echo 1. 필요한 패키지 설치 중...
pip install -r requirements.txt
echo.

echo 2. 드론 에이전트 실행
python run_agent.py

pause

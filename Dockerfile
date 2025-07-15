# Python 3.11 슬림 이미지를 베이스로 사용
FROM python:3.11-slim

# 작업 디렉토리 설정
WORKDIR /app

# 시스템 패키지 업데이트 및 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# requirements.txt 복사 및 의존성 설치
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 애플리케이션 코드 복사
COPY . .

# 포트 8765 노출
EXPOSE 8765

# Python 출력 버퍼링 비활성화
ENV PYTHONUNBUFFERED=1

# 서버 시작 명령
CMD ["python", "-u", "drone_server.py"]
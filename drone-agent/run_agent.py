#!/usr/bin/env python3
"""
드론 에이전트 실행 스크립트
"""

import asyncio
import sys
import argparse
from drone_agent import DroneAgent
from config import *

def parse_arguments():
    """
    명령줄 인수를 파싱하는 함수
    """
    parser = argparse.ArgumentParser(description='드론 에이전트 실행')
    
    parser.add_argument('--name', '-n', 
                       default=DRONE_NAME,
                       help='드론 이름')
    
    parser.add_argument('--connection', '-c',
                       default=CONNECTION_STRING,
                       help='드론 연결 문자열')
    
    parser.add_argument('--server', '-s',
                       default=SERVER_URL,
                       help='서버 WebSocket URL')
    
    parser.add_argument('--interval', '-i',
                       type=float,
                       default=STATUS_UPDATE_INTERVAL,
                       help='상태 업데이트 주기 (초)')
    
    return parser.parse_args()

async def main():
    """
    메인 함수
    """
    # 명령줄 인수 파싱
    args = parse_arguments()
    
    print(f"드론 에이전트 시작")
    print(f"드론 이름: {args.name}")
    print(f"연결 문자열: {args.connection}")
    print(f"서버 URL: {args.server}")
    print(f"업데이트 주기: {args.interval}초")
    print("-" * 50)
    
    # 드론 에이전트 생성
    agent = DroneAgent(
        drone_name=args.name,
        connection_string=args.connection,
        server_url=args.server
    )
    
    # 상태 업데이트 주기 설정
    agent.status_update_interval = args.interval
    
    try:
        # 드론 에이전트 시작
        await agent.start()
    except KeyboardInterrupt:
        print("\n사용자에 의해 종료됨")
    except Exception as e:
        print(f"오류 발생: {e}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())

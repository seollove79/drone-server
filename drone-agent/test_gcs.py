# 간단한 GCS(Ground Control Station) 클라이언트
import asyncio
import websockets
import json
import logging
from datetime import datetime

# 로깅 설정 - 웹소켓 로그 레벨 조정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# WebSocket 로그 레벨을 WARNING으로 설정하여 불필요한 디버그 메시지 방지
logging.getLogger('websockets.client').setLevel(logging.WARNING)

class SimpleGCS:
    """
    간단한 GCS 클라이언트 클래스
    
    드론 서버에 연결하여 드론 상태를 모니터링하고 명령을 전송할 수 있습니다.
    """
    
    def __init__(self, gcs_name: str = "GCS_Test", server_url: str = "ws://127.0.0.1:8765"):
        self.gcs_name = gcs_name
        self.server_url = server_url
        self.websocket = None
        self.connected_drones = []
        self.running = True
        
    async def connect_to_server(self):
        """서버에 연결 - 개선된 버전"""
        max_retries = 3
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                logger.info(f"서버 연결 시도 {attempt + 1}/{max_retries}: {self.server_url}")
                
                # 연결 타임아웃 설정 (ping 타임아웃 증가)
                self.websocket = await asyncio.wait_for(
                    websockets.connect(self.server_url, ping_interval=30, ping_timeout=20),
                    timeout=10.0
                )
                
                # GCS로 등록
                registration_data = {
                    "role": "gcs",
                    "name": self.gcs_name
                }
                
                await self.websocket.send(json.dumps(registration_data))
                
                # 등록 확인 메시지 수신
                registration_confirmed = False
                timeout_count = 0
                max_timeout = 10
                
                while not registration_confirmed and timeout_count < max_timeout:
                    try:
                        response = await asyncio.wait_for(self.websocket.recv(), timeout=2.0)
                        response_data = json.loads(response)
                        
                        if response_data.get("type") == "registration_success":
                            logger.info(f"서버 등록 성공: {response_data.get('message')}")
                            registration_confirmed = True
                        elif response_data.get("type") == "client_list":
                            self.connected_drones = response_data.get("drones", [])
                            logger.info(f"클라이언트 목록 업데이트: 드론 {len(self.connected_drones)}개")
                        else:
                            logger.debug(f"기타 메시지 수신: {response_data.get('type')}")
                            
                    except asyncio.TimeoutError:
                        timeout_count += 1
                        logger.debug(f"등록 확인 대기 중... ({timeout_count}/{max_timeout})")
                        
                if registration_confirmed:
                    logger.info("서버 연결 및 등록 완료")
                    return True
                else:
                    logger.warning("등록 확인 실패, 재시도합니다.")
                    await self.websocket.close()
                    self.websocket = None
                    
            except asyncio.TimeoutError:
                logger.error(f"연결 시간 초과 (시도 {attempt + 1}/{max_retries})")
            except Exception as e:
                logger.error(f"서버 연결 실패 (시도 {attempt + 1}/{max_retries}): {e}")
                
            if self.websocket:
                try:
                    await self.websocket.close()
                except:
                    pass
                self.websocket = None
                
            if attempt < max_retries - 1:
                logger.info(f"{retry_delay}초 후 재시도합니다...")
                await asyncio.sleep(retry_delay)
                
        logger.error("모든 연결 시도가 실패했습니다.")
        return False
            
    async def send_command(self, target_drone: str, command: str, parameters: dict = None):
        """드론에 명령 전송 - 개선된 버전"""
        if not self.websocket:
            logger.error("서버에 연결되지 않았습니다.")
            return False
            
        try:
            # 연결 상태 확인
            await self.websocket.ping()
            
            command_message = {
                "type": "command",
                "target_drone": target_drone,
                "command": command,
                "parameters": parameters or {}
            }
            
            logger.info(f"명령 전송 시도: {target_drone} -> {command}")
            await self.websocket.send(json.dumps(command_message))
            logger.info(f"명령 전송 완료: {target_drone} -> {command}")
            return True
            
        except websockets.exceptions.ConnectionClosed:
            logger.error("명령 전송 중 연결이 끊어졌습니다.")
            self.running = False
            return False
        except Exception as e:
            logger.error(f"명령 전송 실패: {e}")
            return False
            
    async def handle_message(self, message: str):
        """서버로부터 받은 메시지 처리 - 중복 상태 로그 방지"""
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "client_list":
                # 클라이언트 목록 업데이트
                self.connected_drones = data.get("drones", [])
                logger.info(f"연결된 드론 목록: {self.connected_drones}")
                
            elif msg_type == "drone_status":
                # 드론 상태 업데이트 (로그 레벨을 DEBUG로 변경하여 스팸 방지)
                drone_name = data.get("drone")
                battery = data.get("battery", 0)
                position = data.get("position", {})
                status = data.get("status", {})
                
                # 배터리 전압 정보 추가
                battery_voltage = status.get('battery_voltage', 0.0)
                
                # 주기적으로만 상태 출력 (매 10번째 메시지마다)
                if not hasattr(self, '_status_counter'):
                    self._status_counter = 0
                    self._last_status = {}
                
                self._status_counter += 1
                
                # 상태가 변경되었거나 10번째 메시지일 때만 출력
                current_status = f"{drone_name}:{status.get('mode', 'UNKNOWN')}:{battery}%"
                if (self._status_counter % 10 == 0 or 
                    self._last_status.get(drone_name) != current_status):
                    
                    logger.info(f"드론 상태 - {drone_name}: 배터리 {battery}% ({battery_voltage:.2f}V), "
                               f"위치 ({position.get('latitude', 0):.6f}, {position.get('longitude', 0):.6f}), "
                               f"모드 {status.get('mode', 'UNKNOWN')}")
                    
                    self._last_status[drone_name] = current_status
                
            elif msg_type == "chat":
                # 채팅 메시지
                sender = data.get("sender")
                message_content = data.get("message")
                logger.info(f"채팅 - {sender}: {message_content}")
                
        except json.JSONDecodeError:
            logger.error(f"잘못된 JSON 메시지: {message}")
        except Exception as e:
            logger.error(f"메시지 처리 오류: {e}")
            
    async def message_receive_loop(self):
        """메시지 수신 루프 - 개선된 버전"""
        consecutive_errors = 0
        max_consecutive_errors = 3
        
        while self.running:
            try:
                if not self.websocket:
                    logger.error("WebSocket 연결이 없습니다.")
                    break
                
                # 메시지 수신 (더 짧은 타임아웃으로 빠른 처리)
                message = await asyncio.wait_for(self.websocket.recv(), timeout=2.0)
                
                # 메시지 처리를 비동기로 실행하여 블로킹 방지
                asyncio.create_task(self.handle_message(message))
                
                # 성공적으로 메시지를 받았으면 에러 카운터 리셋
                consecutive_errors = 0
                    
            except asyncio.TimeoutError:
                # 타임아웃은 정상적인 상황 - 연결 상태 확인은 별도 루프에서 처리
                continue
                    
            except websockets.exceptions.ConnectionClosed:
                logger.warning("서버 연결이 끊어졌습니다.")
                self.running = False
                break
                
            except Exception as e:
                logger.error(f"메시지 수신 오류: {e}")
                consecutive_errors += 1
                
                # 연속 에러가 많이 발생하면 연결 종료
                if consecutive_errors >= max_consecutive_errors:
                    logger.error(f"연속 {max_consecutive_errors}번의 오류로 연결을 종료합니다.")
                    self.running = False
                    break
                    
                # 짧은 대기 후 재시도
                await asyncio.sleep(0.5)
                
    async def command_input_loop(self):
        """명령 입력 루프"""
        await asyncio.sleep(2)  # 초기 연결 대기
        
        print("\n=== 간단한 GCS 클라이언트 ===")
        print("사용 가능한 명령:")
        print("  arm <drone_name>                     - 드론 무장")
        print("  disarm <drone_name>                  - 드론 무장 해제")
        print("  takeoff <drone_name> [altitude]      - 이륙 (자동 무장 + 모드 설정)")
        print("  land <drone_name>                    - 착륙")
        print("  emergency <drone_name>               - 비상 착륙 (RTL 모드)")
        print("  goto <drone_name> <lat> <lon> [alt]  - 특정 위치로 이동")
        print("  mode <drone_name> <mode>             - 비행 모드 변경")
        print("  rtl <drone_name>                     - Return to Launch")
        print("  list                                 - 연결된 드론 목록")
        print("  status                               - 드론 상태 확인")
        print("  quit                                 - 종료")
        print("-" * 50)
        
        while self.running:
            try:
                # 연결 상태 확인 (버전 호환성 고려)
                websocket_open = False
                if self.websocket:
                    if hasattr(self.websocket, 'closed'):
                        websocket_open = not self.websocket.closed
                    elif hasattr(self.websocket, 'state'):
                        websocket_open = self.websocket.state.name == 'OPEN'
                    else:
                        websocket_open = True  # 상태 확인 불가능한 경우 연결된 것으로 가정
                
                if not websocket_open:
                    print("서버 연결이 끊어졌습니다. 프로그램을 종료합니다.")
                    self.running = False
                    break
                
                command = input("명령 입력: ").strip()
                
                if not command:
                    continue
                    
                parts = command.split()
                
                if parts[0] == "quit":
                    self.running = False
                    break
                    
                elif parts[0] == "list":
                    print(f"연결된 드론: {self.connected_drones}")
                    
                elif parts[0] == "status":
                    print("=== 드론 상태 정보 ===")
                    if self.connected_drones:
                        for drone in self.connected_drones:
                            print(f"드론: {drone} - 연결됨")
                    else:
                        print("연결된 드론이 없습니다.")
                    
                elif parts[0] == "arm" and len(parts) >= 2:
                    await self.send_command(parts[1], "arm")
                    
                elif parts[0] == "disarm" and len(parts) >= 2:
                    await self.send_command(parts[1], "disarm")

                elif parts[0] == "mode" and len(parts) >= 2:
                    drone_name = parts[1]
                    if len(parts) > 2 :
                        mode = str(parts[2])
                    else :
                        mode = "GUIDE"
                    print(f"비행모드변경중... 드론: {drone_name}, 모드: {mode}")
                    mode_success = await self.send_command(drone_name, "set_mode", {"mode": mode})
                    if not mode_success:
                        print("비행모드 변경 실패")
                    else:
                        print("비행모드 변경 전송 완료!")

                elif parts[0] == "arm&takeoff" and len(parts) >= 2:
                    altitude = float(parts[2]) if len(parts) > 2 else 10.0
                    drone_name = parts[1]
                    print(f"시동&이륙 준비 중... 드론: {drone_name}, 고도: {altitude}m")
                    takeoff_success = await self.send_command(drone_name, "arm&takeoff", {"altitude": altitude})
                    if not takeoff_success:
                        print("이륙 명령 전송 실패")
                    else:
                        print("이륙 명령 전송 완료!")
                    
                elif parts[0] == "takeoff" and len(parts) >= 2:
                    altitude = float(parts[2]) if len(parts) > 2 else 10.0
                    drone_name = parts[1]
                    
                    print(f"이륙 준비 중... 드론: {drone_name}, 고도: {altitude}m")
                    takeoff_success = await self.send_command(drone_name, "takeoff", {"altitude": altitude})
                    if not takeoff_success:
                        print("이륙 명령 전송 실패")
                    else:
                        print("이륙 명령 전송 완료!")
                    
                elif parts[0] == "land" and len(parts) >= 2:
                    success = await self.send_command(parts[1], "land")
                    if not success:
                        print("명령 전송 실패")
                        
                elif parts[0] == "emergency" and len(parts) >= 2:
                    # 비상 착륙 (무장 해제 + 착륙)
                    drone_name = parts[1]
                    print(f"비상 착륙 실행 중... 드론: {drone_name}")
                    
                    # RTL 모드로 변경하여 자동 복귀
                    await self.send_command(drone_name, "rtl")
                    print("RTL 모드로 변경 완료")
                    
                else:
                    print("잘못된 명령입니다. 'quit'를 입력하여 종료하거나 올바른 명령을 입력하세요.")
                    
                # 명령 전송 후 잠시 대기하여 서버 응답 처리 시간 확보
                await asyncio.sleep(0.1)
                    
            except KeyboardInterrupt:
                self.running = False
                break
            except Exception as e:
                logger.error(f"명령 입력 오류: {e}")
                
    async def start(self):
        """GCS 클라이언트 시작"""
        if not await self.connect_to_server():
            logger.error("서버 연결 실패")
            return
            
        logger.info("GCS 클라이언트가 성공적으로 시작되었습니다.")
        
        # 비동기 태스크 시작
        tasks = [
            asyncio.create_task(self.message_receive_loop()),
            asyncio.create_task(self.command_input_loop()),
            asyncio.create_task(self.connection_monitor())
        ]
        
        try:
            await asyncio.gather(*tasks, return_exceptions=True)
        except KeyboardInterrupt:
            logger.info("사용자 종료 요청")
        except Exception as e:
            logger.error(f"GCS 실행 중 오류: {e}")
        finally:
            logger.info("GCS 클라이언트 종료 중...")
            self.running = False
            if self.websocket:
                try:
                    # WebSocket 연결 상태 확인 후 종료
                    websocket_open = False
                    if hasattr(self.websocket, 'closed'):
                        websocket_open = not self.websocket.closed
                    elif hasattr(self.websocket, 'state'):
                        websocket_open = self.websocket.state.name == 'OPEN'
                    else:
                        websocket_open = True
                    
                    if websocket_open:
                        await self.websocket.close()
                        logger.info("WebSocket 연결 정상 종료")
                    else:
                        logger.info("WebSocket 연결이 이미 닫혀있음")
                except Exception as e:
                    logger.error(f"WebSocket 종료 중 오류: {e}")

    async def check_connection_status(self):
        """연결 상태를 확인하는 메서드"""
        if not self.websocket:
            return False
        
        # WebSocket 연결 상태 확인 (버전 호환성 고려)
        try:
            if hasattr(self.websocket, 'closed'):
                if self.websocket.closed:
                    logger.warning("WebSocket 연결이 닫혀있습니다.")
                    return False
            elif hasattr(self.websocket, 'state'):
                if self.websocket.state.name != 'OPEN':
                    logger.warning(f"WebSocket 연결 상태: {self.websocket.state.name}")
                    return False
            
            # 간단한 ping을 통해 연결 상태 확인
            await self.websocket.ping()
            return True
        except Exception as e:
            logger.warning(f"연결 상태 확인 실패: {e}")
            return False

    async def connection_monitor(self):
        """연결 상태 모니터링 루프 - 개선된 버전"""
        while self.running:
            try:
                if self.websocket:
                    # 주기적으로 ping을 보내서 연결 상태 확인 (더 긴 간격)
                    await self.websocket.ping()
                    logger.debug("연결 상태 확인 완료")
                else:
                    logger.warning("WebSocket 연결이 없습니다.")
                    self.running = False
                    break
                    
                # 60초마다 연결 상태 확인 (부하 감소)
                await asyncio.sleep(60)
                
            except websockets.exceptions.ConnectionClosed:
                logger.error("연결 모니터링 중 연결이 끊어졌습니다.")
                self.running = False
                break
            except Exception as e:
                logger.error(f"연결 모니터링 오류: {e}")
                # 연결 모니터링 오류는 즉시 종료하지 않고 재시도
                await asyncio.sleep(5)

# 메인 실행 부분
async def main():
    gcs = SimpleGCS()
    await gcs.start()

if __name__ == "__main__":
    asyncio.run(main())

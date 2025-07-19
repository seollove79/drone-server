# 드론 에이전트를 위한 필수 모듈들을 import
import asyncio          # 비동기 프로그래밍을 위한 asyncio
import websockets       # WebSocket 클라이언트 통신
import json            # JSON 데이터 처리
import logging         # 로깅 기능
from typing import Dict, Any
from pymavlink import mavutil  # MAVLink 통신을 위한 모듈

# 로깅 설정 (INFO 레벨로 변경)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DroneAgent:
    """
    드론과 MAVLink로 통신하고 서버와 WebSocket으로 연결하는 드론 에이전트 클래스
    
    이 클래스는 다음과 같은 기능을 제공합니다:
    - MAVLink를 통한 드론 연결 및 제어
    - 드론 상태 모니터링 및 데이터 수집
    - 서버와의 WebSocket 통신
    - 명령 수신 및 드론 제어
    """
    
    def __init__(self, drone_name: str, connection_string: str, server_url: str = "ws://127.0.0.1:8765"):
        """
        드론 에이전트 초기화
        
        Args:
            drone_name: 드론 이름
            connection_string: 드론 연결 문자열 (예: 'udp:127.0.0.1:14550')
            server_url: 서버 WebSocket URL
        """
        self.drone_name = drone_name
        self.connection_string = connection_string
        self.server_url = server_url
        
        # MAVLink 연결 객체
        self.drone_connection = None
        
        # WebSocket 연결 객체
        self.websocket = None
        
            # 드론 상태 정보
        self.drone_status = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "battery_voltage": 0.0,
            "battery_current": 0.0,
            "battery_remaining": 0,
            "gps_fix": 0,
            "satellites_visible": 0,
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0,           # 해수면 고도 (absolute altitude)
            "altitude_relative": 0.0,  # 상대 고도 (relative altitude)
            "groundspeed": 0.0,
            "airspeed": 0.0,
            "heading": 0.0,
            "throttle": 0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
        }
        
        # 상태 업데이트 주기 (초)
        self.status_update_interval = 1.0
        
        # 실행 중 플래그
        self.running = True
        
        # 메시지 필터링을 위한 타임스탬프
        self.last_message_times = {}
        self.message_rate_limits = {
            'GLOBAL_POSITION_INT': 0.1,  # 100ms 간격
            'ATTITUDE': 0.1,             # 100ms 간격
            'VFR_HUD': 0.2,              # 200ms 간격
            'SYS_STATUS': 1.0,           # 1초 간격
            'GPS_RAW_INT': 1.0           # 1초 간격
        }
        
    async def connect_to_drone(self):
        """
        드론에 MAVLink로 연결하는 메서드
        """
        try:
            logger.info(f"드론 연결 시도: {self.connection_string}")
            
            # MAVLink 연결 생성
            self.drone_connection = mavutil.mavlink_connection(self.connection_string)
            
            # 하트비트 대기 (연결 확인)
            logger.info("드론 하트비트 대기 중...")
            self.drone_connection.wait_heartbeat()
            
            logger.info(f"드론 연결 성공! 시스템 ID: {self.drone_connection.target_system}")
            self.drone_status["connected"] = True
            
            return True
            
        except Exception as e:
            logger.error(f"드론 연결 실패: {e}")
            self.drone_status["connected"] = False
            return False
            
    async def connect_to_server(self):
        """
        서버에 WebSocket으로 연결하는 메서드
        """
        try:
            logger.info(f"서버 연결 시도: {self.server_url}")
            
            # WebSocket 연결 생성
            self.websocket = await websockets.connect(self.server_url)
            
            # 드론으로 등록
            registration_data = {
                "role": "drone",
                "name": self.drone_name
            }
            
            await self.websocket.send(json.dumps(registration_data))
            
            # 등록 확인 메시지 수신 (여러 메시지 처리)
            registration_success = False
            max_attempts = 5  # 최대 5개 메시지까지 확인
            
            for attempt in range(max_attempts):
                try:
                    response = await asyncio.wait_for(self.websocket.recv(), timeout=5.0)
                    response_data = json.loads(response)
                    
                    if response_data.get("type") == "registration_success":
                        logger.info(f"서버 등록 성공: {response_data.get('message')}")
                        registration_success = True
                        break
                    else:
                        logger.debug(f"다른 메시지 수신 (시도 {attempt + 1}): {response_data.get('type')}")
                        # 클라이언트 목록이나 다른 메시지는 무시하고 계속 진행
                        
                except asyncio.TimeoutError:
                    logger.error("등록 확인 메시지 수신 시간 초과")
                    break
                except json.JSONDecodeError:
                    logger.error(f"잘못된 JSON 응답: {response}")
                    continue
                    
            if registration_success:
                return True
            else:
                logger.error("서버 등록 실패: 등록 확인 메시지를 받지 못했습니다.")
                return False
                
        except Exception as e:
            logger.error(f"서버 연결 실패: {e}")
            return False
        
    async def set_message_interval(self, message_id, interval):
        """
        특정 MAVLink 메시지의 전송 간격을 설정하는 메서드
        
        Args:
            message_id: 메시지 ID (예: mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT)
            interval: 전송 간격 (초 단위)
        """
        if not self.drone_connection:
            logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.drone_connection.mav.command_long_send(
                self.drone_connection.target_system,
                self.drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # confirmation
                message_id,  # 메시지 ID
                interval,  # 간격 (마이크로초 단위)
                0, 0, 0, 0, 0
            )
        except Exception as e:
            logger.error(f"메시지 간격 설정 오류: {e}")
            return False
            
    def update_drone_status(self):
        """
        드론으로부터 상태 정보를 수집하는 메서드 (최적화됨)
        """
        if not self.drone_connection:
            return
        
        try:
            # 큐에 있는 모든 메시지를 처리 (논블로킹)
            message_count = 0
            max_messages_per_cycle = 50  # 한 번에 처리할 최대 메시지 수
            
            while message_count < max_messages_per_cycle:
                msg = self.drone_connection.recv_match(blocking=False)
                if not msg:
                    break
                    
                message_count += 1
                msg_type = msg.get_type()
                
                # 레이트 리미팅 적용
                if not self.should_process_message(msg_type):
                    continue
                
                # 중요한 메시지 타입만 로깅 (디버깅 시에만)
                if logger.level == logging.DEBUG:
                    if msg_type in ['HEARTBEAT', 'SYS_STATUS', 'GLOBAL_POSITION_INT']:
                        logger.debug(f"메시지 수신: {msg_type}")

                # 메시지 타입별 처리 (성능을 위해 if-elif 순서 최적화)
                if msg_type == 'GLOBAL_POSITION_INT':
                    # 가장 자주 오는 메시지 먼저 처리
                    self.drone_status["latitude"] = msg.lat / 1e7
                    self.drone_status["longitude"] = msg.lon / 1e7
                    self.drone_status["altitude"] = msg.alt / 1000.0
                    self.drone_status["altitude_relative"] = msg.relative_alt / 1000.0
                    
                elif msg_type == 'HEARTBEAT':
                    if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                        self.drone_status["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        
                        mode_name = self.get_flight_mode_name(msg)
                        if self.drone_status["mode"] != mode_name:
                            logger.info(f"비행 모드 변경: {self.drone_status['mode']} -> {mode_name}")
                        self.drone_status["mode"] = mode_name
                        
                elif msg_type == 'VFR_HUD':
                    self.drone_status["groundspeed"] = msg.groundspeed
                    self.drone_status["airspeed"] = msg.airspeed
                    self.drone_status["heading"] = msg.heading
                    self.drone_status["throttle"] = msg.throttle
                    
                elif msg_type == 'ATTITUDE':
                    self.drone_status["roll"] = msg.roll
                    self.drone_status["pitch"] = msg.pitch
                    self.drone_status["yaw"] = msg.yaw
                    
                elif msg_type == 'SYS_STATUS':
                    self.drone_status["battery_voltage"] = msg.voltage_battery / 1000.0
                    self.drone_status["battery_current"] = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                    self.drone_status["battery_remaining"] = msg.battery_remaining
                    
                elif msg_type == 'GPS_RAW_INT':
                    self.drone_status["gps_fix"] = msg.fix_type
                    self.drone_status["satellites_visible"] = msg.satellites_visible
                    
                elif msg_type == 'STATUSTEXT':
                    # 중요한 상태 텍스트만 로깅
                    if msg.severity <= 4:  # Emergency, Alert, Critical, Error만
                        logger.warning(f"드론 메시지 (심각도 {msg.severity}): {msg.text}")
                        
            if message_count >= max_messages_per_cycle:
                logger.debug(f"메시지 처리 제한 도달: {message_count}개 처리됨")
                
        except Exception as e:
            logger.error(f"드론 상태 업데이트 오류: {e}")

    async def send_status_to_server(self):
        """
        드론 상태를 서버로 전송하는 메서드
        """
        if not self.websocket:
            return
            
        try:
            # 상태 메시지 생성
            status_message = {
                "type": "status",
                "battery": self.drone_status["battery_remaining"],
                "position": {
                    "latitude": self.drone_status["latitude"],
                    "longitude": self.drone_status["longitude"],
                    "altitude": self.drone_status["altitude"],
                    "altitude_relative": self.drone_status["altitude_relative"]
                },
                "status": {
                    "connected": self.drone_status["connected"],
                    "armed": self.drone_status["armed"],
                    "mode": self.drone_status["mode"],
                    "gps_fix": self.drone_status["gps_fix"],
                    "satellites": self.drone_status["satellites_visible"],
                    "airspeed": self.drone_status["airspeed"],
                    "groundspeed": self.drone_status["groundspeed"],
                    "heading": self.drone_status["heading"],
                    "battery_voltage": self.drone_status["battery_voltage"],
                    "roll": self.drone_status["roll"],
                    "pitch": self.drone_status["pitch"],
                    "yaw": self.drone_status["yaw"]
                }
            }
            
            # 서버로 상태 전송
            await self.websocket.send(json.dumps(status_message))
            
        except Exception as e:
            logger.error(f"상태 전송 오류: {e}")
            
    async def handle_server_message(self, message: str):
        """
        서버로부터 받은 메시지를 처리하는 메서드
        
        Args:
            message: 서버로부터 받은 메시지
        """
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            logger.info(f"서버 메시지 수신 - 타입: {msg_type}")
            
            if msg_type == "command":
                command = data.get("command")
                parameters = data.get("parameters", {})
                
                logger.info(f"명령 수신: {command}, 매개변수: {parameters}")
                
                # 명령 처리
                await self.execute_command(command, parameters)
                
            elif msg_type == "client_list":
                # 클라이언트 목록 업데이트 (필요시 처리)
                drones = data.get("drones", [])
                gcs_stations = data.get("gcs_stations", [])
                logger.debug(f"클라이언트 목록 업데이트 - 드론: {drones}, GCS: {gcs_stations}")
                
        except json.JSONDecodeError:
            logger.error(f"잘못된 JSON 메시지: {message}")
        except Exception as e:
            logger.error(f"메시지 처리 오류: {e}")
            logger.error(f"메시지 내용: {message}")

    async def execute_command_long(self, cmd_code, param1,param2,param3,param4,param5,param6,param7):
        """
        드론 명령을 실행하는 메서드
        
        Args:
            cmd_code: 명령 코드 (예: MAV_CMD_NAV_TAKEOFF)
            param1, param2, param3, param4, param5, param6, param7: 명령 매개변수
        """
        if not self.drone_connection:
            logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            # 명령 전송
            self.drone_connection.mav.command_long_send(
                self.drone_connection.target_system,
                self.drone_connection.target_component,
                cmd_code,
                0,  # confirmation
                param1, param2, param3, param4, param5, param6, param7
            )
            
            # 명령 응답 확인
            result_msg = self.drone_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if result_msg:
                if result_msg.command == cmd_code:
                    if result_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        logger.info(f"명령 수락됨: {cmd_code}")
                        return True
                    else:
                        # 결과 코드에 따른 상세 메시지
                        result_messages = {
                            mavutil.mavlink.MAV_RESULT_DENIED: "명령 거부됨",
                            mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "지원되지 않는 명령",
                            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "일시적으로 거부됨",
                            mavutil.mavlink.MAV_RESULT_FAILED: "명령 실행 실패",
                            mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "명령 실행 중"
                        }
                        error_msg = result_messages.get(result_msg.result, f"알 수 없는 결과: {result_msg.result}")
                        logger.error(f"명령 실패 ({cmd_code}): {error_msg}")
                        return False
                else:
                    logger.error(f"다른 명령의 응답 수신: {result_msg.command} (예상: {cmd_code})")
                    return False
            else:
                logger.error(f"명령 응답 시간 초과: {cmd_code}")
                return False
            
        except Exception as e:
            logger.error(f"명령 실행 오류: {e}")
            return False

            
    async def execute_command(self, command: str, parameters: Dict[str, Any] = {}):
        """
        드론 명령을 실행하는 메서드
        
        Args:
            command: 실행할 명령
        """

        logger.info(f"명령 실행 요청: {command}, 매개변수: {parameters}")

        try:
            if command == "arm":
                logger.info("드론 시동 명령 실행 중...")
                if self.drone_status["armed"]:
                    logger.warning("이미 시동이 걸려 있습니다.")
                    return
                
                arm_flag = 1 # (1:arm, 0:disarm)
                arm_force = 0 # (0: 안전체크, 1: 안전체크없이 강제)
                result = await self.execute_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,arm_flag,arm_force,0,0,0,0,0)
                if result:
                    logger.info("시동 성공")
                else:
                    logger.error("시동 실패")
                    return
                
            elif command == "disarm":
                logger.info("드론 시동 끄기 명령 실행 중...")
                if not self.drone_status["armed"]:
                    logger.warning("드론이 이미 시동이 꺼져 있습니다.")
                    return
                
                arm_flag = 0 # (1:arm, 0:disarm)
                arm_force = 0 # (0: 안전체크, 1: 안전체크 없이 강제)
                result = await self.execute_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,arm_flag,arm_force,0,0,0,0,0)
                if result:
                    logger.info("시동 끄기 성공")
                else:
                    logger.error("시동 끄기 실패")
                    return
                
            elif command == "arm&takeoff":
                logger.info("드론 시동&이륙 명령 실행 중...")
                
                self.drone_connection.mav.set_mode_send(
                    self.drone_connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4  # GUIDED 모드로 설정 (4는 GUIDED 모드의 ID)
                )
               
                if not self.drone_status["armed"]:
                    arm_flag = 1 # (1:arm, 0:disarm)
                    arm_force = 0 # (0: 안전체크, 1: 안전체크없이 강제)
                    result = await self.execute_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,arm_flag,arm_force,0,0,0,0,0)
                    if not result:
                        logger.error("시동 실패")
                        return
                
                # 이륙명령
                if "altitude" in parameters:
                    altitude = parameters["altitude"]
                else:
                    altitude = 5  # 기본 이륙 고도 5m
                result = await self.execute_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,  # param1: pitch
                    0,  # param2: empty 
                    0,  # param3: empty
                    0,  # param4: yaw
                    0,  # param5: latitude
                    0,  # param6: longitude
                    altitude  # param7: altitude
                )

                if result:
                    logger.info(f"이륙 명령 성공: {altitude}m")
                else:
                    logger.error("이륙 명령 실패")
                    return
                
            elif command == "takeoff":
                logger.info("test : 드론 이륙 명령 실행 중...")
                if not self.drone_status["armed"]:
                    logger.error("드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")
                    return
                if "altitude" in parameters:
                    altitude = parameters["altitude"]
                else:
                    altitude = 5
                result = await self.execute_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,  # param1: pitch
                    0,  # param2: empty 
                    0,  # param3: empty
                    0,  # param4: yaw
                    0,  # param5: latitude
                    0,  # param6: longitude
                    altitude  # param7: altitude
                )
                if result:
                    logger.info(f"이륙 명령 성공: {altitude}m")
                else:
                    logger.error("이륙 명령 실패")
                    return
                
            elif command == "land":
                # 착륙 명령
                logger.info("드론 착륙 명령 실행 중...")
                if not self.drone_status["armed"]:
                    logger.error("시동되지 않은 상태입니다.")
                    return
                
                result = await self.execute_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
                if result:
                    logger.info("착륙 명령 성공")
                else:
                    logger.error("착륙 명령 실패")
                    return
                
            elif command == "goto":
                # 특정 위치로 이동
                lat = parameters.get("latitude", 0)
                lon = parameters.get("longitude", 0)

                # 고도가 없으면 현재고도로 설정
                if "altitude" in parameters:
                    alt = parameters["altitude"]
                else:
                    alt = self.drone_status["altitude"]
                    # 현재 고도가 0이면 기본값 설정
                    if alt <= 0:
                        alt = 10  # 기본 고도 10m
                
                if lat != 0 and lon != 0:
                    logger.info(f"이동 명령 실행 중... 위치: ({lat}, {lon}, {alt})")
                    if not self.drone_status["armed"]:
                        logger.error("드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")
                        return
                    
                    # 새로운 이동 메서드 사용
                    success = await self.goto_position(lat, lon, alt)
                    if success:
                        logger.info(f"이동 명령 성공: ({lat}, {lon}, {alt})")
                    else:
                        logger.error("이동 명령 실패")
                        return
                else:
                    logger.error("이동 명령 오류: 위도와 경도가 필요합니다.")
                
                    
            elif command == "set_mode":
                # 비행 모드 변경
                mode = parameters.get("mode", "GUIDED")
                logger.info(f"모드 변경 명령 실행 중... 모드: {mode}")
                mode_id = self.drone_connection.mode_mapping().get(mode.upper())
                
                if mode_id is not None:
                    self.drone_connection.mav.set_mode_send(
                        self.drone_connection.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )
                    logger.info(f"모드 변경 명령 전송 완료: {mode}")
                else:
                    logger.error(f"알 수 없는 모드: {mode}")

            elif command == "rtl":
                # Return to Launch (홈으로 복귀)
                logger.info("RTL 명령 실행 중...")
                if not self.drone_status["armed"]:
                    logger.error("드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")
                    return
                
                result = await self.execute_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0, 0, 0, 0, 0, 0, 0
                )
                
                if result:
                    logger.info("RTL 명령 전송 완료")
                else:
                    logger.error("RTL 명령 실패")
                    return
                                    
            # elif command == "rtl":
            #     # Return to Launch (홈으로 복귀)
            #     logger.info("RTL 명령 실행 중...")
            #     self.drone_connection.mav.command_long_send(
            #         self.drone_connection.target_system,
            #         self.drone_connection.target_component,
            #         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            #         0, 0, 0, 0, 0, 0, 0, 0
            #     )
            #     logger.info("RTL 명령 전송 완료")
            #     # 명령 응답 확인
            #     await self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, timeout=5)
                
            else:
                logger.warning(f"알 수 없는 명령: {command}")
                
        except Exception as e:
            logger.error(f"명령 실행 오류: {e}")
            
        # 명령 실행 후 상태 확인
        await asyncio.sleep(1)  # 상태 업데이트 대기
        # logger.info(f"명령 실행 후 드론 상태 - 연결: {self.drone_status['connected']}, "
        #            f"무장: {self.drone_status['armed']}, 모드: {self.drone_status['mode']}")
    
      
    async def status_update_loop(self):
        """
        주기적으로 드론 상태를 업데이트하고 서버로 전송하는 루프 (최적화됨)
        """
        last_status_send = 0
        status_send_interval = 1.0  # 서버 전송 간격
        
        while self.running:
            try:
                # 드론 상태 업데이트 (더 자주 실행)
                self.update_drone_status()
                
                # 서버로 상태 전송 (제한된 주기로)
                current_time = asyncio.get_event_loop().time()
                if current_time - last_status_send >= status_send_interval:
                    await self.send_status_to_server()
                    last_status_send = current_time
                    
                # 짧은 대기 시간으로 더 자주 메시지 확인
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"상태 업데이트 루프 오류: {e}")
                await asyncio.sleep(1)
                
    async def message_receive_loop(self):
        """
        서버로부터 메시지를 수신하는 루프
        """
        while self.running:
            try:
                if self.websocket:
                    # 메시지 수신 대기
                    message = await self.websocket.recv()
                    
                    # 메시지 처리
                    await self.handle_server_message(message)
                    
            except websockets.exceptions.ConnectionClosed:
                logger.warning("서버 연결이 끊어졌습니다.")
                break
            except Exception as e:
                logger.error(f"메시지 수신 오류: {e}")
                await asyncio.sleep(1)
                
    async def start(self):
        """
        드론 에이전트를 시작하는 메서드
        """
        logger.info(f"드론 에이전트 시작: {self.drone_name}")
        
        # 드론 연결
        if not await self.connect_to_drone():
            logger.error("드론 연결 실패")
            return
            
        # 서버 연결
        if not await self.connect_to_server():
            logger.error("서버 연결 실패")
            return
        
      
        await self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000)  # 1초 간격으로 하트비트 전송
        await self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 10000000)  # 10초 간격으로 배터리 정보 전송        
        await self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1000000) # 1초 간격으로 VFR HUD 정보 전송
        await self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1000000) # 1초 간격으로 자세 정보 전송
            
        # 비동기 태스크 시작
        tasks = [
            asyncio.create_task(self.status_update_loop()),
            asyncio.create_task(self.message_receive_loop())
        ]
        
        try:
            # 모든 태스크 실행
            await asyncio.gather(*tasks)
        except KeyboardInterrupt:
            logger.info("사용자 종료 요청")
        finally:
            await self.cleanup()
            
    async def cleanup(self):
        """
        정리 작업을 수행하는 메서드
        """
        logger.info("드론 에이전트 정리 중...")
        
        self.running = False
        
        # WebSocket 연결 종료
        if self.websocket:
            await self.websocket.close()
            
        # 드론 연결 종료
        if self.drone_connection:
            self.drone_connection.close()
            
        logger.info("드론 에이전트 정리 완료")
        
    def get_flight_mode_name(self, heartbeat_msg):
        """
        HEARTBEAT 메시지에서 정확한 비행 모드 이름을 추출하는 헬퍼 메서드
        
        Args:
            heartbeat_msg: HEARTBEAT MAVLink 메시지
            
        Returns:
            str: 비행 모드 이름
        """
        try:
            # ArduCopter 모드 매핑 (custom_mode 값 기준)
            ardupilot_modes = {
                0: "STABILIZE",
                1: "ACRO",
                2: "ALT_HOLD",
                3: "AUTO",
                4: "GUIDED",
                5: "LOITER",
                6: "RTL",
                7: "CIRCLE",
                9: "LAND",
                11: "DRIFT",
                13: "SPORT",
                14: "FLIP",
                15: "AUTOTUNE",
                16: "POSHOLD",
                17: "BRAKE",
                18: "THROW",
                19: "AVOID_ADSB",
                20: "GUIDED_NOGPS",
                21: "SMART_RTL",
                22: "FLOWHOLD",
                23: "FOLLOW",
                24: "ZIGZAG",
                25: "SYSTEMID",
                26: "AUTOROTATE",
                27: "AUTO_RTL"
            }
            
            custom_mode = heartbeat_msg.custom_mode
            
            # ArduPilot 모드인 경우
            if custom_mode in ardupilot_modes:
                return ardupilot_modes[custom_mode]
            
            # 기본 MAVLink 모드 문자열 사용
            return mavutil.mode_string_v10(heartbeat_msg)
            
        except Exception as e:
            logger.debug(f"모드 이름 추출 오류: {e}")
            return f"Mode({heartbeat_msg.custom_mode})"
            
    async def goto_position(self, lat, lon, alt):
        """
        드론을 특정 위치로 이동시키는 메서드 (GUIDED 모드에서 사용)
        
        Args:
            lat: 위도
            lon: 경도  
            alt: 고도
            
        Returns:
            bool: 성공 여부
        """
        if not self.drone_connection:
            logger.error("드론이 연결되지 않았습니다.")
            return False
            
        if not self.drone_status["armed"]:
            logger.error("드론이 시동되지 않았습니다.")
            return False
            
        try:
            # GUIDED 모드 확인/설정
            if self.drone_status["mode"] != "GUIDED":
                logger.info("GUIDED 모드로 변경 중...")
                self.drone_connection.mav.set_mode_send(
                    self.drone_connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4  # GUIDED 모드
                )
                # 모드 변경 대기
                await asyncio.sleep(1)
            
            # SET_POSITION_TARGET_GLOBAL_INT 메시지 전송
            self.drone_connection.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.drone_connection.target_system,
                self.drone_connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask (position only)
                int(lat * 1e7),  # lat_int (degE7)
                int(lon * 1e7),  # lon_int (degE7)
                alt,  # alt (meters)
                0,  # vx
                0,  # vy
                0,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                0,  # yaw
                0   # yaw_rate
            )
            
            logger.info(f"위치 타겟 설정 완료: ({lat}, {lon}, {alt})")
            return True
            
        except Exception as e:
            logger.error(f"위치 이동 오류: {e}")
            return False

    def should_process_message(self, msg_type):
        """
        메시지 처리 여부를 결정하는 메서드 (레이트 리미팅)
        """
        if msg_type not in self.message_rate_limits:
            return True
            
        current_time = asyncio.get_event_loop().time()
        last_time = self.last_message_times.get(msg_type, 0)
        
        if current_time - last_time >= self.message_rate_limits[msg_type]:
            self.last_message_times[msg_type] = current_time
            return True
        return False

# 메인 실행 부분
async def main():
    # 드론 에이전트 설정
    drone_name = "Drone_001"
    connection_string = "udp:127.0.0.1:14550"  # SITL 기본 포트
    server_url = "ws://127.0.0.1:9876"
    
    # 드론 에이전트 생성 및 시작
    agent = DroneAgent(drone_name, connection_string, server_url)
    await agent.start()

if __name__ == "__main__":
    asyncio.run(main())

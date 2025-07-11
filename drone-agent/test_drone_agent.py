from pymavlink import mavutil
import time

vehicle = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
vehicle.wait_heartbeat()

def execute_command_long(cmd_code, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """
    드론에 명령을 전송하는 함수
    """
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        cmd_code,
        0,  # confirmation
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
    )
    result_msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if( result_msg.command==cmd_code and result_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        return True
    else:
        return False
    
CMD_CODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
main_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
custom_mode = 4  # 예: GUIDED 모드

if ( execute_command_long(CMD_CODE, main_mode, custom_mode) ):
    print("드론 모드 변경 성공")
else:
    print("드론 모드 변경 실패")

while True:
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        if msg.type != 6:  # 6은 MAV_TYPE_GCS
            print(msg)
    else:
        print("드론 상태 메시지를 수신하지 못했습니다.")
    time.sleep(3)
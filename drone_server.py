# 드론 서버를 위한 필수 모듈들을 import
import asyncio          # 비동기 프로그래밍을 위한 asyncio
import websockets       # WebSocket 통신을 위한 websockets
import json            # JSON 데이터 처리를 위한 json
import uuid            # 고유 식별자 생성을 위한 uuid
from datetime import datetime  # 시간 정보 처리를 위한 datetime
from typing import Dict, Set   # 타입 힌트를 위한 typing

class DroneServer:
    """
    드론과 GCS(Ground Control Station) 간의 통신을 담당하는 WebSocket 서버 클래스
    
    이 클래스는 다음과 같은 기능을 제공합니다:
    - 드론과 GCS 클라이언트의 연결 관리
    - 클라이언트 간의 메시지 중계
    - 명령 전송 및 상태 업데이트
    - 실시간 통신 지원
    """
    def __init__(self):
        # 연결된 모든 클라이언트의 정보를 저장하는 딕셔너리 (클라이언트 ID -> 클라이언트 정보)
        self.clients: Dict[str, dict] = {}
        
        # 드론 클라이언트들의 매핑 정보 (드론 이름 -> 클라이언트 ID)
        self.drones: Dict[str, str] = {}
        
        # GCS 클라이언트들의 매핑 정보 (GCS 이름 -> 클라이언트 ID)
        self.gcs_stations: Dict[str, str] = {}
        
    async def register_client(self, websocket, client_data):
        """
        새로운 클라이언트를 서버에 등록하는 메서드
        
        Args:
            websocket: 클라이언트와의 WebSocket 연결
            client_data: 클라이언트의 등록 정보 (role, name 등)
            
        Returns:
            tuple: (클라이언트 ID, 클라이언트 정보)
        """
        # 고유한 클라이언트 ID를 UUID로 생성
        client_id = str(uuid.uuid4())
        
        # 클라이언트 정보 객체 생성
        client_info = {
            "id": client_id,
            "role": client_data["role"],  # 역할 (drone, gcs)
            "name": client_data.get("name", f"{client_data['role']}_{client_id[:8]}"),  # 이름 (없으면 자동 생성)
            "websocket": websocket,  # WebSocket 연결 객체
            "connected_at": datetime.now().isoformat()  # 연결 시각
        }
        
        # 클라이언트 정보를 메인 딕셔너리에 저장
        self.clients[client_id] = client_info
        
        # 클라이언트 역할에 따라 해당 딕셔너리에도 저장
        if client_info["role"] == "drone":
            self.drones[client_info["name"]] = client_id
        elif client_info["role"] == "gcs":
            self.gcs_stations[client_info["name"]] = client_id
            
        # 모든 클라이언트에게 업데이트된 클라이언트 목록 전송
        await self.broadcast_client_list()
        return client_id, client_info
        
    async def unregister_client(self, client_id):
        """
        클라이언트의 연결을 해제하고 서버에서 제거하는 메서드
        
        Args:
            client_id: 제거할 클라이언트의 ID
        """
        if client_id in self.clients:
            client_info = self.clients[client_id]
            role = client_info["role"]
            name = client_info["name"]
            
            # 클라이언트 역할에 따라 해당 딕셔너리에서도 제거
            if role == "drone" and name in self.drones:
                del self.drones[name]
            elif role == "gcs" and name in self.gcs_stations:
                del self.gcs_stations[name]
                
            # 메인 클라이언트 딕셔너리에서 제거
            del self.clients[client_id]
            
            # 모든 클라이언트에게 업데이트된 클라이언트 목록 전송
            await self.broadcast_client_list()
            
    async def broadcast_client_list(self):
        """
        현재 연결된 모든 클라이언트에게 클라이언트 목록을 브로드캐스트하는 메서드
        
        이 메서드는 드론과 GCS 목록을 모든 클라이언트에게 전송하여
        현재 연결 상태를 동기화합니다.
        """
        # 클라이언트 목록 메시지 생성
        client_list = {
            "type": "client_list",
            "drones": list(self.drones.keys()),        # 연결된 드론 목록
            "gcs_stations": list(self.gcs_stations.keys()),  # 연결된 GCS 목록
            "timestamp": datetime.now().isoformat()    # 전송 시각
        }
        
        # JSON 형태로 직렬화
        message = json.dumps(client_list)
        disconnected_clients = []
        
        # 모든 클라이언트에게 메시지 전송
        for client_id, client_info in self.clients.items():
            try:
                await client_info["websocket"].send(message)
            except websockets.exceptions.ConnectionClosed:
                # 연결이 끊어진 클라이언트는 나중에 제거하기 위해 기록
                disconnected_clients.append(client_id)
                
        # 연결이 끊어진 클라이언트들을 서버에서 제거
        for client_id in disconnected_clients:
            await self.unregister_client(client_id)
            
    async def send_to_client(self, client_id: str, message: str):
        """
        특정 클라이언트에게 메시지를 전송하는 메서드
        
        Args:
            client_id: 대상 클라이언트의 ID
            message: 전송할 메시지
            
        Returns:
            bool: 전송 성공 여부
        """
        if client_id in self.clients:
            try:
                await self.clients[client_id]["websocket"].send(message)
                return True
            except websockets.exceptions.ConnectionClosed:
                # 연결이 끊어진 경우 클라이언트를 제거
                await self.unregister_client(client_id)
                return False
        return False
        
    async def send_to_role(self, role: str, message: str, exclude_client: str = None):
        """
        특정 역할을 가진 모든 클라이언트에게 메시지를 전송하는 메서드
        
        Args:
            role: 대상 역할 (drone, gcs)
            message: 전송할 메시지
            exclude_client: 제외할 클라이언트 ID (선택사항)
        """
        # 해당 역할을 가진 클라이언트들의 ID를 필터링
        targets = [client_id for client_id, client_info in self.clients.items() 
                  if client_info["role"] == role and client_id != exclude_client]
        
        # 각 대상 클라이언트에게 메시지 전송
        for client_id in targets:
            await self.send_to_client(client_id, message)
            
    async def handle_message(self, sender_id: str, message: str):
        """
        클라이언트로부터 받은 메시지를 처리하는 메인 메서드
        
        이 메서드는 메시지 타입에 따라 다음과 같이 처리합니다:
        - command: GCS에서 드론으로 명령 전송
        - status: 드론에서 GCS로 상태 업데이트
        - chat: 일반 채팅 메시지 브로드캐스트
        
        Args:
            sender_id: 메시지를 보낸 클라이언트의 ID
            message: 받은 메시지
        """
        try:
            # JSON 메시지 파싱
            data = json.loads(message)
            msg_type = data.get("type", "chat")
            
            # 발신자 정보 확인
            sender_info = self.clients.get(sender_id)
            if not sender_info:
                return
                
            if msg_type == "command" and sender_info["role"] == "gcs":
                # GCS에서 드론으로 명령 전송 처리
                target_drone = data.get("target_drone")
                if target_drone and target_drone in self.drones:
                    target_id = self.drones[target_drone]
                    
                    # 명령 메시지 구성
                    command_message = json.dumps({
                        "type": "command",
                        "sender": sender_info["name"],
                        "command": data.get("command"),
                        "parameters": data.get("parameters", {}),
                        "timestamp": datetime.now().isoformat()
                    })
                    # 대상 드론에게 명령 전송
                    await self.send_to_client(target_id, command_message)
                    
            elif msg_type == "status" and sender_info["role"] == "drone":
                # 드론에서 GCS로 상태 업데이트 전송 처리
                status_message = json.dumps({
                    "type": "drone_status",
                    "drone": sender_info["name"],        # 드론 이름
                    "battery": data.get("battery", 0),   # 배터리 상태
                    "position": data.get("position", {}), # 위치 정보
                    "status": data.get("status", "unknown"), # 드론 상태
                    "timestamp": datetime.now().isoformat()
                })
                # 모든 GCS에게 상태 정보 전송 (발신자 제외)
                await self.send_to_role("gcs", status_message, sender_id)
                
            else:
                # 일반 채팅 메시지 브로드캐스트 처리
                broadcast_message = json.dumps({
                    "type": "chat",
                    "sender": sender_info["name"],         # 발신자 이름
                    "sender_role": sender_info["role"],    # 발신자 역할
                    "message": data.get("message", message), # 메시지 내용
                    "timestamp": datetime.now().isoformat()
                })
                
                # 모든 클라이언트에게 전송 (발신자 제외)
                for client_id in self.clients:
                    if client_id != sender_id:
                        await self.send_to_client(client_id, broadcast_message)
                        
        except json.JSONDecodeError:
            # JSON 파싱 실패 시 일반 텍스트 메시지로 처리
            await self.handle_text_message(sender_id, message)
            
    async def handle_text_message(self, sender_id: str, message: str):
        """
        일반 텍스트 메시지를 처리하는 메서드
        
        JSON 형태가 아닌 일반 텍스트 메시지를 받았을 때 호출됩니다.
        
        Args:
            sender_id: 메시지를 보낸 클라이언트의 ID
            message: 받은 텍스트 메시지
        """
        sender_info = self.clients.get(sender_id)
        if not sender_info:
            return
            
        # 텍스트 메시지를 JSON 형태로 포맷팅
        broadcast_message = json.dumps({
            "type": "chat",
            "sender": sender_info["name"],
            "sender_role": sender_info["role"],
            "message": message,
            "timestamp": datetime.now().isoformat()
        })
        
        # 모든 클라이언트에게 전송 (발신자 제외)
        for client_id in self.clients:
            if client_id != sender_id:
                await self.send_to_client(client_id, broadcast_message)
    
    async def handler(self, websocket):
        """
        새로운 WebSocket 연결을 처리하는 핸들러 메서드
        
        이 메서드는 각 클라이언트 연결에 대해 다음과 같은 작업을 수행합니다:
        1. 클라이언트 등록
        2. 등록 확인 메시지 전송
        3. 메시지 수신 루프 실행
        4. 연결 종료 시 정리 작업
        
        Args:
            websocket: 클라이언트와의 WebSocket 연결
        """
        client_id = None
        
        try:
            # 클라이언트 등록 과정
            registration_data = await websocket.recv()  # 등록 데이터 수신
            client_data = json.loads(registration_data)  # JSON 파싱
            
            # 클라이언트를 서버에 등록
            client_id, client_info = await self.register_client(websocket, client_data)
            
            # 등록 성공 확인 메시지 전송
            await websocket.send(json.dumps({
                "type": "registration_success",
                "client_id": client_id,
                "message": f"Registered as {client_info['role']}: {client_info['name']}"
            }))
            
            # 서버 로그에 등록 정보 출력
            print(f"Client registered: {client_info['name']} ({client_info['role']})")
            
            # 메시지 수신 루프 - 클라이언트가 연결을 끊을 때까지 계속 실행
            while True:
                try:
                    message = await websocket.recv()  # 메시지 수신 대기
                    await self.handle_message(client_id, message)  # 메시지 처리
                except websockets.exceptions.ConnectionClosed:
                    # 클라이언트가 연결을 끊었을 때
                    print(f"Client {client_info['name']} disconnected")
                    break
                    
        except Exception as e:
            # 예외 발생 시 로그 출력
            print(f"Error in handler: {e}")
        finally:
            # 연결 종료 시 클라이언트 정리 작업
            if client_id:
                await self.unregister_client(client_id)
                
    async def start(self, host="127.0.0.1", port=8765):
        """
        드론 서버를 시작하는 메서드
        
        지정된 호스트와 포트에서 WebSocket 서버를 시작하고 
        클라이언트 연결을 대기합니다.
        
        Args:
            host: 서버 호스트 주소 (기본값: 127.0.0.1)
            port: 서버 포트 번호 (기본값: 8765)
        """
        # WebSocket 서버 시작
        server = await websockets.serve(self.handler, host, port)
        print(f"Drone Server started at ws://{host}:{port}")
        
        # 서버가 종료될 때까지 대기
        await server.wait_closed()

# 메인 실행 부분
if __name__ == "__main__":
    # 드론 서버 인스턴스 생성
    server = DroneServer()
    
    # 비동기 이벤트 루프를 사용하여 서버 시작
    asyncio.run(server.start())
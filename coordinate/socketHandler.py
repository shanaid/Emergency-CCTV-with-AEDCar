import socketio

global safetycar
safetycar = []

sio = socketio.AsyncServer(async_mode='asgi', path='/socket')
socket_app = socketio.ASGIApp(sio)

@sio.event(namespace='/socketio')
async def connect(sid, env) :
    print(str(sid), ' : connect')

@sio.event(namespace='/socketio')
async def disconnect(sid) :
    print(str(sid), ' : disconnect')
    global safetycar
    safetycar = []


@sio.event(namespace='/socketio')
async def send_pose(sid, data) :
    # print(data)
    global safetycar
    safetycar = data

def get_pose() :
    return safetycar


# 여기부터는 아마 이럴 거 같다!

#
# import socketio
#
# # 소켓 통신을 위한 AsyncServer와 ASGIApp 생성
# sio = socketio.AsyncServer(async_mode='asgi', path='/socket')
# socket_app = socketio.ASGIApp(sio)
#
# # 방과 SID(Socket ID)를 추적하기 위한 딕셔너리
# rooms_to_sid = {}
#
# @sio.event(namespace='/socketio')
# async def connect(sid, env):
#     print(f"{sid} : 연결됨")
#
# @sio.event(namespace='/socketio')
# async def disconnect(sid):
#     print(f"{sid} : 연결 해제됨")
#     # 방에서 연결이 해제된 SID를 제거
#     rooms_to_sid = {room: s for room, s in rooms_to_sid.items() if s != sid}
#
# # 특정 방에 데이터를 전송하는 함수
# async def send_to_room(room, event, data):
#     if room in rooms_to_sid:
#         await sio.emit(event, data, room=rooms_to_sid[room], namespace='/socketio')
#     else:
#         print(f"방 {room}이 연결되지 않았습니다.")

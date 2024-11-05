import cv2
import check_skeleton
import time
import asyncio

from collections import defaultdict, deque
from ultralytics import YOLO
from main import send_coordinate, authentication, disconnect
from udp import UdpSender


# 보행자의 쓰러짐을 y좌표를 통한 기울기 값으로 판단합니다
def is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    if nose_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False
    
    if nose_y < left_ankle_y or nose_y < right_ankle_y or nose_y < left_knee_y or nose_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y != 0.0 and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y != 0.0 and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if nose_y != 0.0 and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(nose_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True
    return False


def is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_ears = [y for y in [left_ear_y, right_ear_y] if y is not None and y > 0]
    average_ears_y = sum(valid_ears) / len(valid_ears) if valid_ears else None

    if average_ears_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False

    if average_ears_y < left_ankle_y or average_ears_y < right_ankle_y or average_ears_y < left_knee_y or average_ears_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y is not None and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y is not None and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if average_ears_y is not None and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(average_ears_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True
    return False


def is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_eyes = [y for y in [left_eye_y, right_eye_y] if y is not None and y > 0]
    average_eyes_y = sum(valid_eyes) / len(valid_eyes) if valid_eyes else None

    if average_eyes_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False

    if average_eyes_y < left_ankle_y or average_eyes_y < right_ankle_y or average_eyes_y < left_knee_y or average_eyes_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y is not None and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y is not None and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if average_eyes_y is not None and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(average_eyes_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True
    return False


# 그리드 영역을 통해 보행자가 비스듬히 쓰러졌는지, 수평으로 쓰러졌는지 판단합니다.
def check_nose_grid(check_list, bounding_box):
    
    nose_x, nose_y = check_list[0], check_list[1]
    left_ankle_x, left_ankle_y = check_list[2], check_list[3]
    right_ankle_x, right_ankle_y = check_list[4], check_list[5]
    left_knee_x, left_knee_y = check_list[6], check_list[7]
    right_knee_x, right_knee_y = check_list[8], check_list[9]

    # 상체 기준
    upper_x, upper_y = nose_x, nose_y

    # 하체 기준
    if left_ankle_y is not None and right_ankle_y is not None:
        lower_x = (left_ankle_x + right_ankle_x) / 2
        lower_y = (left_ankle_y + right_ankle_y) / 2
    elif left_ankle_y is not None:
        lower_x, lower_y = left_ankle_x, left_ankle_y
    elif right_ankle_y is not None:
        lower_x, lower_y = right_ankle_x, right_ankle_y
    else:  # left_ankle_y와 right_ankle_y가 모두 None인 경우
        if left_knee_y is not None and right_knee_y is not None:
            lower_x = (left_knee_x + right_knee_x) / 2
            lower_y = (left_knee_y + right_knee_y) / 2
        elif left_knee_y is not None:
            lower_x, lower_y = left_knee_x, left_knee_y
        elif right_knee_y is not None:
            lower_x, lower_y = right_knee_x, right_knee_y
        else:
            return False  # 모든 관련 y 값이 None인 경우

    # 그리드 위치 계산
    upper_grid_pos = check_skeleton.get_grid_position(bounding_box, upper_x, upper_y)
    lower_grid_pos = check_skeleton.get_grid_position(bounding_box, lower_x, lower_y)

    # 경사 판단
    if check_skeleton.incline_judgment(upper_grid_pos, lower_grid_pos):
        return 1
    else:
        return 0


def check_eye_ear_grid(check_list, bounding_box):
    
    left_ear_x, left_ear_y = check_list[0], check_list[1]
    right_ear_x, right_ear_y = check_list[2], check_list[3]
    
    left_eye_x, left_eye_y = check_list[4], check_list[5]
    right_eye_x, right_eye_y = check_list[6], check_list[7]

    left_ankle_x, left_ankle_y = check_list[8], check_list[9]
    right_ankle_x, right_ankle_y = check_list[10], check_list[11]

    left_knee_x, left_knee_y = check_list[12], check_list[13]
    right_knee_x, right_knee_y = check_list[14], check_list[15]

    # 상체 기준
    if left_ear_y is not None and right_ear_y is not None:
        upper_x = (left_ear_x + right_ear_x) / 2
        upper_y = (left_ear_y + right_ear_y) / 2
    elif left_ear_y is not None:
        upper_x, upper_y = left_ear_x, left_ear_y
    elif right_ear_y is not None:
        upper_x, upper_y = right_ear_x, right_ear_y
    else:  # left_ear_y와 right_ear_y가 모두 None인 경우
        if left_eye_y is not None and right_eye_y is not None:
            upper_x = (left_eye_x + right_eye_x) / 2
            upper_y = (left_eye_y + right_eye_y) / 2
        elif left_eye_y is not None:
            upper_x, upper_y = left_eye_x, left_eye_y
        elif right_eye_y is not None:
            upper_x, upper_y = right_eye_x, right_eye_y
        else:
            return False  # 모든 관련 y 값이 None인 경우

    # 하체 기준
    if left_ankle_y is not None and right_ankle_y is not None:
        lower_x = (left_ankle_x + right_ankle_x) / 2
        lower_y = (left_ankle_y + right_ankle_y) / 2
    elif left_ankle_y is not None:
        lower_x, lower_y = left_ankle_x, left_ankle_y
    elif right_ankle_y is not None:
        lower_x, lower_y = right_ankle_x, right_ankle_y
    else:  # left_ankle_y와 right_ankle_y가 모두 None인 경우
        if left_knee_y is not None and right_knee_y is not None:
            lower_x = (left_knee_x + right_knee_x) / 2
            lower_y = (left_knee_y + right_knee_y) / 2
        elif left_knee_y is not None:
            lower_x, lower_y = left_knee_x, left_knee_y
        elif right_knee_y is not None:
            lower_x, lower_y = right_knee_x, right_knee_y
        else:
            return False  # 모든 관련 y 값이 None인 경우

    # 그리드 위치 계산
    upper_grid_pos = check_skeleton.get_grid_position(bounding_box, upper_x, upper_y)
    lower_grid_pos = check_skeleton.get_grid_position(bounding_box, lower_x, lower_y)

    # 경사 판단
    if check_skeleton.incline_judgment(upper_grid_pos, lower_grid_pos):
        return 1
    else:
        return 0


# 좌표를 딕셔너리로 추출합니다.
def extract_keypoint(keypoint_data):
    keypoints = {
        'nose': {'x': None, 'y': None},
        'left_eye': {'x': None, 'y': None},
        'right_eye': {'x': None, 'y': None},
        'left_ear': {'x': None, 'y': None},
        'right_ear': {'x': None, 'y': None},
        'left_knee': {'x': None, 'y': None},
        'right_knee': {'x': None, 'y': None},
        'left_ankle': {'x': None, 'y': None},
        'right_ankle': {'x': None, 'y': None}
    }

    # 좌표 저장
    if keypoint_data[0][0][2].numel() == 1 and keypoint_data[0][0][1] > 0:
        keypoints['nose']['x'] = keypoint_data[0][0][0].item()
        keypoints['nose']['y'] = keypoint_data[0][0][1].item()
    
    if keypoint_data[0][1][2].numel() == 1 and keypoint_data[0][1][1] > 0:
        keypoints['left_eye']['x'] = keypoint_data[0][1][0].item()
        keypoints['left_eye']['y'] = keypoint_data[0][1][1].item()
    
    if keypoint_data[0][2][2].numel() == 1 and keypoint_data[0][2][1] > 0:
        keypoints['right_eye']['x'] = keypoint_data[0][2][0].item()
        keypoints['right_eye']['y'] = keypoint_data[0][2][1].item()
    
    if keypoint_data[0][3][2].numel() == 1 and keypoint_data[0][3][1] > 0:
        keypoints['left_ear']['x'] = keypoint_data[0][3][0].item()
        keypoints['left_ear']['y'] = keypoint_data[0][3][1].item()
    
    if keypoint_data[0][4][2].numel() == 1 and keypoint_data[0][4][1] > 0:
        keypoints['right_ear']['x'] = keypoint_data[0][4][0].item()
        keypoints['right_ear']['y'] = keypoint_data[0][4][1].item()

    if keypoint_data[0][13][2].numel() == 1 and keypoint_data[0][13][1] > 0:
        keypoints['left_knee']['x'] = keypoint_data[0][13][0].item()
        keypoints['left_knee']['y'] = keypoint_data[0][13][1].item()
    
    if keypoint_data[0][14][2].numel() == 1 and keypoint_data[0][14][1] > 0:
        keypoints['right_knee']['x'] = keypoint_data[0][14][0].item()
        keypoints['right_knee']['y'] = keypoint_data[0][14][1].item()
    
    if keypoint_data[0][15][2].numel() == 1 and keypoint_data[0][15][1] > 0:
        keypoints['left_ankle']['x'] = keypoint_data[0][15][0].item()
        keypoints['left_ankle']['y'] = keypoint_data[0][15][1].item()
    
    if keypoint_data[0][16][2].numel() == 1 and keypoint_data[0][16][1] > 0:
        keypoints['right_ankle']['x'] = keypoint_data[0][16][0].item()
        keypoints['right_ankle']['y'] = keypoint_data[0][16][1].item()

    return keypoints


def is_falling_func(width, height, keypoint_data, is_falling, bounding_box):
    keypoints = extract_keypoint(keypoint_data)

    # y 좌표 확인
    nose_x = keypoints['nose']['x']
    nose_y = keypoints['nose']['y']
    left_eye_x = keypoints['left_eye']['x']
    left_eye_y = keypoints['left_eye']['y']
    right_eye_x = keypoints['right_eye']['x']
    right_eye_y = keypoints['right_eye']['y']
    left_ear_x = keypoints['left_ear']['x']
    left_ear_y = keypoints['left_ear']['y']
    right_ear_x = keypoints['right_ear']['x']
    right_ear_y = keypoints['right_ear']['y']
    left_ankle_x = keypoints['left_ankle']['x']
    left_ankle_y = keypoints['left_ankle']['y']
    left_knee_x = keypoints['left_knee']['x']
    left_knee_y = keypoints['left_knee']['y']
    right_ankle_x = keypoints['right_ankle']['x']
    right_ankle_y = keypoints['right_ankle']['y']
    right_knee_x = keypoints['right_knee']['x']
    right_knee_y = keypoints['right_knee']['y']


    if left_ankle_y is None and left_knee_y is None and right_ankle_y is None and right_knee_y is None:
        return False
    
    aspect_ratio_threshold = 1.2

    if width > height * aspect_ratio_threshold:
        grid_status = 0
        if nose_y is not None and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
            grid_status = check_nose_grid([nose_x, nose_y, left_ankle_x, left_ankle_y, right_ankle_x, right_ankle_y,
                                    left_knee_x, left_knee_y, right_knee_x, right_knee_y], bounding_box)
            if grid_status:
                if is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, 70):
                    return True
            else:
                if is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                    return True

        if not is_falling and (left_ear_y is not None or right_ear_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
            grid_status = check_eye_ear_grid([left_ear_x, left_ear_y, right_ear_x, right_ear_y, left_eye_x, left_eye_y, right_eye_x, right_eye_y, left_ankle_x, left_ankle_y,
                                              right_ankle_x, right_ankle_y, left_knee_x, left_knee_y, right_knee_x, right_knee_y], bounding_box)
            if is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                return True
        
        if not is_falling and (left_eye_y is not None or right_eye_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
            if is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                return True

    return False


def check_row(keypoint_data, slope_threshold=50):
    keypoints = extract_keypoint(keypoint_data)

    # 키포인트 좌표 추출
    nose_x = keypoints['nose']['x'] if 'nose' in keypoints else None
    left_ankle_x = keypoints['left_ankle']['x'] if 'left_ankle' in keypoints else None
    left_knee_x = keypoints['left_knee']['x'] if 'left_knee' in keypoints else None
    right_ankle_x = keypoints['right_ankle']['x'] if 'right_ankle' in keypoints else None
    right_knee_x = keypoints['right_knee']['x'] if 'right_knee' in keypoints else None

    # 하체 좌표 리스트 생성
    lower_body_x = []
    if left_ankle_x is not None:
        lower_body_x.append(left_ankle_x)
    if left_knee_x is not None:
        lower_body_x.append(left_knee_x)
    if right_ankle_x is not None:
        lower_body_x.append(right_ankle_x)
    if right_knee_x is not None:
        lower_body_x.append(right_knee_x)

    # 하체 좌표가 있을 경우 판단
    if len(lower_body_x) >= 2:
        # x좌표의 평균 계산
        average_x = sum(lower_body_x) / len(lower_body_x)
        
        # 각 좌표와 평균 사이의 차이를 계산
        x_diffs = [abs(x - average_x) for x in lower_body_x]

        # 모든 차이가 임계값 내에 있는지 확인
        threshold_met = all(diff < slope_threshold for diff in x_diffs)
    elif len(lower_body_x) == 1:
        # 하나의 좌표만 있을 경우
        threshold_met = True  
    else:
        threshold_met = False

    return threshold_met



async def pose_estimation(keypoints, results, check_data, tracking_data, annotated_frame):
    send_x, send_y = None, None
    start_x, start_y, end_x, end_y = None, None, None, None

    # 바운딩 박스 좌표 추출
    if results[0].boxes is not None and len(results[0].boxes) > 0:
        for i, box in enumerate(results[0].boxes):
            if box is None or box.id is None:
                continue

            tracking_id = box.id.item()
            x1, y1, x2, y2 = box.xyxy[0]
            width = box.xywh[0][2].item()
            height = box.xywh[0][3].item()

            if keypoints is not None and len(keypoints) > i:
                keypoint_data = keypoints[i].data

                if keypoint_data.size(1) <= 17:
                    # 각 키포인트의 좌표 추출
                    is_falling = is_falling_func(width, height, keypoint_data, False, box.xyxy[0])
                    current_time = time.time()
                    status_type = 2

                    # 높이 기반 쓰러짐 판단 로직
                    if tracking_id not in tracking_data:
                        tracking_data[tracking_id] = {
                            'start_time': current_time,
                            'sent': False,
                            'height_history': [],
                            'is_still_falling': False
                        }

                    # 현재 높이 저장
                    tracking_data[tracking_id]['height_history'].append(height)

                    # 이전 높이와 비교하여 쓰러짐 판단
                    if len(tracking_data[tracking_id]['height_history']) > 4:
                        height_three_seconds_ago = tracking_data[tracking_id]['height_history'][-4]  # 3초 전의 높이
                        if height_three_seconds_ago - height > 100:  # 높이 차이 비교
                            is_falling = check_row(keypoint_data)

                    # 오래된 높이 기록 삭제 (최대 10개 유지)
                    if len(tracking_data[tracking_id]['height_history']) > 10:
                        tracking_data[tracking_id]['height_history'].pop(0)

                    # 쓰러짐 상태 판단
                    if is_falling:
                        # 쓰러진 상태로 판단되면
                        if not tracking_data[tracking_id]['is_still_falling']:
                            tracking_data[tracking_id]['start_time'] = current_time  # 쓰러진 시간 업데이트
                            tracking_data[tracking_id]['is_still_falling'] = True  # 쓰러진 상태 플래그 설정

                        elapsed_time = current_time - tracking_data[tracking_id]['start_time']  # 항상 업데이트
                        if elapsed_time >= 5:
                            status_type = 0  # Falling 상태
                            # 객체 발 밑 좌표
                            send_x, send_y = (x1 + x2) / 2, y2
                            start_x, start_y, end_x, end_y = x1, y1, x2, y2
                            if not tracking_data[tracking_id]['sent']:
                                await send_coordinate(float(send_x), float(send_y), 0)
                                print(send_x, send_y)
                                tracking_data[tracking_id]['sent'] = True
                        else:
                            status_type = 1  # Falling (not confirmed)
                    else:
                        # 쓰러지지 않은 경우에는 상태를 초기화하지 않고 유지
                        elapsed_time = current_time - tracking_data[tracking_id]['start_time']
                        if tracking_data[tracking_id]['is_still_falling'] and elapsed_time < 5:
                            status_type = 1  # Falling (not confirmed)
                        else:
                            status_type = 2  # Standing
                            tracking_data[tracking_id]['is_still_falling'] = False  # 상태 초기화

                    status_list = [{'text': 'Falling', 'color': (0, 0, 255)},
                                   {'text': 'Falling (not confirmed)', 'color': (0, 255, 255)},
                                   {'text': 'Standing', 'color': (255, 0, 0)}]
                    
                    status = status_list[status_type]

                    # 현재 상태를 이미지에 표시
                    cv2.putText(annotated_frame, status['text'], (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status['color'], 2)

    lst = []
    if start_x is not None:
        lst.append((int(start_x), int(start_y), int(end_x), int(end_y)))
    
    return (annotated_frame, lst)


async def process_video(udp_sender, camera_id):
    model = YOLO("model/yolov8s-pose.pt").to('cuda')

    # 동영상 파일 열기
    video_path = "falling2.mp4"
    cap = cv2.VideoCapture(0)

    # 객체 상태 추적
    tracking_data = {}
    check_data = {}

    # 프레임 건너뛰기 설정
    frame_skip = 2  # 2프레임마다 처리

    while True:
        # 프레임 건너뛰기
        for _ in range(frame_skip):
            ret, frame = cap.read()
            if not ret:
                break

        if not ret:
            break

        # 이미지에서 포즈 추정 수행
        results = model.track(frame, persist=True)

        # 결과를 이미지에 표시
        annotated_frame = results[0].plot()

        # 스켈레톤 데이터 추출
        keypoints = results[0].keypoints

        annotated_frame, bounding_box = await pose_estimation(keypoints, results, check_data, tracking_data, annotated_frame)
        

        if bounding_box:
            for (x1, y1, x2, y2) in bounding_box:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # 이미지 출력
        cv2.imshow('Pose Estimation', annotated_frame)

        udp_sender.send_frame(annotated_frame)

        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            await disconnect(camera_id)
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

async def send_accesstoken():
    result = await authentication()
    print(result)
    return result

if __name__ == "__main__":
    SERVER_IP = "43.202.61.242"
    PORT = 5432
    result = asyncio.run(send_accesstoken())
    udp_sender = UdpSender(SERVER_IP, PORT, result['camera_id'])
    
    try:
        asyncio.run(process_video(udp_sender, result['camera_id']))
    finally:
        udp_sender.close()
    asyncio.run(process_video())
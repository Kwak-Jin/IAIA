import cv2
import os

# 비디오 캡처 객체 생성 (웹캠 사용)
cap = cv2.VideoCapture(2)  # 0은 기본 웹캠

# 저장할 디렉토리 지정
save_dir = "/home/gyeonheal/catkin_ws/src/ur_python/src/captured_images_checker"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 이미지 번호 추적
image_count = 20

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 프레임 표시
    cv2.imshow('Video Feed', frame)

    # 's' 키를 눌렀을 때 이미지 저장
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # 's' 키
        # 이미지 파일 이름 설정 (예: captured_images/1.jpg)
        filename = os.path.join(save_dir, f"{image_count}.jpg")
        cv2.imwrite(filename, frame)
        print(f"프레임이 {filename}으로 저장되었습니다.")
        
        # 이미지 번호 증가
        image_count += 1

    # 'q' 키를 눌러 종료
    elif key == ord('q'):
        break

# 비디오 캡처 객체 해제 및 윈도우 종료
cap.release()
cv2.destroyAllWindows()

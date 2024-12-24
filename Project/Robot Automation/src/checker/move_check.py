import cv2
import numpy as np

# 원본 이미지 로드
# image = cv2.imread("game_play/20.jpg")
#image = cv2.imread("/home/gun/catkin_ws/src/ur_python/src/base_image/initial.jpg")

cap = cv2.VideoCapture(2)
_, image = cap.read()
cv2.imshow("mainshow", image)

# 
# src_points (입력 이미지에서 ROI의 꼭짓점 좌표)
src_points = np.array([
        [132, 15], [575, 12], [634, 471], [103, 479]
], dtype='float32')

# dst_points (출력 이미지에서 네 꼭짓점을 직사각형으로 매핑)
width, height = 800, 600  # 원하는 출력 크기
dst_points = np.array([
    [0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]
], dtype='float32')

# 변환 행렬 계산
matrix = cv2.getPerspectiveTransform(src_points, dst_points)

# 원근 변환 적용
warped = cv2.warpPerspective(image, matrix, (width, height))

cv2.imshow("warped image",warped)
# warped = cv2.rotate(warped, cv2.ROTATE_180)

stone = warped.copy()

# Define the corners of the board
x_left_upper_corner = 51
y_left_upper_corner = 44
x_right_down_corner = 749
y_right_dwon_croner = 564

# Calculate the distance for each box
x_dist = round(abs(x_left_upper_corner - x_right_down_corner) / 8)
y_dist = round(abs(y_left_upper_corner - y_right_dwon_croner) / 8)

# List to store box IDs
box_data = []
box_id = []

# Starting points
start_x = x_left_upper_corner
start_y = y_left_upper_corner

# Draw grid and assign IDs
for i in range(8):
    for j in range(8):
        # Calculate top-left and bottom-right corners for the current box
        x1 = start_x + j * x_dist
        y1 = start_y + i * y_dist
        x2 = x1 + x_dist
        y2 = y1 + y_dist

        reversed_i = 7 - i
        reversed_j = 7 - j

        # # Assign a unique box ID
        box_data.append(((reversed_i, reversed_j), (y1, y2, x1, x2)))

        # Draw the rectangle on the image
        cv2.rectangle(warped, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=3)

        # Optionally display the ID in the center of the box
        text_x = x1 + x_dist // 2
        text_y = y1 + y_dist // 2
        cv2.putText(warped, f"{reversed_i},{reversed_j}", (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# 결과 _확인
# cv2.imshow("Original Image", image)
cv2.imshow("Warped ROI", warped)
cv2.imwrite("warped_real_coordinate.png", warped)

# ---------------------------- check stones ----------------------------------#

# coordinate for white and black stone
ws_coord = []
bs_coord = []

# number of remaining w, b stones
count_ws = 0
count_bs = 0

# thresholds
low_white = 190

high_black = 60
for box_id, box_coord in box_data:


    # center box calculation for smaller ROI
    y1, y2, x1, x2 = box_coord

    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2

    half_width = (x2 - x1) // 8  # 가로 절반의 절반
    half_height = (y2 - y1) // 8  # 세로 절반의 절반

    new_x1 = center_x - half_width
    new_x2 = center_x + half_width
    new_y1 = center_y - half_height
    new_y2 = center_y + half_height
    #cv2.rectangle(stone, (new_x1, new_y1), (new_x2, new_y2), (255, 0, 255), 2)

    # calculation for mean BGR of each box 
    region = stone[new_y1:new_y2 , new_x1:new_x2]
    #region = stone[y1: y1, x1:x2]
    avg_bgr = np.mean(region, axis=(0, 1))
    mean = (avg_bgr[0]+avg_bgr[1]+avg_bgr[2])/3


    # detect white and black stones / draw rectangle
    if avg_bgr[0] >= low_white:
        cv2.rectangle(stone, (new_x1, new_y1), (new_x2, new_y2), (255, 0, 0), 2)
        #cv2.rectangle(stone, (x1, y1), (x2, y2), (255, 0, 0), 2)
        ws_coord.append(box_id)
        count_ws = count_ws + 1
    elif mean < high_black:
        cv2.rectangle(stone, (new_x1, new_y1), (new_x2, new_y2), (0, 0, 255), 2)
        #cv2.rectangle(stone, (x1, y1), (x2, y2), (0, 0, 255), 2)
        bs_coord.append(box_id)
        count_bs = count_bs + 1

print(f"Remaining white stones{count_ws}")
print(f"Remaining black stones{count_bs}")


# # 이전 데이터 로드 함수
# def load_coords_from_file(filename):
#     try:
#         with open(filename, "r") as file:
#             return [eval(line.strip()) for line in file.readlines()]  # 문자열을 튜플로 변환
#     except FileNotFoundError:
#         return []  # 파일이 없으면 빈 리스트 반환

# # 현재 데이터 저장 함수
# def save_coords_to_file(filename, data):
#     with open(filename, "w") as file:
#         for coord in data:
#             file.write(f"{coord}\n")

# # 이전 데이터 로드
# prev_ws_coord = load_coords_from_file("coord_check/ws_coord.txt")
# prev_bs_coord = load_coords_from_file("coord_check/bs_coord.txt")

# # 처음 실행이라면 현재 좌표를 초기 좌표로 저장
# if not prev_ws_coord:
#     save_coords_to_file("coord_check/ws_coord.txt", ws_coord)
#     initial_ws_coord = ws_coord

# if not prev_bs_coord:
#     save_coords_to_file("coord_check/bs_coord.txt", bs_coord)
#     initial_bs_coord = bs_coord

# # 이전 데이터와 현재 데이터 비교
# new_ws = set(ws_coord) - set(prev_ws_coord)  # 새로 추가된 좌표
# removed_ws = set(prev_ws_coord) - set(ws_coord)  # 제거된 좌표

# new_bs = set(bs_coord) - set(prev_bs_coord)  # 새로 추가된 좌표
# removed_bs = set(prev_bs_coord) - set(bs_coord)  # 제거된 좌표

# # 결과 출력
# print("White stones (compared to previous):")
# print(" - Newly added:", new_ws)
# print(" - Removed:", removed_ws)

# print("Black stones (compared to previous):")
# print(" - Newly added:", new_bs)
# print(" - Removed:", removed_bs)

# # 현재 데이터를 파일에 저장 (이후 실행에서 "이전 데이터"로 사용)
# save_coords_to_file("coord_check/ws_coord.txt", ws_coord)
# save_coords_to_file("coord_check/bs_coord.txt", bs_coord)

print(bs_coord)

# ------------ show result in image -------------------#
cv2.imshow("showshow",stone)
cv2.waitKey(0)
cv2.destroyAllWindows()
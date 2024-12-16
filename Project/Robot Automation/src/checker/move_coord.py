import cv2
import numpy as np

def image2coordinate(image):
    # 원본 이미지 로드
    # image = cv2.imread(image_path)

    # src_points (입력 이미지에서 ROI의 꼭짓점 좌표)
    src_points = np.array([
        [132, 15], [575, 12], [634, 471], [103, 479]
    ], dtype='float32')

    # dst_points (출력 이미지에서 네 꼭짓점을 직사각형으로 매핑)
    width, height = 800, 600  # 원하는 출력 크기
    dst_points = np.array([
        [0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]
    ], dtype='float32')

    # 변환 행렬 계산 및 원근 변환 적용
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warped = cv2.warpPerspective(image, matrix, (width, height))
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
    start_x = x_left_upper_corner
    start_y = y_left_upper_corner

    # Draw grid and assign IDs
    for i in range(8):
        for j in range(8):
            x1 = start_x + j * x_dist
            y1 = start_y + i * y_dist
            x2 = x1 + x_dist
            y2 = y1 + y_dist

            r_i = 7-i
            r_j = 7-j

            box_data.append(((r_i, r_j), (y1, y2, x1, x2)))

    # White and black stones coordinates
    ws_coord = []
    bs_coord = []

    # Thresholds for detection
    low_white = 190
    high_black = 70

    for box_id, box_coord in box_data:
        y1, y2, x1, x2 = box_coord

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        half_width = (x2 - x1) // 8
        half_height = (y2 - y1) // 8

        new_x1 = center_x - half_width
        new_x2 = center_x + half_width
        new_y1 = center_y - half_height
        new_y2 = center_y + half_height

        region = stone[new_y1:new_y2, new_x1:new_x2]
        avg_bgr = np.mean(region, axis=(0, 1))
        mean = (avg_bgr[0] + avg_bgr[1] + avg_bgr[2]) / 3

        if avg_bgr[0] >= low_white:
            cv2.rectangle(stone, (new_x1, new_y1), (new_x2, new_y2), (255, 0, 0), 2)
            ws_coord.append(box_id)
        elif mean < high_black:
            cv2.rectangle(stone, (new_x1, new_y1), (new_x2, new_y2), (0, 0, 255), 2)
            bs_coord.append(box_id)

    # 이전 데이터 로드 함수
    def load_coords_from_file(filename):
        try:
            with open(filename, "r") as file:
                return [eval(line.strip()) for line in file.readlines()]
        except FileNotFoundError:
            return []

    # 현재 데이터 저장 함수
    def save_coords_to_file(filename, data):
        with open(filename, "w") as file:
            for coord in data:
                file.write(f"{coord}\n")

    # 이전 데이터 로드
    prev_bs_coord = load_coords_from_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")

    if not prev_bs_coord:
        prev_bs_coord = load_coords_from_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/initial_bs_coord.txt")


    new_bs = set(bs_coord) - set(prev_bs_coord)
    removed_bs = set(prev_bs_coord) - set(bs_coord)

    # 현재 데이터를 파일에 저장
    save_coords_to_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt", bs_coord)

    # 반환 값
    return removed_bs, new_bs
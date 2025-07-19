import cv2
import numpy as np
import socket


def get_angle(bc, cc):
    # Векторы базового маркера (от левого к правому)
    base_v1 = bc[1] - bc[0]
    # Векторы текущего маркера
    curr_v1 = cc[1] - cc[0]

    # Угол между векторами
    dot_product = np.dot(base_v1, curr_v1)
    norm_product = np.linalg.norm(base_v1) * np.linalg.norm(curr_v1)
    if norm_product == 0:
        return 0.0
    cos_angle = dot_product / norm_product
    angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    angle -= 90
    if abs(angle) > 45:
        if angle > 0:
            angle -= 90
        else:
            angle += 90
    return angle


def get_cords(frame):
    # Массивы и словари для хранения
    useful, trash, centers, corners_dict = {}, {}, {}, {}
    all_corners, all_ids = [], []
    base_ids, find_ids = {0, 1, 2, 3}, {40, 50, 60, 70, 80}

    # Проверка изображения
    if frame is None:
        exit(2)

    # Перевод в оттенки серого
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Словари ArUco для проверки
    dictionaries = {
        'DICT_4X4_250': cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250),
        'DICT_5X5_250': cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250),
        'DICT_6X6_250': cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
    }

    # Обнаружение маркеров во всех словарях
    for name, aruco_dict in dictionaries.items():
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is not None:
            for i in range(len(ids)):
                all_corners.append(corners[i])
                all_ids.append(ids[i])

    # Проверка наличия маркеров
    if not all_ids:
        exit(3)

    # Преобразуем ids в одномерный массив
    all_ids = np.array(all_ids).flatten()

    # Сохраняем центры
    for i, corner in enumerate(all_corners):
        marker_id = int(all_ids[i])
        points = corner[0].astype(int)
        center_x = int(np.mean(points[:, 0]))
        center_y = int(np.mean(points[:, 1]))
        centers[marker_id] = np.array([center_x, center_y], dtype=np.float32)
        corners_dict[marker_id] = corner[0].astype(np.float32)

    # Проверяем, есть ли маркеры 0, 1, 2, 3
    missing = [mid for mid in base_ids if mid not in centers]
    if missing:
        exit(4)

    # Строим систему координат
    p0, p1, p2, p3 = centers[0], centers[1], centers[2], centers[3]

    # Координаты в мм (X от 0 к 1) & (Y от 0 к 3)
    mm0 = np.array([25, 25], dtype=np.float32)
    mm2 = np.array([325, 325], dtype=np.float32)
    mm1 = np.array([325, 25], dtype=np.float32)
    mm3 = np.array([25, 325], dtype=np.float32)

    # Точки в пикселях и мм
    src_points = np.array([p0, p1, p2, p3], dtype=np.float32)
    dst_points = np.array([mm0, mm1, mm2, mm3], dtype=np.float32)

    # Аффинное преобразование
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)


    def pixel(pixel_point):
        pixel_point = np.array([[[pixel_point[0], pixel_point[1]]]], dtype=np.float32)
        mm_point = cv2.perspectiveTransform(pixel_point, matrix)
        return mm_point[0][0]


    # Равноудалённая точка
    eqt_pixel = np.mean([p0, p1, p2, p3], axis=0)
    eqt_mm = pixel(eqt_pixel)
    useful[-1] = tuple([round(float(eqt_mm[0]), 2), round(float(eqt_mm[1]), 2), 0.0])

    # Определение поворота маркеров
    rotate_id = 0

    # Проверка на наличие и распознавание ID 0
    if rotate_id not in corners_dict:
        exit(5)

    base_corners = corners_dict[rotate_id]

    # Обработка новых маркеров
    for marker_id in all_ids:
        if marker_id not in base_ids:
            center_pixel = centers[marker_id]
            center_mm = pixel(center_pixel)
            current_corners = corners_dict[marker_id]
            angle = get_angle(base_corners, current_corners)


            # Выводим в консоль
            if marker_id in find_ids:
                useful[int(marker_id)] = [round(float(center_mm[0]), 2), round(float(center_mm[1]), 2), round(float(angle), 2)]
            else:
                trash[int(marker_id)] = [round(float(center_mm[0]), 2), round(float(center_mm[1]), 2), round(float(angle), 2)]

    # Возвращаем словари из точек
    return [useful, trash]


def get_frame(camera_index):
    # Инициализация камеры
    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        exit(6)

    # Сделать снимок
    ret, frame = camera.read()
    if not ret:
        camera.release()  # Освобождаем камеру
        exit(7)

    # Освобождение камеры
    camera.release()

    return frame


class RobotClient:
    def __init__(self, host, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((host, port))

    def send_command(self, command):
        print(f"[CLIENT] Отправляем команду: {command}")
        self.s.sendall((command + '\n').encode())
        data = self.s.recv(1024)
        response = data.decode().strip()
        print("[CLIENT] Ответ от сервера:", response)
        if response == "BUSY":
            print("[CLIENT] Сервер занят, завершаем работу.")
            self.s.close()
            raise RuntimeError("Server is busy")
        return response

    def close(self):
        self.s.close()


def move_to(x, y, z, robot):
    robot.send_command(f"MOVE_TO {x} {y} {z}")


def on(robot):
    robot.send_command("TOOL_VACUUM_ON")


def off(robot):
    robot.send_command("TOOL_VACUUM_OFF")


def set_speed(x, y, z, robot):
    robot.send_command(f"SET_MAX_SPEED {x} {y} {z}")


def rotate_to(angle, robot):
    robot.send_command(f"TOOL_ROTATE_TO {angle}")


def bring_and_drop(x1, y1, x2, y2, ang, robot):
    move_to(x1, y1, 10, client)
    move_to(x1, y1, 0, client)
    on(client)
    rotate_to(ang, robot)
    move_to(x2, y2, 10, client)
    move_to(x2, y2, 8, client)
    off(client)


result = get_cords(get_frame(0))
to_move = result[0]
to_bin = result[1]


client = RobotClient('127.0.0.1', 5000)
try:
    client.send_command('SET_MAX_SPEED 1500 1500 1500')
    move_to(400, 410, 10, client)
    result = get_cords(get_frame(0))
    to_move = result[0]
    to_bin = result[1]

    for key in to_bin.keys():
        ax, ay = to_bin[key][0], to_bin[key][1]
        move_to(ax, ay, 10, client)
        move_to(ax, ay, 0, client)
        on(client)
        move_to(400, 410, 10, client)
        off(client)

    result = get_cords(get_frame(0))
    to_move = result[0]

    ax, ay, a_ang = to_move[70][0], to_move[70][1], to_move[70][2]
    bring_and_drop(ax, ay, 325, 325, a_ang, client)

    ax, ay, a_ang = to_move[50][0], to_move[50][1], to_move[50][2]
    bring_and_drop(ax, ay, 325, 325, a_ang, client)

    ax, ay, a_ang = to_move[60][0], to_move[60][1], to_move[60][2]
    bring_and_drop(ax, ay, 25, 325, a_ang, client)

    ax, ay, a_ang = to_move[40][0], to_move[40][1], to_move[40][2]
    bring_and_drop(ax, ay, 25, 325, a_ang, client)

    ax, ay, a_ang = to_move[80][0], to_move[80][1], to_move[80][2]
    bring_and_drop(ax, ay, 175, 175, a_ang, client)

    ax, ay, a_ang = to_move[50][0], to_move[50][1], 0
    bring_and_drop(ax, ay, 325, 175, a_ang, client)

    ax, ay, a_ang = to_move[40][0], to_move[40][1], 0
    bring_and_drop(ax, ay, 25, 175, a_ang, client)

    ax, ay, a_ang = to_move[70][0], to_move[70][1], 0
    bring_and_drop(ax, ay, 175, 175, a_ang, client)

    ax, ay, a_ang = to_move[60][0], to_move[60][1], 0
    bring_and_drop(ax, ay, 175, 175, a_ang, client)

    ax, ay, a_ang = to_move[50][0], to_move[50][1], 0
    bring_and_drop(ax, ay, 175, 175, a_ang, client)

    ax, ay, a_ang = to_move[40][0], to_move[40][1], 0
    bring_and_drop(ax, ay, 175, 175, a_ang, client)

except RuntimeError:
    print("[CLIENT] Сервер занят, завершаем работу.")
finally:
    client.close()
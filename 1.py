import socket
import threading
import serial
import serial.tools.list_ports
import time

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

HOST = '0.0.0.0'  # Сервер слушает на всех сетевых интерфейсах
PORT = 5000  # Порт сервера

# Глобальные переменные для управления занятостью
busy = False
current_client = None
last_command_time = 0
INACTIVITY_TIMEOUT = 30  # 30 секунд


# --------------------------------------------------------
# Блок работы с Arduino
# --------------------------------------------------------
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200, timeout=1)
            print(port.device)
            time.sleep(2)  # Ждём, пока Arduino перезагрузится
            ser.flushInput()
            ser.write(b'PING\n')  # Отправляем команду PING
            print(1)
            response = ser.readline().decode().strip()
            print(response)
            if response == 'PONG':
                print(f"Arduino обнаружена на {port.device}")
                return ser
            ser.close()
        except serial.SerialException:
            continue
    print("Arduino не найдена")
    return None


ser = find_arduino_port()
if ser is None:
    exit()

# Для гарантированной последовательности обращений к ser введём блокировку
serial_lock = threading.Lock()


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def transform_command(command):
    """
    Текущая логика трансформации: для MOVE_TO / MOVE_TO_ROBOT
    меняем координаты, как раньше.
    """
    parts = command.strip().split()

    # Пример: "MOVE_TO x y z" или "MOVE_TO_ROBOT x y z"
    if len(parts) == 4 and (parts[0] in ("MOVE_TO", "MOVE_TO_ROBOT")):
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            return "Invalid coordinate values"

        if parts[0] == "MOVE_TO_ROBOT":
            # Координаты робота напрямую
            x_new = x
            y_new = y
            z_new = z
        else:
            # MOVE_TO: меняем X и Y местами + смещения
            x_new = -y + 552
            y_new = -x + 268
            z_new = z - 3
        # Собираем итоговую строку
        return f"{parts[0]} {int(x_new)} {int(y_new)} {int(z_new)}"
    if parts[0] == "TOOL_ROTATE_TO":
        try:
            angle = float(parts[1])
            v = arduino_map(angle, 90, 0, 45, 142)
            return f"{parts[0]} {int(angle)}"

        except ValueError:
            return "Invalid angle values"

    # Если это не MOVE_TO* - возвращаем исходную команду как есть.
    return command


def send_to_arduino_and_get_response(command):
    """
    Отправка команды в Arduino с последующим чтением
    единственной строки-ответа.
    """
    print(f"[SERVER] Принята команда: {command}")

    # Для MOVE_TO / MOVE_TO_ROBOT трансформируем
    if command.startswith("MOVE_TO"):
        command = transform_command(command)

    with serial_lock:
        ser.write((command + '\n').encode())
        while True:
            response = ser.readline().decode().strip()
            if response:
                print(f"[SERVER] Ответ от Arduino: {response}")
                return response
            else:
                time.sleep(0.1)


# --------------------------------------------------------
# Блок работы с сокетами (серверная часть)
# --------------------------------------------------------
def handle_client_connection(conn, addr):
    """
    Потоковая функция для обслуживания одного клиента.
    Сервер находится в состоянии "busy", пока этот поток активен.
    Если 30 секунд нет команд — разрываем соединение и освобождаем сервер.
    """
    global busy, current_client, last_command_time

    print(f"[SERVER] Подключился клиент: {addr}")

    # Чтобы ловить таймаут без блокировки recv, используем таймер сокета
    conn.settimeout(1.0)  # Каждую секунду будем "пробуждаться" и проверять таймер

    last_command_time = time.time()  # Запоминаем, когда клиент подключился

    try:
        while True:
            # Каждую итерацию пытаемся читать данные:
            # если нет данных в течение 1 секунды -> ловим socket.timeout
            try:
                data = conn.recv(1024)
            except socket.timeout:
                # Проверяем, не вышло ли 30 сек. (INACTIVITY_TIMEOUT)
                if time.time() - last_command_time > INACTIVITY_TIMEOUT:
                    print(f"[SERVER] Нет активности от {addr} более {INACTIVITY_TIMEOUT} секунд. Закрываем соединение.")
                    break
                else:
                    # Иначе просто продолжаем ждать
                    continue

            if not data:
                # Клиент закрыл соединение
                break

            command = data.decode().strip()
            if not command:
                continue

            # Обновляем время последней активности
            last_command_time = time.time()

            # Обрабатываем команду и получаем ответ
            response = send_to_arduino_and_get_response(command)

            # Отправляем ответ обратно клиенту
            conn.sendall((response + '\n').encode())

    except ConnectionError:
        print(f"[SERVER] Потеряно соединение с {addr}")
    finally:
        conn.close()
        print(f"[SERVER] Соединение закрыто: {addr}")
        busy = False
        current_client = None
        print("[SERVER] Сервер снова в состоянии ожидания (busy=False).")


def start_server():
    global busy, current_client

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"[SERVER] Сервер запущен на {HOST}:{PORT}, ожидаем подключений...")

    try:
        while True:
            conn, addr = server_socket.accept()
            # Проверяем, не занят ли сервер
            if busy:
                # Сервер занят — отправим ответ и закроем соединение
                print(f"[SERVER] Новый клиент {addr}, но сервер занят. Отклоняем.")
                conn.sendall(b"BUSY\n")
                conn.close()
            else:
                # Сервер свободен — принимаем соединение
                busy = True
                current_client = (conn, addr)
                client_thread = threading.Thread(
                    target=handle_client_connection,
                    args=(conn, addr),
                    daemon=True
                )
                client_thread.start()

    except KeyboardInterrupt:
        print("[SERVER] Остановка сервера (Ctrl+C)")
    finally:
        server_socket.close()
        print("[SERVER] Сервер завершил работу.")





if __name__ == "__main__":
    start_server()

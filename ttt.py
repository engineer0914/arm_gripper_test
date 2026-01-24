'''

ls -l /dev/ttyACM*

'''

import serial
import time

PORT = "/dev/ttyACM1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Connected to OpenCR")
print("Type grip = GRIP, open = OPEN, q = quit")


def wait_grasp_result(ser, timeout=5.0):
    start = time.time()

    while time.time() - start < timeout:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()

            if "[RESULT] GRASP_OK" in line:
                print("[OpenCR]", line)
                return True

            elif "[RESULT] GRASP_FAIL" in line:
                print("[OpenCR]", line)
                return False

        time.sleep(0.01)

    print("⚠️ [Python] Grasp result timeout")
    return False


while True:
    cmd = input(">> ").strip().lower()

    if cmd == "grip":
        ser.write(b'grip\n')
        print("Sent: grip")

        success = wait_grasp_result(ser)

        if success:
            print("✅ Python 판단: 파지 성공")
        else:
            print("❌ Python 판단: 파지 실패")

    elif cmd == "open":
        ser.write(b'open\n')
        print("Sent: open")

    elif cmd == "q":
        break

    # 일반 로그 필터 (원하면 유지)
    time.sleep(0.1)
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if (
            "[GRASP]" in line
            or "[OPEN]" in line
            or "[Init]" in line
            or "[RX]" in line
        ):
            print("[OpenCR]", line)

ser.close()
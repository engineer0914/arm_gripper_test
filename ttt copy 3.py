"""

이코드는 로봇 암과 그리퍼(OpenCR 기반)를 시리얼 통신으로 제어하여, 특정 위치로 접근한 후 그리퍼를 작동시키고 결과를 확인하는 기능을 포함합니다.
현재 위치에서 100mm 접근후 그리퍼를 작동시키고, 성공 여부에 따라 결과를 출력한 후 원위치로 복귀합니다.

"""



import rbpodo as rb
import numpy as np
import time
import serial  # 시리얼 통신 라이브러리 추가

# ====== 설정 변수 ======
ROBOT_IP_FILE = "IP_robotarm.txt"
GRIPPER_PORT = "/dev/ttyACM1" # 포트 확인 필요 (ls -l /dev/ttyACM*)
GRIPPER_BAUD = 115200

# ====== 로봇 IP 읽기 ======
def read_robot_ip(filename):
    try:
        with open(filename, 'r') as file:
            return file.read().strip()
    except Exception as e:
        print(f"⚠️ IP 파일 읽기 오류: {e}")
        return None

# ====== 그리퍼 응답 대기 함수 ======
def wait_grasp_result(ser, timeout=3.0):
    """
    OpenCR로부터 그립 결과를 기다립니다.
    [RESULT] GRASP_OK -> True 반환
    [RESULT] GRASP_FAIL -> False 반환
    """
    start_time = time.time()
    buffer = ""
    
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            try:
                # 데이터 읽기 (디코딩 에러 무시)
                line = ser.readline().decode(errors="ignore").strip()
                
                # 디버깅용 로그 (필요 시 주석 해제)
                # print(f"[OpenCR Raw] {line}")

                if "[RESULT] GRASP_OK" in line:
                    return True, line
                elif "[RESULT] GRASP_FAIL" in line:
                    return False, line
            except Exception as e:
                print(f"Serial Read Error: {e}")
                
        time.sleep(0.01)
    
    return False, "Timeout"

# ====== 메인 루틴 ======
def _main():
    # 1. 로봇 및 시리얼 연결 설정
    robot_ip = read_robot_ip(ROBOT_IP_FILE)
    if not robot_ip:
        print("❌ 로봇 IP를 찾을 수 없습니다.")
        return

    print(f"✅ 로봇 IP: {robot_ip}")
    print(f"✅ 그리퍼 포트 연결 시도: {GRIPPER_PORT}...")

    # 시리얼 연결 (그리퍼)
    try:
        ser = serial.Serial(GRIPPER_PORT, GRIPPER_BAUD, timeout=1)
        time.sleep(2) # 아두이노/OpenCR 리셋 대기
        print("✅ 그리퍼(OpenCR) 연결 성공")
    except Exception as e:
        print(f"❌ 그리퍼 연결 실패: {e}")
        return

    # 로봇 연결
    try:
        robot = rb.Cobot(robot_ip)
        rc = rb.ResponseCollector()
        robot.set_operation_mode(rc, rb.OperationMode.Real)
        robot.set_speed_bar(rc, 0.5)
        print("✅ 로봇 연결 성공")
    except Exception as e:
        print(f"❌ 로봇 연결 실패: {e}")
        ser.close()
        return

    # ==========================================
    # 🚀 [핵심 요청 기능] 접근 -> 파지 -> 결과출력 -> 복귀
    # ==========================================
    
    # 테스트 설정
    approach_axis = np.array([0, 0, 100, 0, 0, 0]) # Z축 방향으로 100mm 이동 (상황에 따라 [100, 0, 0...] 등으로 변경)
    vel = 200
    acc = 400

    print("\n========================================")
    print("🚀 [시나리오] 접근 -> 그립(피드백) -> 복귀 테스트 시작")
    print("========================================\n")

    try:
        # 1️⃣ 접근 (Move Forward)
        print(f"➡️ 1. 목표 지점으로 100mm 접근 중... {approach_axis}")
        robot.move_l_rel(rc, approach_axis, vel, acc, rb.ReferenceFrame.Tool)
        
        # 이동 시작 및 완료 대기
        if robot.wait_for_move_started(rc, 0.5).is_success():
            robot.wait_for_move_finished(rc)
        
        # 2️⃣ 그립 명령 전송 (Grip)
        print("✊ 2. 그리퍼 'grip' 명령 전송...")
        ser.write(b'grip\n')
        
        # 3️⃣ 결과 대기 및 판별 (Check Result)
        success, msg = wait_grasp_result(ser, timeout=5.0)
        
        print("-" * 20)
        if success:
            print("✅ 성공 (Grasp OK)")
        else:
            print(f"❌ 실패 (Reason: {msg})")
        print("-" * 20)
        
        # 결과를 육안으로 확인할 시간 조금 대기
        time.sleep(1.0) 

        # 4️⃣ 복귀 (Move Backward) - 결과와 상관없이 수행
        print(f"⬅️ 3. 원위치로 100mm 복귀 중...")
        robot.move_l_rel(rc, -approach_axis, vel, acc, rb.ReferenceFrame.Tool)
        
        if robot.wait_for_move_started(rc, 0.5).is_success():
            robot.wait_for_move_finished(rc)

        print("\n✅ 시나리오 종료.")

        # 테스트 종료 후 그리퍼 놓기 (옵션)
        # ser.write(b'open\n')

    except Exception as e:
        print(f"⚠️ 동작 중 오류 발생: {e}")
        robot.stop(rc)

    finally:
        ser.close()
        print("🔌 연결 종료")

if __name__ == "__main__":
    _main()
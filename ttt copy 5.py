"""


#로봇팔 처음 직각뷰
#로봇팔
84.433, 7.489, -100.911, 5.396, -77.776, 89.251

1번 위치로 이동
#로봇팔

119.93, -74.337, -65.468, 5.429, -39.154, 90.093


직선 100 mm 전진

그립 상태 확인후 후퇴

직선 100mm후진



2번 위치로 이동
뒤집어서 화물 두기
156.568, -26.226, -64.151, -17.524, -86.465, 149.291

140.693, -19.038, -54.949, -181.03, -71.698, 213.36

174.247, 30.168, -102.519, -273.759, -95.278, 194.636

250.992, 10.65, -74.101, -351.343, -109.729, 241.215




놓기

직선 후진


2차 위치 이동

248.922, 2.197, -12.322, -323.103, -76.941, 241.179

그립

261.083, 13.651, -22.047, -342.81, 10.964, 241.18

261.104, -1.539, 57.927, -344.944, 44.739, 241.18

245.043, 68.932, 75.329, -350.226, 32.002, 241.18




다시 원점으로 복귀


158.455, 15.715, -65.223, -260.597, -106.016, 222.073


169.582, 26.334, -74.685, -198.129, 0.926, 218.054



139.174, 6.409, -58.681, -224.157, 76.581, 275.082



95.289, 7.554, -94.856, -185.734, 84.04, 279.56


"""




import rbpodo as rb
import numpy as np
import time
import serial

# ====== 설정 변수 ======
ROBOT_IP_FILE = "IP_robotarm.txt"
GRIPPER_PORT = "/dev/ttyACM1"  # 포트 확인 (ls -l /dev/ttyACM*)
GRIPPER_BAUD = 115200

# 속도/가속도 설정
VEL_J, ACC_J = 70, 70    # 관절 이동 (move_j) 속도
VEL_L, ACC_L = 200, 400    # 직선 이동 (move_l) 속도

# ====== 로봇 IP 읽기 ======
def read_robot_ip(filename):
    try:
        with open(filename, 'r') as file:
            return file.read().strip()
    except:
        return None

# ====== 그리퍼 통신 함수 ======
def send_gripper_cmd(ser, cmd):
    """그리퍼에 명령 전송 (grip/open)"""
    command = f"{cmd}\n".encode()
    ser.write(command)
    print(f"📤 그리퍼 명령 전송: {cmd}")

def wait_grasp_result(ser, timeout=5.0):
    """그리퍼 파지 결과 대기"""
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if "[RESULT] GRASP_OK" in line:
                    return True, line
                elif "[RESULT] GRASP_FAIL" in line:
                    return False, line
            except:
                pass
        time.sleep(0.01)
    return False, "Timeout"

# ====== 로봇 이동 래퍼 함수 ======
def move_j_safe(robot, rc, joint_deg, desc=""):
    """관절 이동 (Move J)"""
    print(f"🦾 [이동] {desc} ...")
    robot.move_j(rc, joint_deg, VEL_J, ACC_J)
    if robot.wait_for_move_started(rc, 0.5).is_success():
        robot.wait_for_move_finished(rc)
    rc.error().throw_if_not_empty()

def move_l_rel_safe(robot, rc, offset_mm, desc=""):
    """툴 기준 직선 상대 이동 (Move L Rel)"""
    print(f"📏 [직선] {desc} ({offset_mm})")
    target = np.array([offset_mm[0], offset_mm[1], offset_mm[2], 0, 0, 0])
    robot.move_l_rel(rc, target, VEL_L, ACC_L, rb.ReferenceFrame.Tool)
    if robot.wait_for_move_started(rc, 0.5).is_success():
        robot.wait_for_move_finished(rc)
    rc.error().throw_if_not_empty()

# ====== 메인 루틴 ======
def _main():
    # 1. 좌표 데이터 정의 (Numpy Array 변환)
    pos_start    = np.array([84.433, 7.489, -100.911, 5.396, -77.776, 89.251])
    
    # 1번 위치 (집는 곳)
    pos_pick     = np.array([119.93, -74.337, -65.468, 5.429, -39.154, 90.093])
    
    # 운반 경로 (Waypoints)
    pos_way1     = np.array([156.568, -26.226, -64.151, -17.524, -86.465, 149.291])
    pos_way2     = np.array([140.693, -19.038, -54.949, -181.03, -71.698, 213.36])
    pos_way3     = np.array([174.246, 37.718, -88.715, -273.758, -95.273, 194.643])

    # 2번 위치 (놓는 곳)
    pos_place    = np.array([250.586, 7.637, -73.343, -351.029, -116.947, 241.192])



    pos_2ret1     = np.array([248.922, 2.197, -12.322, -323.103, -76.941, 241.179])

    # grip

    pos_2ret2     = np.array([261.083, 13.651, -22.047, -342.81, 10.964, 241.18])
    pos_2ret3     = np.array([261.104, -1.539, 57.927, -344.944, 44.739, 241.18])



    pos_2ret4    = np.array([245.064, 68.848, 77.411, -351.52, 32.143, 241.171])


    # 복귀 경로
    pos_ret1     = np.array([158.455, 15.715, -65.223, -260.597, -106.016, 222.073])
    pos_ret2     = np.array([169.582, 26.334, -74.685, -198.129, 0.926, 218.054])
    pos_ret3     = np.array([139.174, 6.409, -58.681, -224.157, 76.581, 275.082])
    pos_final    = np.array([95.289, 7.554, -94.856, -185.734, 84.04, 279.56])

    # 2. 연결 설정
    robot_ip = read_robot_ip(ROBOT_IP_FILE)
    if not robot_ip: return

    try:
        print(f"🔌 장치 연결 중... (Robot: {robot_ip}, Gripper: {GRIPPER_PORT})")
        ser = serial.Serial(GRIPPER_PORT, GRIPPER_BAUD, timeout=1)
        robot = rb.Cobot(robot_ip)
        rc = rb.ResponseCollector()
        robot.set_operation_mode(rc, rb.OperationMode.Real)
        # robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        robot.set_speed_bar(rc, 0.5)
        time.sleep(2) # 아두이노 리셋 대기
        print("✅ 모든 장치 연결 완료.\n")
    except Exception as e:
        print(f"❌ 연결 오류: {e}")
        return



    try:
        # ================= STEP 0: 초기 위치 =================
        move_j_safe(robot, rc, pos_start, "초기 직각뷰 위치로 이동")

        # ================= STEP 1: 1번 위치로 이동 및 파지 =================
        move_j_safe(robot, rc, pos_pick, "1번 위치(Pick)로 이동")
        
        # # 1-1. 직선 전진 (100mm)
        # move_l_rel_safe(robot, rc, [0, 0, 100], "접근 (Z+ 100mm)")

        # 1-2. 그리퍼 집기
        send_gripper_cmd(ser, "grip")
        success, msg = wait_grasp_result(ser)
        if success:
            print("✅ 파지 성공 (Grasp OK)")
        else:
            print(f"❌ 파지 실패 ({msg}) - 계속 진행합니다.")
        time.sleep(0.5)

        # 1-3. 직선 후퇴 (-100mm)
        move_l_rel_safe(robot, rc, [0, 0, -100], "후퇴 (Z- 100mm)")


        # ================= STEP 2: 2번 위치로 이동 (운반/뒤집기) =================
        move_j_safe(robot, rc, pos_way1, "경유지 1")
        move_j_safe(robot, rc, pos_way2, "경유지 2 (회전)")
        move_j_safe(robot, rc, pos_way3, "경유지 3")

        move_j_safe(robot, rc, pos_place, "2번 위치(Place) 도착")

        # 2-2. 그리퍼 놓기
        send_gripper_cmd(ser, "open")
        time.sleep(1.0) # 완전히 놓을 시간 대기
        print("✅ 물체 놓기 완료")

        # 1-2. 그리퍼 집기
        send_gripper_cmd(ser, "grip")
        success, msg = wait_grasp_result(ser)
        if success:
            print("✅ 파지 성공 (Grasp OK)")
        else:
            print(f"❌ 파지 실패 ({msg}) - 계속 진행합니다.")
        time.sleep(0.5)


        move_j_safe(robot, rc, pos_2ret1, "..")
        move_j_safe(robot, rc, pos_2ret2, "..")
        move_j_safe(robot, rc, pos_2ret3, "..")


        move_j_safe(robot, rc, pos_2ret4, "..") # 도착점

        # 2-2. 그리퍼 놓기
        send_gripper_cmd(ser, "open")
        time.sleep(1.0) # 완전히 놓을 시간 대기
        print("✅ 물체 놓기 완료")

        # 1-2. 그리퍼 집기
        send_gripper_cmd(ser, "grip")
        success, msg = wait_grasp_result(ser)
        if success:
            print("✅ 파지 성공 (Grasp OK)")
        else:
            print(f"❌ 파지 실패 ({msg}) - 계속 진행합니다.")
        time.sleep(0.5)


        move_j_safe(robot, rc, pos_2ret3, "..")

        move_j_safe(robot, rc, pos_2ret2, "..")

        move_j_safe(robot, rc, pos_2ret1, "..")

        move_j_safe(robot, rc, pos_place, "2번 위치(Place) 도착")



        # 2-2. 그리퍼 놓기
        send_gripper_cmd(ser, "open")
        time.sleep(1.0) # 완전히 놓을 시간 대기
        print("✅ 물체 놓기 완료")

        # 1-2. 그리퍼 집기
        send_gripper_cmd(ser, "grip")
        success, msg = wait_grasp_result(ser)
        if success:
            print("✅ 파지 성공 (Grasp OK)")
        else:
            print(f"❌ 파지 실패 ({msg}) - 계속 진행합니다.")
        time.sleep(0.5)


        move_j_safe(robot, rc, pos_way3, "경유지 3")

        move_j_safe(robot, rc, pos_way2, "경유지 2 (회전)")

        move_j_safe(robot, rc, pos_way1, "경유지 1")

        move_j_safe(robot, rc, pos_pick, "1번 위치(Pick)로 이동")


        # 2-2. 그리퍼 놓기
        send_gripper_cmd(ser, "open")
        time.sleep(1.0) # 완전히 놓을 시간 대기
        print("✅ 물체 놓기 완료")

        # # 1-2. 그리퍼 집기
        # send_gripper_cmd(ser, "grip")
        # success, msg = wait_grasp_result(ser)
        # if success:
        #     print("✅ 파지 성공 (Grasp OK)")
        # else:
        #     print(f"❌ 파지 실패 ({msg}) - 계속 진행합니다.")
        # time.sleep(0.5)




        print("\n🎉 모든 시나리오 종료.")

    except Exception as e:
        print(f"\n⚠️ 동작 중 오류 발생: {e}")
        robot.stop(rc)
    
    finally:
        ser.close()
        print("🔌 연결 해제")

if __name__ == "__main__":
    _main()
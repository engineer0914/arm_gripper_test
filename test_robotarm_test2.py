# 다음의 테스트를 하는 코드 입니다.
# 1.컨트롤 박스 정보 제공
# 2.각 관절별 이동
# 3.툴플랜지 기준 선형 이동
# 4.베이스 기준 선형 이동
# 5.그리퍼 테스트

import rbpodo as rb
import numpy as np
import time

# ====== 로봇 IP 읽기 ======
def read_robot_ip(filename="IP_robotarm.txt"):
    """IP_robotarm.txt 파일에서 로봇 IP 주소를 읽어옵니다."""
    try:
        with open(filename, 'r') as file:
            ip_address = file.read().strip()
            return ip_address
    except FileNotFoundError:
        print(f"Error: {filename} 파일을 찾을 수 없습니다.")
        return None
    except Exception as e:
        print(f"파일 읽기 오류: {e}")
        return None


# ====== 그리퍼 제어 함수 ======
def grip(command, robot, rc):
    """그리퍼 제어: 'grab' 또는 'release'"""
    try:
        if command == "release":
            print("🔹 그리퍼: 릴리즈 동작")
            robot.set_dout_bit_combination(rc, 0, 3, 1, rb.Endian.LittleEndian)
            time.sleep(0.1)
            robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.LittleEndian)

        elif command == "grab":
            print("🔹 그리퍼: 집기 동작")
            robot.set_dout_bit_combination(rc, 0, 3, 2, rb.Endian.LittleEndian)
            time.sleep(0.1)
            robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.LittleEndian)
    except Exception as e:
        print(f"그리퍼 제어 오류: {e}")


# ====== 메인 루틴 ======
def _main():
    # 파일에서 IP 주소 읽기
    robot_ip = read_robot_ip()
    if robot_ip is None:
        print("로봇 IP 주소를 읽을 수 없어 프로그램을 종료합니다.")
        return

    print(f"\n✅ 로봇 IP: {robot_ip}")
    ROBOT_IP = robot_ip

    # 로봇 연결
    robot = rb.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()
    
    # 모드 및 속도 설정
    robot.set_operation_mode(rc, rb.OperationMode.Real)
    robot.set_speed_bar(rc, 0.5)

    print(f"------------------------------------")
    print(f"\n1️⃣ 컨트롤 박스 정보\n")

    res, cb_info = robot.get_control_box_info(rc)
    if res.is_success():
        print(f"컨트롤 박스 정보: {cb_info}\n")

    # 조인트 각도 읽기
    joint_angles = []
    for i in range(6):  # J0 ~ J5
        _, out = robot.get_system_variable(rc, getattr(rb.SystemVariable, f"SD_J{i}_ANG"))
        rc = rc.error().throw_if_not_empty()
        joint_angles.append(out)

    joint_array = np.array(joint_angles, dtype=float)
    print(f"현재 조인트 각도: {joint_array}\n")

    # 테스트 파라미터
    step = 5
    acc = 200
    vel = 200

    # ====== 각 조인트 순차 테스트 ======
    for i in range(6):
        try:
            print(f"\n=== 🔸 Joint {i} 테스트 시작 ===")

            robot.flush(rc)
            original_pos = joint_array.copy()

            # 1️⃣ -3도 이동
            down_pos = original_pos.copy()
            down_pos[i] -= 3
            print(f"Joint {i} -{step}° -3도 이동 중...")
            robot.move_j(rc, down_pos, vel, acc)
            if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()

            # 2️⃣ +3도 이동 (원위치로 복귀)
            up_pos = original_pos.copy()
            up_pos[i] += 3
            print(f"Joint {i} +{step}° +3도 이동 중...")
            robot.move_j(rc, up_pos, vel, acc)
            if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()

            # 3️⃣ 다시 원래 위치로 복귀
            print(f"Joint {i} 원위치 복귀 중...")
            robot.move_j(rc, original_pos, vel, acc)
            if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()

        except Exception as e:
            print(f"⚠️ 로봇 제어 오류 (Joint {i}): {e}")
            try:
                robot.stop(rc)
            except:
                pass

    print("\n✅ 모든 조인트 테스트 완료.")



    # ====== 툴플랜지 선형 움직임 테스트 ======

        # 이동할 목표점들 (상대 이동)
    target_points = [
            np.array([100, 0, 0, 0, 0, 0]),   # X+
            np.array([0, 100, 0, 0, 0, 0]),   # Y+
            np.array([0, 0, 100, 0, 0, 0])   # Z+
        ]

    acc = 400
    vel = 300

    try:
        print("\n=== 🔸 툴플랜지 선형 움직임 테스트 시작 ===")

        directions = ["X+", "Y+", "Z+"]

        for idx, target_point in enumerate(target_points):
            print(f"\n➡️ {directions[idx]} 방향으로 이동 중: {target_point}")
            robot.move_l_rel(rc, target_point, vel, acc, rb.ReferenceFrame.Tool)
            if robot.wait_for_move_started(rc, 0.5).is_success():
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()

            # 원위치 복귀 (반대 방향으로 이동)
            print(f"⬅️ {directions[idx]} 방향 원위치 복귀 중...")
            robot.move_l_rel(rc, -target_point, vel, acc, rb.ReferenceFrame.Tool)
            if robot.wait_for_move_started(rc, 0.5).is_success():
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()
            

        print("\n✅ 툴플랜지 선형 움직임 테스트 완료.")
        
    except Exception as e:
        print(f"⚠️ 툴플랜지 이동 오류: {e}")
        try:
            robot.stop(rc)
        except:
            pass

    # ====== 베이스 기준 선형 움직임 테스트 ======
    try:
        print("\n=== 🔸 툴플랜지 선형 움직임 테스트 시작 ===")

        directions = ["X+", "Y+", "Z+"]

        for idx, target_point in enumerate(target_points):
            print(f"\n➡️ {directions[idx]} 방향으로 이동 중: {target_point}")
            robot.move_l_rel(rc, target_point, vel, acc, rb.ReferenceFrame.Base)
            if robot.wait_for_move_started(rc, 0.5).is_success():
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()
            
            # 원위치 복귀 (반대 방향으로 이동)
            print(f"⬅️ {directions[idx]} 방향 원위치 복귀 중...")
            robot.move_l_rel(rc, -target_point, vel, acc, rb.ReferenceFrame.Base)
            if robot.wait_for_move_started(rc, 0.5).is_success():
                robot.wait_for_move_finished(rc)
            rc.error().throw_if_not_empty()

        print("\n✅ 툴플랜지 선형 움직임 테스트 완료.")


    except Exception as e:
        print(f"⚠️ 툴플랜지 이동 오류: {e}")
        try:
            robot.stop(rc)
        except:
            pass

    # ====== 공압 그리퍼 테스트 ======
    try:
        print("\n=== 🔸 공압 그리퍼 테스트 시작 ===")
        for j in range(3):
            print(f"  ➜ 그리퍼 테스트 {j+1}/3")
            grip("grab", robot, rc)
            time.sleep(0.5)
            grip("release", robot, rc)
            time.sleep(0.5)

        print("\n✅ 공압 동작 테스트 완료.")

        

    except Exception as e:
        print(f"⚠️ 그리퍼 제어 오류: {e}")
        try:
            robot.stop(rc)
        except:
            pass


if __name__ == "__main__":
    _main()

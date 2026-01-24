"""

IP TXT에 지정된 ip를 참고하여 출력함
로봇팔의 현재 각도를 읽고 터미널에 출력하는 스크립트입니다.



"""


import rbpodo as rb
import time

# ====== 로봇 IP 읽기 ======
def read_robot_ip(filename="IP_robotarm.txt"):
    try:
        with open(filename, 'r') as file:
            return file.read().strip()
    except Exception as e:
        print(f"IP 파일 읽기 오류: {e}")
        return None

def _main():
    # 1. IP 설정 및 연결
    robot_ip = read_robot_ip()
    if robot_ip is None:
        print("IP 파일을 찾을 수 없습니다.")
        return

    ROBOT_IP = robot_ip
    robot = rb.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()
    
    # 연결 모드 설정
    robot.set_operation_mode(rc, rb.OperationMode.Real)

    # 2. 조인트 각도 읽기 (J0 ~ J5)
    joint_angles = []
    print("\n데이터 읽는 중...")

    for i in range(6): 
        # get_system_variable은 (Response, Value) 튜플을 반환합니다.
        res, out = robot.get_system_variable(rc, getattr(rb.SystemVariable, f"SD_J{i}_ANG"))
        
        # 수정됨: res.is_success()를 사용하여 성공 여부 확인
        if res.is_success() and out is not None:
            joint_angles.append(out)
        else:
            print(f"Error reading Joint {i}")
            # 에러가 있다면 내용을 출력하고 종료 (선택 사항)
            rc.error().throw_if_not_empty() 
            return

    # 3. 쉼표로 구분하여 터미널 출력
    # map(str, joint_angles)를 사용하여 숫자 리스트를 문자열로 변환 후 join
    output_string = ", ".join(map(str, joint_angles))
    
    print("\n=== 현재 조인트 각도 (J0, J1, J2, J3, J4, J5) ===")
    print(output_string)

if __name__ == "__main__":
    _main()









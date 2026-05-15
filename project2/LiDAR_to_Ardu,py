# Practice5.py 참고

import serial
import time

port_L = "/dev/ttyUSB0" # USB-시리얼 포트 - LiDAR 연결된 포트로 설정
port_Ardu = "/dev/ttyS0" # USB-시리얼 포트 - 아두이노 연결된 포트로 설정
baudrate_L = 460800 # 보드레이트 적용
baudrate_Ardu = 460800 # 보드레이트 적용
ser_L = serial.Serial(port_L, baudrate_L, timeout=1) # LiDAR 시리얼 포트와 보드레이트 설정 -> 460800 bps
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1) # 라즈베리파이의 시리얼 포트와 보드레이트 설정 -> 115200 bps

# RESET 요청 패킷 전송 (0xA5 0x40)
scan_request = bytes([0xA5, 0x40])
ser_L.write(scan_request)
time.sleep(1) # 1초 동안 멈춤

# SCAN 요청 패킷 전송 (0xA5 0x20)
scan_request = bytes([0xA5, 0x20])
ser_L.write(scan_request)

# 응답 데이터 읽기
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # Start Flag와 Inversed Start Flag 검증
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue

    # Check Bit 검증
    check_bit = data[1] & 0x01
    if check_bit != 1:
        continue

    # 품질
    quality = data[0] >> 2

    # 각도 계산
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0 #각도

    # 거리 계산
    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0  # 거리mm
    if distance < 50:  # 거리가 너무 짧은 경우 노이즈로 간주하고 무시
        continue  
    
    
# --- 수정 필요 : 장애물 회피 로직 추가 ---

    # ── 첫 루프 진입 시 1회만 초기화 ───
    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []        # 1회전 포인트 누적 버퍼
        front_min  = 9999.0    # 전방 최소 거리 (mm)
        left_min   = 9999.0    # 좌측 최소 거리 (mm)
        right_min  = 9999.0    # 우측 최소 거리 (mm)
        left_cnt   = 0         # 왼쪽 구역 유효 포인트 수
        right_cnt  = 0         # 오른쪽 구역 유효 포인트 수
        MIN_COUNT  = 3         # 최소 포인트 수 (미만이면 노이즈로 무시)
        EMERGENCY  = 130.0     # 즉시 후진 거리 (mm)
        DETECT     = 210.0     # 장애물 감지 거리 (mm)
        def _cleanup():
            try:
                ser_Ardu.write(b"S\n")
                ser_L.write(bytes([0xA5, 0x25]))
                time.sleep(0.1)
                ser_L.close()
                ser_Ardu.close()
            except Exception:
                pass
        atexit.register(_cleanup)
        print("=" * 50)
        print("  장애물 회피 자동차 시작  (Ctrl+C 로 종료)")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print("=" * 50)

    # ── 품질 필터: quality==0 은 노이즈 포인트 → 스킵 ───
    if quality == 0:
        continue

    # ── 포인트를 구역별 최솟값 및 카운트에 반영 ───
    # 전방 구역: ±30°
    if angle <= 40 or angle >= 320:
        front_min = min(front_min, distance)

    # 오른쪽 구역 (CW 기준: 0°=앞, 90°=우 → 30~90°)
    elif (angle > 40 and angle < 90) and distance <= 210:
        right_min = min(right_min, distance)
        right_cnt += 1

    # 왼쪽 구역 (CW 기준: 270°=좌 → 270~330°)
    elif (angle > 270 and angle < 320) and distance <= 210:
        left_min = min(left_min, distance)
        left_cnt += 1

    scan_buf.append((angle, distance))

    # ── 새 회전 시작(s_flag==1) → 1회전치 데이터로 판단 및 명령 전송 ──
    if s_flag == 1 and len(scan_buf) > 10:

        # 최소 포인트 수 미만 구역은 노이즈로 간주하여 거리 무효화
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0

        # 왼쪽 장애물 회피 ───
        if left_min <= DETECT:
            print(f"Left  obstacle  {left_min:.0f}mm (pts:{left_cnt})")
            if left_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.50\n")
            else:
                ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
                steer = ratio * 0.95
                speed = 0.70 * (1 - ratio * 0.6)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())

        # 오른쪽 장애물 회피 ───
        elif right_min <= DETECT:
            print(f"Right obstacle  {right_min:.0f}mm (pts:{right_cnt})")
            if right_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.50\n")
            else:
                ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
                steer = -ratio * 0.95
                speed = 0.70 * (1 - ratio * 0.6)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())

        # 전방 장애물 회피 ───
        elif front_min <= DETECT:
            if front_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.50\n")
                print(f"Front EMERGENCY! {front_min:.0f}mm → 후진")
            else:
                ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
                speed = 0.70 * (1 - ratio * 0.7)
                steer = 0.50 if right_min > left_min else -0.50
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"Front obstacle  {front_min:.0f}mm → {'우' if steer>0 else '좌'}  spd={speed:.2f}")

        # 장애물 없음 → 직진 ───
        else:
            ser_Ardu.write(b"F 0.00 0.70\n") 

        # 버퍼 및 구역 값 초기화
        scan_buf  = []
        front_min = 9999.0
        left_min  = 9999.0
        right_min = 9999.0
        left_cnt  = 0
        right_cnt = 0

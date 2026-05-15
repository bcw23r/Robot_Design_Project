import serial
import time
import math

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue
    if (data[1] & 0x01) != 1:
        continue

    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    if distance < 80:
        continue

    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []
        EMERGENCY  = 140.0     # 즉시 후진 거리 (mm)
        DETECT     = 350.0     # 장애물 감지 거리 (mm)
        MAX_STEER  = 0.85      # 최대 조향값
        REAR_MIN   = 150.0     # 후방 제외 시작 각도 (°)
        REAR_MAX   = 210.0     # 후방 제외 끝 각도 (°)
        FRONT_DEAD = 0.08      # sin 값이 이 이하이면 정면 장애물로 판단
        back_cnt   = 0
        extra_back = 0

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
        print("=" * 55)
        print("  장애물 회피 [최근접 장애물 기준] 시작  (Ctrl+C 종료)")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  후방제외: {int(REAR_MIN)}°~{int(REAR_MAX)}°  /  최대조향: {MAX_STEER}")
        print("=" * 55)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    if s_flag == 1 and len(scan_buf) > 15:

        # ── 후방(REAR_MIN~REAR_MAX)을 제외한 장애물 포인트 필터링 ────
        near_pts = [
            (a, d) for a, d in scan_buf
            if d <= DETECT and not (REAR_MIN <= a <= REAR_MAX)
        ]

        # ── 우선순위 1: 확장 후진 진행 중 ───────────────────────────
        if extra_back > 0:
            ser_Ardu.write(b"B 0.90\n")
            extra_back -= 1
            print(f"EXTENDED_BACK 잔여={extra_back}사이클")

        # ── 우선순위 2: 장애물 없음 → 직진 ─────────────────────────
        elif not near_pts:
            ser_Ardu.write(b"F 0.00 0.70\n")
            back_cnt = 0

        # ── 우선순위 3: 긴급 후진 (EMERGENCY 이내 포인트 존재) ──────
        elif min(d for _, d in near_pts) <= EMERGENCY:
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.90\n")
                extra_back = 2          # 이번 포함 총 3사이클
                back_cnt   = 0
                print("EXTENDED_BACK 시작! (3x)")
            else:
                # 가장 가까운 포인트 정보 출력
                near_angle, near_dist = min(near_pts, key=lambda x: x[1])
                ser_Ardu.write(b"B 0.90\n")
                print(f"EMERGENCY!  최근접={near_dist:.0f}mm@{near_angle:.0f}°  ({back_cnt}/6)")

        # ── 우선순위 4: 최근접 장애물 기반 회피 ──────────────────────
        else:
            # 가장 가까운 장애물 포인트 탐색
            near_angle, near_dist = min(near_pts, key=lambda x: x[1])

            # 거리 비율 (0=멀다, 1=EMERGENCY 바로 앞)
            ratio = (DETECT - near_dist) / (DETECT - EMERGENCY)
            ratio = min(max(ratio, 0.0), 1.0)

            # 각도를 부호 있는 값으로 변환 (CW 기준)
            # 0°=정면, 양수=오른쪽, 음수=왼쪽
            a_signed = near_angle if near_angle <= 180 else near_angle - 360

            # sin 반발력: 장애물 반대 방향으로 조향
            # -sin(양수 각도) → 음수 steer (장애물이 오른쪽 → 왼쪽으로)
            # -sin(음수 각도) → 양수 steer (장애물이 왼쪽 → 오른쪽으로)
            steer_raw = -math.sin(math.radians(a_signed)) * ratio * MAX_STEER

            # ── 정면 장애물 판단 (sin≈0): 좌우 여유 비교로 방향 결정 ──
            if abs(steer_raw) < FRONT_DEAD * MAX_STEER:
                # 좌반구(180°~360°)와 우반구(0°~180°) 최소 거리 비교
                left_dists  = [d for a, d in near_pts if a > 180]
                right_dists = [d for a, d in near_pts if 0 < a <= 180]
                l_min = min(left_dists)  if left_dists  else DETECT
                r_min = min(right_dists) if right_dists else DETECT
                # 공간이 더 넓은 쪽으로 회전
                steer = -(ratio * MAX_STEER) if l_min > r_min else (ratio * MAX_STEER)
                print(f"FRONT  {near_dist:.0f}mm@{near_angle:.0f}°  "
                      f"L_min={l_min:.0f}  R_min={r_min:.0f}  → {'R' if steer<0 else 'L'}")
            else:
                steer = max(-MAX_STEER, min(MAX_STEER, steer_raw))
                dir_s = "R" if steer < 0 else "L"
                print(f"AVOID  {near_dist:.0f}mm@{near_angle:.0f}°  "
                      f"sin={math.sin(math.radians(a_signed)):+.2f}  "
                      f"→ {dir_s}  steer={steer:.2f}")

            speed = 0.70 * (1 - ratio * 0.7)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())

        # ── 사이클 말 초기화 ──────────────────────────────────────────
        scan_buf = []

# ============================================================
# IMU 测试: 验证前进/转弯时加速度和角速度的变化轴向
# 前进加速 → 看 acc.x/y/z 哪个变
# 转弯 → 看 gyro.x/y/z 哪个变
# ============================================================

from maix import time, app
from maix.ext_dev import imu

sensor = imu.IMU("default", mode=imu.Mode.DUAL,
                  acc_scale=imu.AccScale.ACC_SCALE_2G,
                  acc_odr=imu.AccOdr.ACC_ODR_1000,
                  gyro_scale=imu.GyroScale.GYRO_SCALE_256DPS,
                  gyro_odr=imu.GyroOdr.GYRO_ODR_8000)

print("[IMU] Checking calibration...")
if sensor.calib_gyro_exists():
    sensor.load_calib_gyro()
    print("[IMU] Loaded previous calibration (skip 10s wait)")
else:
    print("[IMU] First boot — calibrating, keep device STILL for 10s...")
    sensor.calib_gyro(10000)
    print("[IMU] Calibration saved.")
print("[IMU] ==========================================")
print("[IMU] Test steps:")
print("[IMU]   1) Hold still → note acc baseline (~0,0,9.8)")
print("[IMU]   2) Accelerate FORWARD → see which acc axis changes")
print("[IMU]   3) Turn LEFT/RIGHT → see which gyro axis changes")
print("[IMU] ==========================================")
print("")
print(f"{'Time':>6s}  {'acc_x':>7s} {'acc_y':>7s} {'acc_z':>7s}  {'gyro_x':>7s} {'gyro_y':>7s} {'gyro_z':>7s}")
print("-" * 70)

last_print = time.ticks_ms()
while not app.need_exit():
    data = sensor.read_all(calib_gryo=True, radian=False)

    now = time.ticks_ms()
    if now - last_print >= 100:  # 10Hz 打印
        print(f"{now:6d}  {data.acc.x:+7.3f} {data.acc.y:+7.3f} {data.acc.z:+7.3f}  "
              f"{data.gyro.x:+7.1f} {data.gyro.y:+7.1f} {data.gyro.z:+7.1f}")
        last_print = now

    time.sleep_ms(5)

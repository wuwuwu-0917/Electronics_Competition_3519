# ============================================================
# 舵机固定角度测试 (MaixPy)
#   - 上电后舵机固定 90°, 不做任何检测/PID控制
# ============================================================

from maix import app, pinmap, pwm, err, sys

# ---- 舵机参数 --------------------------------------------------
SERVO_PIN     = "A31"
SERVO_PWM_ID  = 7
SERVO_FREQ    = 50        # 标准舵机 50Hz
SERVO_MIN_DUTY = 2.5      # 0° 对应占空比 % (1ms)
SERVO_MAX_DUTY = 12.5     # 180° 对应占空比 % (2ms)

TARGET_ANGLE = 90.0       # 固定目标角度

# ---- 舵机初始化 ------------------------------------------------
err.check_raise(
    pinmap.set_pin_function(SERVO_PIN, f"PWM{SERVO_PWM_ID}"),
    f"[Servo] 设置 {SERVO_PIN} → PWM{SERVO_PWM_ID} 失败"
)
servo = pwm.PWM(SERVO_PWM_ID, freq=SERVO_FREQ, duty=0, enable=True)

duty = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * TARGET_ANGLE / 180.0
servo.duty(duty)

print(f"[INFO] Servo on {SERVO_PIN} (PWM{SERVO_PWM_ID})")
print(f"[INFO] freq={SERVO_FREQ}Hz  angle={TARGET_ANGLE}deg  duty={duty:.2f}%")
print(f"[INFO] Servo fixed at {TARGET_ANGLE}°, press Ctrl+C to exit.")

# ---- 主循环: 保持舵机角度, 等待退出 ----
while not app.need_exit():
    pass

print("[INFO] Servo test done.")

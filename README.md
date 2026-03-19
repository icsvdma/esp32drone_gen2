# ESP32 Drone Gen2

ESP32ベースのドローン制御システム  
ESP32-based drone control system

## 積分ワインドアップとは？ / What is Integral Windup?

### 日本語

#### 概要
積分ワインドアップ（Integral Windup）は、PID制御において積分項が過度に蓄積されることで発生する問題です。これにより、制御システムが不安定になり、オーバーシュートや振動が発生します。

#### 問題の詳細

PID制御の積分項（I項）は、誤差を時間積分することで定常偏差を解消します：

```
積分項 = Ki × ∫error(t)dt
```

しかし、以下の状況では積分項が過度に蓄積されます：

1. **長時間の大きな誤差**
   - ドローンが目標角度に到達できない場合
   - モーターが飽和している場合（最大出力に達している）
   - 積分項が無制限に増加し続ける

2. **急激な目標値変更**
   - 操縦者が急にスティックを動かした場合
   - 積分項が過去の誤差を保持したまま

#### 問題の影響（ドローンの場合）

- **オーバーシュート**: 目標角度を大幅に超える
- **振動**: ドローンが左右に揺れ続ける
- **応答遅延**: 制御指令に対する反応が遅くなる
- **不安定性**: 最悪の場合、制御不能になる

#### 解決方法：アンチワインドアップ

このプロジェクトでは、積分項を一定範囲に制限する方法を実装しています：

```cpp
// 積分項の更新
integral += error * dt;

// アンチワインドアップ：積分項を制限
if (integral > INTEGRAL_MAX) {
    integral = INTEGRAL_MAX;
} else if (integral < INTEGRAL_MIN) {
    integral = INTEGRAL_MIN;
}
```

**設定値:**
- `INTEGRAL_MAX = 100.0`
- `INTEGRAL_MIN = -100.0`

これにより、積分項が±100.0の範囲に制限され、過度な蓄積を防ぎます。

#### ドローン制御での実例

**シナリオ**: ドローンを右に10度傾けたい

1. **アンチワインドアップなし**
   ```
   誤差 = 10度 - 0度 = 10度
   積分項 += 10 × 0.01秒 = 0.1
   （繰り返し）
   積分項が無制限に増加 → 500, 1000, 2000...
   モーター出力 = 過大 → オーバーシュート → 振動
   ```

2. **アンチワインドアップあり**
   ```
   誤差 = 10度 - 0度 = 10度
   積分項 += 10 × 0.01秒 = 0.1
   （繰り返し）
   積分項が100.0で制限される
   モーター出力 = 適切 → スムーズな制御
   ```

---

### English

#### Overview
Integral Windup is a problem in PID control where the integral term accumulates excessively, causing control system instability, overshoot, and oscillation.

#### Problem Details

The integral term (I-term) in PID control eliminates steady-state error by integrating the error over time:

```
Integral term = Ki × ∫error(t)dt
```

However, the integral term can accumulate excessively in these situations:

1. **Large error over extended periods**
   - When the drone cannot reach the target angle
   - When motors are saturated (at maximum output)
   - The integral term grows unbounded

2. **Sudden setpoint changes**
   - When the pilot suddenly moves the control stick
   - The integral term retains accumulated past errors

#### Effects on Drone Control

- **Overshoot**: Exceeds target angle significantly
- **Oscillation**: Drone continues to wobble left and right
- **Delayed response**: Slow reaction to control commands
- **Instability**: In worst cases, loss of control

#### Solution: Anti-Windup

This project implements integral clamping to limit the integral term:

```cpp
// Update integral term
integral += error * dt;

// Anti-windup: clamp integral term
if (integral > INTEGRAL_MAX) {
    integral = INTEGRAL_MAX;
} else if (integral < INTEGRAL_MIN) {
    integral = INTEGRAL_MIN;
}
```

**Settings:**
- `INTEGRAL_MAX = 100.0`
- `INTEGRAL_MIN = -100.0`

This limits the integral term to the range ±100.0, preventing excessive accumulation.

#### Real-World Example in Drone Control

**Scenario**: Want to tilt drone 10 degrees to the right

1. **Without Anti-Windup**
   ```
   error = 10° - 0° = 10°
   integral += 10 × 0.01s = 0.1
   (repeated)
   integral grows unbounded → 500, 1000, 2000...
   motor output = excessive → overshoot → oscillation
   ```

2. **With Anti-Windup**
   ```
   error = 10° - 0° = 10°
   integral += 10 × 0.01s = 0.1
   (repeated)
   integral clamped at 100.0
   motor output = appropriate → smooth control
   ```

---

## PID Controller Implementation

### Default PID Gains (デフォルトPIDゲイン)
- **Kp = 2.0** - Proportional gain for angle control (角度制御の比例ゲイン)
- **Ki = 0.05** - Integral gain with anti-windup (アンチワインドアップ付き積分ゲイン)
- **Kd = 0.5** - Derivative gain with spike protection (スパイク保護付き微分ゲイン)

### Safety Features (安全機能)
1. **Integral Anti-Windup** (積分アンチワインドアップ): ±100.0 limits
2. **Derivative Spike Protection** (微分スパイク保護): Minimum dt = 0.001s
3. **Runtime Tuning** (実行時調整): PS3 controller buttons for PID adjustment

### In-Flight PID Tuning (飛行中のPID調整)

PS3コントローラーのボタンで飛行中にPIDゲインを調整できます：

- **Left/Right**: PID parameter selection (P, I, or D)
- **Up**: Increase selected parameter
- **Down**: Decrease selected parameter

---

## Hardware Configuration

### Motor Layout (モーター配置)
```
      Front (前)
   m2(FL)  m1(FR)
      \ X /
       |↑|  
      / X \
   m4(RL)  m3(RR)
```

- m1: Front-Right (GPIO 26)
- m2: Front-Left (GPIO 25)
- m3: Rear-Right (GPIO 27)
- m4: Rear-Left (GPIO 14)

### Control Mixing (制御ミキシング)
```cpp
m1 = base_thrust - roll_output - pitch_output  // FR
m2 = base_thrust + roll_output - pitch_output  // FL
m3 = base_thrust - roll_output + pitch_output  // RR
m4 = base_thrust + roll_output + pitch_output  // RL
```

---

## Building and Uploading

### Requirements
- PlatformIO
- ESP32 development board
- PS3 Controller

### Build
```bash
platformio run
```

### Upload
```bash
platformio run --target upload
```

---

## Troubleshooting (トラブルシューティング)

### ドローンが振動する / Drone oscillates
- PID gains may be too high (PIDゲインが高すぎる可能性)
- Try reducing Kp and Kd values (KpとKdを下げてみる)

### 応答が遅い / Slow response
- PID gains may be too low (PIDゲインが低すぎる可能性)
- Try increasing Kp value (Kpを上げてみる)

### 一方向に傾く / Drifts in one direction
- Sensor calibration may be needed (センサーキャリブレーションが必要)
- Check motor balance (モーターバランスを確認)

---

## References (参考文献)

### PID Control Theory
- [PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [積分ワインドアップ - Wikipedia](https://ja.wikipedia.org/wiki/PID%E5%88%B6%E5%BE%A1)

### Anti-Windup Techniques
1. **Clamping/Saturation** (本プロジェクトで使用 / Used in this project)
2. Back-calculation
3. Conditional integration

---

## License

[Add your license here]

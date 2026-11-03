# 操縦性・安定性レビュー（esp32drone_gen2）

対象コミット: `20131f5`（2025-05-29）
レビュー方法: ソースコードの静的レビューのみ（実機・ビルドでの検証はしていない）

---

## 1. リポジトリの現状

### 構成

| ファイル | 役割 |
|---|---|
| `src/main.cpp` | `setup()` で `DroneController::init()`、`loop()` で `update(0.01)` を約 100Hz で呼ぶ |
| `src/drone_controller.cpp` | 制御のまとめ役。PS3 入力 → PID（Roll/Pitch）→ 4 モータへのミキシング |
| `src/sensor.cpp` | MPU6050 を ESP-IDF の I2C ドライバで直接読む。別タスク（10ms 周期）で Madgwick に流す |
| `src/madgwick_filter.cpp` | Arduino `Madgwick` ライブラリのラッパ（6 軸 `updateIMU`） |
| `src/pid_controller.cpp` | 単純な PID。ゲインは PS3 の十字キーで実行時に増減する |
| `src/ps3_controller.cpp` | 左スティック Y = スロットル、右スティック = Roll/Pitch 目標角（±30°）、十字キー左右 = P/I/D 選択、上下 = ゲイン増減 |
| `src/motor.cpp` | LEDC PWM（16kHz, 10bit）でブラシモータを駆動 |
| `src/batt_adc.cpp` | バッテリ電圧を 3 秒ごとに ADC で測って printf |
| `src/ledctrl.cpp` | WS2812 で PID 選択状態を表示 |

### 開発段階

- コミットは 4 件（2025-05-20〜05-29）。プロトタイプ段階。
- `platformio.ini` に `build_flags = -DDEBUG_MODE` が入っているため、**現状のビルドではモータは一切回らない**（`drone_controller.cpp:67-81` でデバッグ表示だけになる）。
- コミットメッセージにある通り「ヨー軸が常にドリフトする」問題が残っている。
- ヨー制御、アーム/ディスアーム、通信断時のフェイルセーフ、低電圧保護は未実装。

---

## 2. 指摘事項（重要度順）

### 【致命的】A. ジャイロ/加速度のオフセット補正が一切効いていない

- `include/sensor.h:24-29` に `OFS_Gyro_X` などが定義されているが、どこからも使われていない。
- `MPU6050 mpu_host`（electroniccats ライブラリ）も初期化されておらず、オフセットレジスタにも書かれていない。
- 生のジャイロバイアスがそのまま Madgwick に入る。例えば `OFS_Gyro_Z = 132` が ±250dps 換算のバイアスだとすると `132/131 ≒ 1°/s` で、1 分で約 60° ヨーがずれる。**ヨードリフトの主因**と考えられる（6 軸 IMU は地磁気がないのでヨーの絶対角はそもそも原理的にドリフトするが、バイアス未補正だとそれが極端に速くなる）。
- Roll/Pitch も加速度オフセット未補正だと水平の基準がずれ、「手を離すと一方向に流れる」原因になる。
- 補足: これらの値は IMU_Zero などで求めた**オフセットレジスタ用の値**の可能性が高い（推測）。その場合、生データから単純に引くとスケールが合わない。

### 【致命的】B. 角度 PID のみで、角速度（レート）ループがない

- `drone_controller.cpp:64-65` で、Madgwick の角度を直接 PID している。
- D 項が「角度誤差の差分 / 固定 dt」なので、ノイズの多い角度を数値微分している＋スティックを動かした瞬間に D キックが出る。
- 小型機ではジャイロの角速度を直接使う**カスケード制御（外側: 角度 P → 内側: 角速度 PID）**が定石。ジャイロは Madgwick を通さないので遅れが少なく、振動抑制が格段に良くなる。

### 【致命的】C. ヨー制御が存在しない

- PID は `pid_roll` / `pid_pitch` のみ。ミキサにヨー項がない。
- モータ/プロペラの個体差でトルクがずれると機体は回り続け、止める手段がない。
- 6 軸 IMU ではヨー角を目標にするのではなく、**ジャイロ Z の角速度を 0 に保つレート制御**にすべき。

### 【重大】D. 制御周期が不正確・遅い

- `main.cpp:19` は `vTaskDelay(10ms)` なので、実周期は 10ms + 処理時間（NeoPixel の `show()`、printf を含む）になる。にもかかわらず dt は 0.01 固定 → I/D 項のスケールがずれる。
- センサタスク（優先度 5、コア未指定）と `loop()`（優先度 1、コア 1）が非同期。角度は最大 1 周期ぶん古く、Bluetooth スタック（コア 0）と競合する可能性もある。
- 100Hz は小型ドローンには遅い（一般的には 250Hz〜1kHz 以上）。
- `sensor.cpp:23` で I2C が 100kHz（`I2C_MASTER_FREQ_HZ` = 400kHz の定義が使われていない）。14 バイト読み出しに約 2ms かかる。

### 【重大】E. MPU6050 のレンジ・デジタルローパスフィルタ（DLPF）未設定

- 初期化は PWR_MGMT_1 の wake のみ。デフォルトは ジャイロ ±250dps / 加速度 ±2g / DLPF 260Hz。
- ±250dps は機体が少し速く回ると飽和する → 姿勢推定が壊れる。
- DLPF 260Hz だとモータ振動がそのまま加速度に乗り、Madgwick の Roll/Pitch がガタつく。
- WHO_AM_I の確認や `init()` の戻り値チェックもない（失敗してもゼロデータで動いてしまう）。

### 【重大】F. 安全機能がない

- **フェイルセーフなし**: `ps3_controller.cpp:28-45` は切断時に何もしないので、最後のスロットル値のまま飛び続ける。
- **アーム/ディスアームなし**: スロットル 0 でも PID 出力がミキサに加算されるため、地上で傾いていると一部のモータが回る。
- **積分ワインドアップ**: `pid_controller.cpp:8` で積分が無制限。地上待機中にも積分が溜まり、離陸の瞬間にガクッと傾く。
- PID 出力の上限なし。`(int16_t)` キャストで範囲外の float を変換すると未定義動作。

### 【中】G. スティックの中心が −1° ずれている

`mapTarget(-rx, -127, 128, -30, 30)` はスティック中立（0）で
`(0+127)*60/255 - 30 = 29 - 30 = -1`（整数除算）となり、**中立で Roll/Pitch ともに −1° の指令**が出続ける。デッドバンドもなく、分解能も 1° 刻み。

### 【中】H. ミキサの符号が実機用とデバッグ用で違う

- 実機用（`drone_controller.cpp:77-80`）: FR = T − R − P, FL = T + R − P, RR = T − R + P, RL = T + R + P
- デバッグ用（同 110-113）: m1 = T + R + P, m2 = T − R + P, m3 = T + R − P, m4 = T − R − P

デバッグ表示で確認した内容が実機の動きと一致しない。さらに `base_roll = -1 * getRoll()` と `mapTarget(-rx …)` で符号を二重に反転しているので、符号の整合性が追いにくい。センサの取り付け向きと合っているかは実機で確認が必要（確認手順は 4 章）。

### 【中】I. スロットルが飽和して姿勢制御の余裕がない

- スロットルは 0〜1024 にそのまま割り当て、PID 出力を足した後に 0〜1023 でクリップしているだけ。
- 高スロットルでは上側、低スロットルでは下側がクリップされ、姿勢補正が効かなくなる。ミキサの飽和処理（全体をシフトして差分を保つ）と、スロットル上限が必要。

### 【中】J. PID ゲインの扱い

- 起動時のゲインは 0（グローバルオブジェクトなのでゼロ初期化される。コンストラクタでは初期化していないので壊れやすい）。毎回ボタンで調整しなおしで、保存されない。
- P は ±1.0 刻み、I/D は ±0.01 刻みで、下限がない → **負のゲインにできてしまう**（正帰還になって暴れる）。
- `notify()`（BT コールバック）と制御ループの間で排他制御がない。

### 【低】K. その他

- `#include "mpu6050.h"`（`main.cpp`, `drone_controller.cpp`, `madgwick_filter.cpp`）: ファイルは `sensor.h` に改名済み。Windows/macOS では大文字小文字を区別しないのでライブラリの `MPU6050.h` が拾われてビルドできるが、Linux（CI など）ではビルドエラーになる。
- `platform = espressif32` がバージョン固定されていない。Arduino-ESP32 3.x では `ledcSetup`/`ledcAttachPin` が廃止されているので、更新でビルドが壊れる。
- `batt_adc.cpp:23-27`: `rawvolt` を 0 リセットせずに足し込み、ループが `<=` で 65 回 → 測定値が実際より高く出る。`#ifndef DEBUG` の条件も逆。
- `ledctrl.cpp:10-14` で `define_system.h` のマクロを違う値で再定義している。
- `motor.cpp:5` のコメント（20kHz）と実際の値（16kHz）が合っていない。
- Madgwick は起動直後、初期姿勢へ収束するまで数秒かかる。起動直後に飛ばすと傾いた基準になる。

---

## 3. 修正案

優先順位: **A → F → E → D/B/C → G/H/I → J/K**

### 3.1 起動時ジャイロキャリブレーション＋レンジ/DLPF 設定（A, E）

```cpp
// sensor.h
#define MPU6050_SMPLRT_DIV_REG   0x19
#define MPU6050_CONFIG_REG       0x1A
#define MPU6050_GYRO_CONFIG_REG  0x1B
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_WHO_AM_I_REG     0x75

#define GYRO_LSB_PER_DPS   32.8f    // ±1000dps
#define ACCEL_LSB_PER_G    4096.0f  // ±8g

// sensor.cpp
esp_err_t MPU6050_user::write_reg(uint8_t reg, uint8_t val) {
	uint8_t buf[2] = {reg, val};
	return i2c_master_write_to_device(i2c_port, dev_addr, buf, 2, pdMS_TO_TICKS(10));
}

esp_err_t MPU6050_user::init_mpu() {
	uint8_t who = 0;
	ESP_ERROR_CHECK_WITHOUT_ABORT(read_bytes(MPU6050_WHO_AM_I_REG, &who, 1));
	if (who != 0x68) return ESP_FAIL;          // 互換品は 0x70/0x72 などの場合あり
	write_reg(MPU6050_PWR_MGMT_1_REG, 0x01);    // wake + PLL(ジャイロX)クロック
	vTaskDelay(pdMS_TO_TICKS(100));
	write_reg(MPU6050_CONFIG_REG,       0x02);  // DLPF 約 98Hz（振動が多ければ 0x03 = 約 44Hz）
	write_reg(MPU6050_SMPLRT_DIV_REG,   0x00);  // DLPF 有効時 1kHz
	write_reg(MPU6050_GYRO_CONFIG_REG,  0x10);  // ±1000dps
	write_reg(MPU6050_ACCEL_CONFIG_REG, 0x10);  // ±8g
	return ESP_OK;
}

// 機体を静止させた状態で呼ぶ
void MPU6050_user::calibrate_gyro(int samples) {
	int32_t sx = 0, sy = 0, sz = 0;
	for (int i = 0; i < samples; i++) {
		if (read_bytes(MPU6050_ACCEL_XOUT_H_REG, data, 14) == ESP_OK) {
			sx += (int16_t)((data[8]  << 8) | data[9]);
			sy += (int16_t)((data[10] << 8) | data[11]);
			sz += (int16_t)((data[12] << 8) | data[13]);
		}
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	gx_ofs = sx / samples;
	gy_ofs = sy / samples;
	gz_ofs = sz / samples;
}
```

読み取り後に `gx -= gx_ofs;` などで補正し、Madgwick 側のスケールを `GYRO_LSB_PER_DPS` / `ACCEL_LSB_PER_G` に合わせる。I2C は `conf.master.clk_speed = I2C_MASTER_FREQ_HZ;`（400kHz）にする。内蔵プルアップは弱いので外付けプルアップ（2.2k〜4.7kΩ）があることを確認する。

### 3.2 1 つの制御タスクに統合して周期を固定（D）

```cpp
// 250Hz（4ms）。センサ読み → 姿勢推定 → PID → ミキサ を同じタスクで順に実行
void DroneController::controlTask(void *param) {
	auto *self = static_cast<DroneController*>(param);
	TickType_t last = xTaskGetTickCount();
	int64_t prev_us = esp_timer_get_time();
	for (;;) {
		vTaskDelayUntil(&last, pdMS_TO_TICKS(4));
		int64_t now_us = esp_timer_get_time();
		float dt = (now_us - prev_us) * 1e-6f;
		prev_us = now_us;
		self->step(dt);   // read IMU → madgwick.updateIMU → PID → mix
	}
}
// コア1に固定、高優先度。Bluetooth はコア0
xTaskCreatePinnedToCore(DroneController::controlTask, "ctrl", 8192, this,
                        configMAX_PRIORITIES - 2, nullptr, 1);
```

- `madgwick.begin(250)` と周期を合わせる。
- LED 更新や printf は制御タスクから外し、別の低優先度タスク（例: 10Hz）でまとめて行う。

### 3.3 カスケード PID ＋ ヨーレート制御（B, C, F）

```cpp
class PIDController {
public:
	float kp = 0, ki = 0, kd = 0;
	float i_limit = 100.0f, out_limit = 300.0f;

	float compute(float setpoint, float measured, float dt) {
		float error = setpoint - measured;
		integral = clampf(integral + ki * error * dt, -i_limit, i_limit);
		// D は測定値の微分（目標変化による D キックを防ぐ）
		float d_meas = (measured - prev_meas) / dt;
		d_filt += 0.3f * (d_meas - d_filt);       // 簡易 1 次 LPF
		prev_meas = measured;
		return clampf(kp * error + integral - kd * d_filt, -out_limit, out_limit);
	}
	void reset() { integral = 0; d_filt = 0; prev_meas = 0; }
private:
	float integral = 0, prev_meas = 0, d_filt = 0;
	static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};
```

```cpp
// 外側: 角度 → 目標角速度（P のみで十分なことが多い）
float roll_rate_sp  = ANGLE_KP * (roll_sp_deg  - madgwick.getRoll());
float pitch_rate_sp = ANGLE_KP * (pitch_sp_deg - madgwick.getPitch());
float yaw_rate_sp   = yaw_stick * MAX_YAW_RATE_DPS;   // 左スティック X

// 内側: 角速度 PID（ジャイロの dps を直接使う）
float r = pid_roll_rate .compute(roll_rate_sp,  gyro_x_dps, dt);
float p = pid_pitch_rate.compute(pitch_rate_sp, gyro_y_dps, dt);
float y = pid_yaw_rate  .compute(yaw_rate_sp,   gyro_z_dps, dt);
```

ジャイロの軸と Roll/Pitch の対応・符号は IMU の取り付け向きによる。4 章の手順で確認すること。

### 3.4 ミキサ（飽和処理つき）とアーム/フェイルセーフ（F, H, I）

```cpp
void DroneController::mix_and_output(float thr, float r, float p, float y) {
	// 実機用とデバッグ用で必ず同じ関数を使う
	// 符号はプロペラの回転方向・センサの向きによる（4 章で確認）
	float m[4] = {
		thr - r - p + y,  // FR
		thr + r - p - y,  // FL
		thr - r + p - y,  // RR
		thr + r + p + y,  // RL
	};
	// 飽和したら全体をシフトして、モータ間の差（姿勢補正量）を保つ
	float mx = m[0], mn = m[0];
	for (float v : m) { mx = fmaxf(mx, v); mn = fminf(mn, v); }
	float shift = 0;
	if (mx > PWM_MAX) shift = PWM_MAX - mx;
	else if (mn < PWM_IDLE) shift = PWM_IDLE - mn;
	for (int i = 0; i < 4; i++) m[i] = constrain(m[i] + shift, PWM_IDLE, PWM_MAX);

#ifdef DEBUG_MODE
	printf("%d,%d,%d,%d\n", (int)m[0], (int)m[1], (int)m[2], (int)m[3]);
#else
	motors[0].set_speed(m[0]); motors[1].set_speed(m[1]);
	motors[2].set_speed(m[2]); motors[3].set_speed(m[3]);
#endif
}

void DroneController::step(float dt) {
	// ... IMU 更新 ...
	bool link_ok = ps3ctrl.isConnected() &&
	               (millis() - ps3ctrl.lastUpdateMs() < 300);
	if (!link_ok) armed = false;                         // フェイルセーフ
	if (!armed || throttle < THR_MIN_FOR_CONTROL) {
		stop_all_motors();
		pid_roll_rate.reset(); pid_pitch_rate.reset(); pid_yaw_rate.reset();
		return;                                          // 地上ではワインドアップさせない
	}
	// ... PID → mix_and_output ...
}
```

- アームは「スロットル最下部＋START ボタン」など、誤操作しにくい条件にする。
- スロットル上限は PWM_MAX の 80〜85% くらいにして、姿勢制御の余裕を残す。

### 3.5 スティック入力の修正（G）

```cpp
static float stick_norm(int8_t raw, int deadband = 6) {
	int v = raw;
	if (abs(v) <= deadband) return 0.0f;
	float s = (abs(v) - deadband) / (127.0f - deadband);
	if (s > 1.0f) s = 1.0f;
	return v > 0 ? s : -s;          // -1.0〜+1.0、中立は厳密に 0
}

roll_target_deg  = -stick_norm(Ps3.data.analog.stick.rx) * MAX_ANGLE_DEG; // 符号は 1 か所だけで決める
pitch_target_deg = -stick_norm(Ps3.data.analog.stick.ry) * MAX_ANGLE_DEG;
```

目標値は `int16_t` ではなく `float` で持つ（1° 刻みの段差をなくす）。

### 3.6 ゲイン管理（J）

- コンストラクタで初期値を明示的に入れる。
- 調整後は `Preferences`（NVS）に保存し、起動時に読み込む。
- `set_kp()` などで `fmaxf(0, …)` として負にならないようにする。刻み幅は現在値の 10% 程度にすると調整しやすい。
- `notify()` からは「要求フラグ」だけ立てて、ゲインの書き換えは制御タスク側で行う（または `portMUX` で保護する）。

### 3.7 小さい修正（K）

- `#include "mpu6050.h"` → `#include "sensor.h"`
- `platform = espressif32@6.x.x` のようにバージョンを固定する（Arduino-ESP32 2.x 系の API を使っているため）。
- `batt_adc.cpp`: `rawvolt = 0;` で初期化してから `loop < NUM_OF_SAMPLE` でループする。`#ifndef DEBUG` → `#ifdef DEBUG_MODE`。
- 低電圧時はスロットルを制限するか、着陸を促す。電圧が下がると同じデューティでも推力が落ちるので、電圧補償（`duty * V_nominal / V_batt`）も効果がある。

---

## 4. 実機での確認手順（符号チェック）

1. `DEBUG_MODE` のまま、機体を手で持って**右側を下げる** → Roll の推定値の符号を記録する。
2. 同様に**機首を上げる** → Pitch の符号を記録する。
3. 上から見て**時計回りに回す** → ジャイロ Z の符号を記録する。
4. スロットルを少し上げた状態（デバッグ表示）で 1 と 2 を繰り返し、**下がった側のモータ出力が増える**ことを確認する（増えなければミキサの該当項の符号を反転する）。
5. ヨーは、時計回りに回したとき**反時計回りのトルクを出すモータ（=時計回りに回るプロペラのモータ）の出力が増える**ことを確認する。
6. プロペラを外した状態でアーム → スロットル → 通信断（コントローラの電源 OFF）を試し、モータが停止することを確認する。

## 5. チューニングの進め方

1. 内側のレート PID から。P だけで、小刻みに振動が出始める手前まで上げる → 少し下げる。
2. D を足して、P をもう少し上げられるか試す（D はノイズに弱いので、DLPF と D の LPF が前提）。
3. I は、ゆっくり流れるのを止める程度に小さく入れる。
4. 最後に外側の角度 P を、戻りの速さが好みになるまで調整する。
5. Roll → Pitch → Yaw の順に 1 軸ずつ。テストベンチ（1 軸ジンバル、ひもで吊るすなど）を使うと安全。

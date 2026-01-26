# 1. MPU-9250 실제 Raw Data 예시

MPU-9250의 레지스터(메모리)에서 I2C로 데이터를 읽어오면, -32768 ~ +32767 범위의 16비트 정수(int16) 값이 나옵니다.

**상황 설정:** 사용자가 약간 앞으로 뛰면서($+X$ 속도 유지), 몸이 살짝 흔들리고($-X, +Y$ 가속), 아주 미세하게 왼쪽으로 회전하려는 순간입니다.

### [Step 1] Raw Data 획득 (Int16)
**MPU-9250 설정:** 가속도 범위 $\pm2g$, 자이로 범위 $\pm250^\circ/s$

* **ACCEL_XOUT_H/L:** -83 (미세한 감속 진동)
* **ACCEL_YOUT_H/L:** 200 (우측 쏠림 진동)
* **ACCEL_ZOUT_H/L:** 16500 (중력가속도 $1g \approx 16384$ 근처)
* **GYRO_ZOUT_H/L:** 750 (Z축 회전)

### [Step 2] 전처리: 물리 단위(SI) 변환
이 정수값들을 우리가 아는 $m/s^2$과 $rad/s$로 바꿔야 계산이 가능합니다.

**가속도 변환 (Sensitivity: 16384 LSB/g)**
* $a_x (g) = -83 / 16384 \approx -0.00506 g$
* $a_x (m/s^2) = -0.00506 \times 9.80665 \approx \mathbf{-0.0497 \, m/s^2}$
* $a_y (m/s^2) = (200 / 16384) \times 9.80665 \approx \mathbf{0.1197 \, m/s^2}$

**자이로 변환 (Sensitivity: 131 LSB/($^\circ/s$))**
* $\omega_z (^\circ/s) = 750 / 131 \approx 5.725^\circ/s$
* $\omega_z (rad/s) = 5.725 \times (\pi / 180) \approx \mathbf{0.0999 \, rad/s}$

---

# 2. EKF 예측 단계 계산 (Prediction Step)

이제 위에서 구한 실제 물리값($u_k$)을 가지고 상태를 업데이트합니다.

### A. 초기 상태 및 입력 정의

**이전 상태 ($x_{k-1}$):**
$$
x_{k-1} = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \\ \theta \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 1.5 \\ 0.5 \\ 0 \end{bmatrix}
$$
(위치 0,0 / 속도 X:1.5, Y:0.5 / 방향: 0 rad)

**입력 벡터 ($u_k$) - 변환된 센서값:**
$$
u_k = [a_x, a_y, \omega_z]^T = [-0.0497, \, 0.1197, \, 0.0999]
$$

**시간 간격 ($dt$):** $0.01s$ (100Hz 샘플링)

### B. 좌표계 회전 (Body Frame $\to$ Global Frame)
가속도센서는 몸(Device) 기준이므로, 현재 사용자가 바라보는 방향($\theta$)으로 회전시켜 지도(Global) 기준 가속도로 바꿔야 합니다.

현재 방향 $\theta = 0$이므로, $\cos(0)=1, \sin(0)=0$.
즉, 회전 행렬이 단위 행렬이 되어 몸 기준 가속도가 그대로 글로벌 가속도가 됩니다.

$$
a_{global\_x} \approx -0.0497
$$
$$
a_{global\_y} \approx 0.1197
$$

(만약 $\theta$가 0이 아니었다면 $a_{gx} = a_x \cos\theta - a_y \sin\theta$ 공식을 씁니다.)

### C. 상태 전이 계산 (State Transition)
물리학의 등가속도 운동 공식을 적용합니다.

**위치 예측 ($p = p + v \Delta t + \frac{1}{2} a \Delta t^2$)**
* $p_x = 0 + (1.5 \times 0.01) + (0.5 \times -0.0497 \times 0.0001)$
    $= 0 + 0.015 - 0.000002485$
    $\approx \mathbf{0.014997 \, m}$
* $p_y = 0 + (0.5 \times 0.01) + (0.5 \times 0.1197 \times 0.0001)$
    $= 0 + 0.005 + 0.000005985$
    $\approx \mathbf{0.005006 \, m}$

**속도 예측 ($v = v + a \Delta t$)**
* $v_x = 1.5 + (-0.0497 \times 0.01)$
    $= 1.5 - 0.000497$
    $\approx \mathbf{1.4995 \, m/s}$
* $v_y = 0.5 + (0.1197 \times 0.01)$
    $= 0.5 + 0.001197$
    $\approx \mathbf{0.5012 \, m/s}$

**방향 예측 ($\theta = \theta + \omega \Delta t$)**
* $\theta = 0 + (0.0999 \times 0.01)$
    $= 0 + 0.000999$
    $\approx \mathbf{0.0010 \, rad}$

---

# 3. 최종 결과 ($x_{k|k-1}$)

MPU-9250의 Raw Data [-83, 200, 750]이 들어왔을 때, 0.01초 후의 상태 벡터는 다음과 같이 계산됩니다.

$$
x_{k|k-1} = \begin{bmatrix} 0.014997 \\ 0.005006 \\ 1.4995 \\ 0.5012 \\ 0.0010 \end{bmatrix}
$$

> **[Senior Note]**
> 이 계산은 "Prediction(예측)" 단계입니다. 실제 임베디드 코드에서는 이 값($x_{k|k-1}$)과 오차 공분산 행렬($P$)을 메모리에 저장해두고, GPS 데이터가 들어오는 순간(Correction) 다시 꺼내어 보정하게 됩니다. 센서 Raw 값(int16)을 물리 단위(float)로 바꾸는 과정에서 Sensitivity Scale Factor를 데이터시트(MPU-9250 Product Spec)에서 찾아 정확히 적용하는 것이 구현의 핵심입니다.


# 1. 실제 I2C 레지스터에서 보이는 값 (Register Dump)

MPU-9250은 데이터를 **상위 8비트(High Byte)**와 **하위 8비트(Low Byte)**로 나누어 저장합니다. (Big-endian 방식)

### (1) ACCEL_X (값: -83)
음수는 **2의 보수(Two's Complement)** 형태로 표현됩니다.

* **10진수:** -83
* **2진수(16bit):** `1111 1111 1010 1101`
* **개발자가 보는 값:** `0xFF`, `0xAD`
* **레지스터 0x3B (ACCEL_XOUT_H):** `0xFF` (1111 1111)
* **레지스터 0x3C (ACCEL_XOUT_L):** `0xAD` (1010 1101)

### (2) ACCEL_Y (값: 200)
양수는 그대로 16진수로 변환됩니다.

* **10진수:** 200
* **2진수(16bit):** `0000 0000 1100 1000`
* **개발자가 보는 값:** `0x00`, `0xC8`
* **레지스터 0x3D (ACCEL_YOUT_H):** `0x00`
* **레지스터 0x3E (ACCEL_YOUT_L):** `0xC8` (200)

### (3) ACCEL_Z (값: 16500, 약 1g)
값이 크기 때문에 상위 바이트에도 데이터가 꽉 찹니다.

* **10진수:** 16500
* **2진수(16bit):** `0100 0000 0111 0100`
* **개발자가 보는 값:** `0x40`, `0x74`
* **레지스터 0x3F (ACCEL_ZOUT_H):** `0x40` (64)
* **레지스터 0x40 (ACCEL_ZOUT_L):** `0x74` (116)
* **검산:** $(64 \times 256) + 116 = 16384 + 116 = 16500$

### (4) GYRO_Z (값: 750)

* **10진수:** 750
* **2진수(16bit):** `0000 0010 1110 1110`
* **개발자가 보는 값:** `0x02`, `0xEE`
* **레지스터 0x47 (GYRO_ZOUT_H):** `0x02`
* **레지스터 0x48 (GYRO_ZOUT_L):** `0xEE`

---

# 2. 시리얼 모니터/디버거 화면 예시 (ASCII Art)

실제 펌웨어 개발 시 I2C Read를 수행하면 아래와 같은 Byte Stream이 들어옵니다.

```plaintext
[I2C Read Sequence from Address 0x3B]

Addr  | Name         | Hex Value | Binary View (8-bit)
------+--------------+-----------+--------------------
0x3B  | ACCEL_X_H    | 0xFF      | 1111 1111  <-- -83의 상위 비트 (Sign Extension)
0x3C  | ACCEL_X_L    | 0xAD      | 1010 1101
0x3D  | ACCEL_Y_H    | 0x00      | 0000 0000
0x3E  | ACCEL_Y_L    | 0xC8      | 1100 1000
0x3F  | ACCEL_Z_H    | 0x40      | 0100 0000
0x40  | ACCEL_Z_L    | 0x74      | 0111 0100
...   | ...          | ...       | ...
0x47  | GYRO_Z_H     | 0x02      | 0000 0010
0x48  | GYRO_Z_L     | 0xEE      | 1110 1110
```

# 1. 가속도(Acceleration) 변환 공식

가속도 센서는 기본적으로 중력가속도($g$) 단위로 값을 내보냅니다. 이를 우리가 물리 계산에 쓸 **미터 퍼 세크 제곱($m/s^2$)**으로 바꿔야 합니다.

### (1) 가속도 X축 계산 분해

$$
a_x (m/s^2) = \underbrace{\left( \frac{-83}{16384} \right)}_{\text{g단위 변환}} \times \underbrace{9.80665}_{\text{SI단위 변환}} \approx -0.0497
$$

* **-83:** `[Raw Data]` 센서 레지스터(ACCEL_XOUT)에서 읽어온 16비트 정수 원본 값 (LSB).
* **/:** `[나눗셈 연산]` Raw 데이터를 실제 물리적 비율로 줄이는 과정.
* **16384:** `[Sensitivity Scale Factor]` 센서 민감도 상수.
    * **의미:** MPU-9250을 $\pm2g$ 모드로 설정했을 때, $1g$의 중력을 받으면 센서는 16384라는 값을 뱉습니다. 즉, **16384 LSB = 1g**.
* **-0.00506:** `[Intermediate Value]` $g$ 단위로 변환된 가속도 값. (약 -0.005g)
* **×:** `[곱셈 연산]` 단위를 변환하기 위한 과정.
* **9.80665:** `[Standard Gravity]` 표준 중력 상수($g$). $1g \approx 9.8m/s^2$.
* **-0.0497:** `[Final Value]` 최종 물리량. 단위는 $m/s^2$. (칼만 필터 입력값)

### (2) 가속도 Y축 계산 분해

$$
a_y (m/s^2) = \left( \frac{200}{16384} \right) \times 9.80665 \approx 0.1197
$$

* **200:** `[Raw Data]` 센서 레지스터(ACCEL_YOUT)에서 읽어온 16비트 정수 원본 값.
* **16384:** `[Sensitivity Scale Factor]` 위와 동일한 민감도 상수.
* **9.80665:** `[Standard Gravity]` 표준 중력 상수.

---

# 2. 자이로(Gyroscope) 변환 공식

자이로 센서는 **초당 회전 각도(Degree per second, dps)**를 측정합니다. 하지만 삼각함수($\sin, \cos$)를 쓰는 수학 공식들은 **라디안(Radian)** 단위를 씁니다.

### (3) 자이로 Z축 계산 분해

$$
\omega_z (rad/s) = \underbrace{\left( \frac{750}{131} \right)}_{\text{도(Degree) 변환}} \times \underbrace{\left( \frac{\pi}{180} \right)}_{\text{라디안 변환}} \approx 0.0999
$$

* **750:** `[Raw Data]` 센서 레지스터(GYRO_ZOUT)에서 읽어온 16비트 정수 원본 값.
* **131:** `[Sensitivity Scale Factor]` 자이로 민감도 상수.
    * **의미:** MPU-9250을 $\pm250^\circ/s$ 모드로 설정했을 때, **131 LSB = 1^\circ/s**입니다.
* **5.725:** `[Intermediate Value]` 60분법 각도($^\circ/s$)로 변환된 회전 속도. (약 초당 5.7도 회전)
* **π / 180:** `[Conversion Factor]` Degree를 Radian으로 바꾸는 상수.
    * $\pi \approx 3.14159...$
    * $180^\circ = \pi \text{ rad}$ 이므로, $1^\circ = \frac{\pi}{180} \text{ rad}$.
    * 이 값은 약 **0.0174533** 입니다.
* **0.0999:** `[Final Value]` 최종 물리량. 단위는 $rad/s$. (칼만 필터의 방향 예측 식 $\theta = \theta + \omega dt$ 에 쓰임)

> **[Senior Optimization Tip]**
> 임베디드 시스템(MCU)에서는 나눗셈과 실수 연산 비용이 큽니다.
> 매번 `(raw / 16384.0) * 9.8`을 계산하는 대신, 이 상수들을 미리 계산하여 **하나의 상수로 정의(Define)**해두는 것이 좋습니다.
>
> * `ACCEL_SCALE = 9.80665 / 16384.0f` (약 0.000598...)
> * `GYRO_SCALE = (3.141592 / 180.0f) / 131.0f`
>
> 이렇게 하면 `raw * ACCEL_SCALE` 처럼 **곱셈 한 번**으로 변환이 끝나 CPU 사이클을 아낄 수 있습니다.
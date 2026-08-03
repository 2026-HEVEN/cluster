# HEVEN CAN 프로토콜 명세서 (단일 출처)

> **이 문서는 VCU·Cluster 양 레포가 공유하는 단일 출처(single source of truth)입니다.**
> 코드(`include/can_protocol.h`)와 이 문서는 항상 일치해야 하며, 수정 시 **양 레포에 동일하게 반영**합니다. (담당: 김도현)
> 출처: `EZkontrol-CANBUS-MCU-to-VCU.pdf V1.0`, `EZkontrol-CANBUS-MCU-to-METER.pdf V1.1`, 시스템 설계.

---

## 1. 버스 파라미터

| 항목 | 값 |
|------|-----|
| 비트레이트 | **250 kbps** |
| 프레임 | CAN 2.0B **확장(29비트) 식별자** |
| 바이트 순서 | **리틀엔디언** (예: 0x1234 → 0x34 먼저, 0x12 나중) |
| 토폴로지 | 데이지체인, 양 끝단 120Ω 종단 |
| 표준 | SAE J1939 기반 |

---

## 2. 29비트 식별자 구조 (J1939)

```
 28..26   25   24   23..16     15..8       7..0
┌────────┬────┬────┬────────┬──────────┬────────────┐
│PRIORITY│ R  │ DP │  PF    │   PS     │     SA     │
│ 3bit   │ 0  │ 0  │ 8bit   │  8bit    │   8bit     │
└────────┴────┴────┴────────┴──────────┴────────────┘
```
- **PRIORITY**: 0~7 (값 작을수록 우선). 제어명령=3, 피드백=6.
- **PF (PDU Format)**: 메시지 종류. PF<240이면 PS=목적지 주소(DA), PF≥240이면 PS=그룹확장.
- **PS**: 목적지 노드 주소 (PF<240일 때).
- **SA**: 송신 노드 주소.

> 예) `0x0C01EFD0` → PRIORITY=3, PF=0x01, PS=0xEF(MCU1 목적지), SA=0xD0(VCU 송신).
> 즉 **"VCU가 MCU1에게 보내는 1번 메시지"**.

---

## 3. 노드 주소 (SA)

### HEVEN 시스템에서 실제 사용

| 노드 | SA (HEX) | SA (DEC) | 비고 |
|------|----------|----------|------|
| Energy_Meter / **METER** | 0x17 | 23 | EZkontrol 표준 계기 주소 |
| **Cluster_ESP32** | 0xC0 | 192 | 계기판 HMI (시스템 설계) |
| **VCU_ESP32** | 0xD0 | 208 | 차량 제어 |
| **Controller_L (MCU1)** | 0xEF | 239 | 좌측 모터컨트롤러 (기본값) |
| **Controller_R (MCU2)** | 0xF0 | 240 | 우측 — **EZkontrol 앱에서 SA=0xF0 수동 설정 필요** |

### EZkontrol 표준 전체 주소표 (참고)

| 노드 | 주소 |
|------|------|
| METER | 23 (0x17) |
| VCU | 208 (0xD0) |
| MCU1~4 | 239~242 (0xEF~0xF2) |
| BMS1~4 | 243~246 (0xF3~0xF6) |
| GLOBAL | 255 (0xFF) |

> **N.B.** MCU의 SA는 호스트(EZkontrol 앱)에서 "controller number"로 설정. 기본 SA = 239(0xEF).
> Controller_R은 반드시 240(0xF0)으로 바꿔야 좌우가 충돌하지 않음.

---

## 4. 메시지 ID 일람 (HEVEN)

| 방향 | 메시지 | Controller_L (0xEF) | Controller_R (0xF0) | 주기 | Prio |
|------|--------|---------------------|---------------------|------|------|
| VCU → MCU | 제어명령 / 핸드셰이크응답 | `0x0C01EFD0` | `0x0C01F0D0` | 50ms | 3 |
| MCU → VCU | 피드백 Part I (전압/전류/속도) | `0x1801D0EF` | `0x1801D0F0` | 50ms | 6 |
| MCU → VCU | 피드백 Part II (온도/상태/에러) | `0x1802D0EF` | `0x1802D0F0` | 50ms | 6 |
| MCU → METER | 계기 메시지 I | `0x180117EF` | `0x180117F0` | 100ms | 6 |
| MCU → METER | 계기 메시지 II | `0x180217EF` | `0x180217F0` | 100ms | 6 |
| Cluster → VCU | 커맨드 (패독/TC/회생/디버그) | `0x1801D0C0` (신규) | — | 100ms | 6 |
| VCU → Cluster | 휠스피드 RPM | `0x1802C0D0` (신규) | — | 50~100ms 권장 | 6 |
| Cluster → TMA-1/logger | BMS 상태 요약 | `0x18F3FFC0` (신규) | — | 100ms | 6 |
| Cluster → TMA-1/logger | BMS 상세 요약 | `0x18F4FFC0` (신규) | — | 100ms | 6 |

> ID에서 PS(목적지)·SA(송신)만 컨트롤러별로 바뀜. 위 표의 ID는 `PF<<16 | PS<<8 | SA`로 조립됨(+ Priority).

---

## 5. 메시지 상세

### 5.1 VCU → MCU : 제어명령  `0x0C01EFD0` (L) / `0x0C01F0D0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 | 범위 |
|--------|------|--------|--------|------|
| 0~1 | Target Phase Current (토크) | 0.1 A/bit | −3200 A | −3200~3200 A |
| 2~3 | Target Speed | 1 rpm/bit | −32000 | −32000~32000 rpm |
| 4 | Command Controls | — | — | bit0: 0=HALTED / 1=RUNNING<br>bit1: 0=토크모드 / 1=속도모드<br>bit7-2: 예약 |
| 5,6 | 예약 | — | — | 0 |
| 7 | Life signal | — | — | 0~0xFF (매 프레임 +1) |

**토크 스케일링**: `raw16 = (target_A + 3200) × 10` (리틀엔디언). 음수 = 회생제동.
→ 0A=32000, +32A=32320, −32A=31680. **`include/can_protocol.h`의 `torque_to_raw()`와 동일.**

### 5.2 VCU → MCU : 핸드셰이크 응답  `0x0C01EFD0` / `0x0C01F0D0`

8바이트 **전부 `0xAA`**. (MCU의 0x55 요청에 대한 응답 — §6 참고)

### 5.3 MCU → VCU : 피드백 Part I  `0x1801D0EF` (L) / `0x1801D0F0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 | 범위 |
|--------|------|--------|--------|------|
| 0~1 | Bus Voltage | 0.1 V/bit | 0 | 0~300 V |
| 2~3 | Bus Current | 0.1 A/bit | −3200 A | −3200~3200 A |
| 4~5 | Phase Current | 0.1 A/bit | −3200 A | −3200~3200 A |
| 6~7 | Speed | **1 rpm/bit** | −32000 | −32000~32000 rpm |

### 5.4 MCU → VCU : 피드백 Part II  `0x1802D0EF` (L) / `0x1802D0F0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 |
|--------|------|--------|--------|
| 0 | Controller Temperature | 1 ℃/bit | −40 ℃ |
| 1 | Motor Temperature | 1 ℃/bit | −40 ℃ |
| 2 | STATUS — bit0: HALTED/RUNNING, bit1: 토크/속도모드 | — | — |
| 3 | ERROR1 비트맵 (아래) | — | — |
| 4 | ERROR2 비트맵 (아래) | — | — |
| 5 | ERROR3 비트맵 (아래) | — | — |
| 6 | 예약 | — | — |
| 7 | Life signal | — | 0~0xFF |

**ERROR 비트맵** (각 비트 0=정상 / 1=에러):
- **Byte3**: 0 과전류, 1 과부하, 2 과전압, 3 저전압, 4 컨트롤러과열, 5 모터과열, 6 모터스톨, 7 모터결상
- **Byte4**: 0 모터센서, 1 모터보조센서, 2 엔코더정렬불량, 3 폭주방지작동, 4 메인가속, 5 보조가속, 6 프리차지, 7 DC컨택터
- **Byte5**: 0 전력밸브, 1 전류센서, 2 오토튠, 3 RS485, 4 CAN, 5 소프트웨어

### 5.5 MCU → METER : 계기 메시지 I  `0x180117EF` · 100ms

> Cluster가 METER(0x17)로 동작할 때 직접 수신 (§7 경로 결정 참고).

| 바이트 | 항목 | 분해능 | 오프셋 |
|--------|------|--------|--------|
| 0~1 | Bus Voltage | 0.1 V/bit | 0 |
| 2~3 | Bus Current | 0.1 A/bit | −3200 A |
| 4~5 | Phase Current | 0.1 A/bit | −3200 A |
| 6~7 | Speed | **0.1 rpm/bit** ⚠️ | −32000 |

> ⚠️ **METER 경로의 Speed는 0.1rpm/bit** — VCU 경로(5.3, 1rpm/bit)와 분해능이 다르다. 혼동 주의.
> (PDF 본문의 PS 표기 "0x11"은 오타이며, ID값 `0x180117EF`가 정답 — PS=0x17=METER.)

### 5.6 MCU → METER : 계기 메시지 II  `0x180217EF` · 100ms

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0 | Controller Temperature | 1 ℃/bit, offset −40 |
| 1 | Motor Temperature | 1 ℃/bit, offset −40 |
| 2 | Accelerator Opening | 1 %/bit (0~100%) |
| 3 | STATUS — bit2-0 Gear(0:N/A,1:R,2:N,3:D1,4:D2,5:D3,6:S,7:P), bit3 Brake(0/1), bit6-4 Operation Mode(2:Cruise,3:EBS,4:Hold), bit7 DC Contactor(0:OFF/1:ON) | — |
| 4 | ERROR1 (5.4 Byte3과 동일 비트맵) | — |
| 5 | ERROR2 (5.4 Byte4와 동일) | — |
| 6 | ERROR3 (5.4 Byte5와 동일) | — |
| 7 | bit7-4: Life signal | 0~0xFF |

### 5.7 Cluster → VCU : 커맨드  `0x1801D0C0` (신규 할당) · 100ms

> 계기판 config 입력(패독·TC·4단 회생제동 로터리·디버그)을 VCU에 전달. **EZkontrol 표준이 아닌 HEVEN 자체 정의.**
> 기어 스위치는 Cluster ESP32에 직접 연결하지 않고 VCU 쪽 ADC/pass-through 회로에서 읽는다.
> PF=0x01, PS=0xD0(VCU), SA=0xC0(Cluster). MCU→VCU(0x1801D0EF)와 SA로 구분되어 충돌 없음.
> 인코딩 구현: Cluster 펌웨어 `encode_cluster_command()`. 아래 레이아웃과 일치.

| 바이트 | 항목 | 의미 |
|--------|------|------|
| 0 | Reserved | 0 |
| 1 | Config flags | bit0: TC enabled, bit2-1: Regen rotary level(0~3), bit3: Debug enabled, bit7-4: reserved(0) |
| 2 | Flags | bit0: Paddock request, bit7-1: reserved(0) |
| 3~7 | 예약 | 0 |

> ⚠️ 패독은 **요청 신호**일 뿐. VCU가 토크/속도를 상한 이하로 클램프하고 CAN 끊김 시 fail-safe(제한 유지)를 결정해야 함.
> VESS 스위칭은 Cluster GPIO 입력과 이 CAN 커맨드에서 제외한다. VESS 신호 경로/스위칭은 별도 하드웨어 또는 VCU 쪽 계약으로 관리한다.

### 5.8 VCU → Cluster : 표시 상태  `0x1801C0D0` (HEVEN 정의) · 50~100ms 권장

> Cluster가 스위치 입력값이 아니라 **VCU가 확정한 차량 상태**를 표시하기 위한 프레임.
> 현재 Cluster 펌웨어는 이 프레임이 들어오면 기어/HV/브레이크를 갱신한다. SOC는 선택 사항이며, 현재 주 경로는 Cluster의 BLE BMS 직접 수신이다.
> VCU 쪽 구현 전에는 Cluster가 기어를 확정할 수 없으므로 기본 N 표시로 남는다.

| 바이트 | 항목 | 의미 |
|--------|------|------|
| 0 | Gear display | 0:N, 1:R, 2:D, 3:P |
| 1 | Flags | bit0: Brake, bit1: HV active, bit2: SOC valid |
| 2 | SOC percent | 0~100, bit2(SOC valid)=1일 때만 유효 |
| 3~6 | 예약 | 0 |
| 7 | Life signal | 0~0xFF |

> BMS SOC는 현재 `LWS-1608` BLE BMS를 Cluster ESP32가 직접 polling해서 표시한다.
> VCU가 추후 SOC를 확정값으로 보내야 하는 경우에만 이 프레임의 `SOC valid`와 `SOC percent`를 사용한다. VCU 프레임에서 SOC valid가 0이어도 BLE로 받은 SOC는 지우지 않는다.

### 5.9 VCU → Cluster : 휠스피드 RPM `0x1802C0D0` (HEVEN 정의) · 50~100ms 권장

> VCU가 휠스피드 센서 RPM 값을 Cluster ESP32에 전달하는 프레임이다.
> Cluster는 이 값을 타이어 지름 0.4597m 기준 km/h로 변환하고, 네 바퀴 중 최솟값/최댓값을 제외한 가운데 두 바퀴 평균을 대표 차량 속도로 LCD에 표시한다.
> 300ms 이상 수신되지 않으면 Cluster 표시 속도는 0km/h로 내려간다.

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0~1 | Front Left wheel RPM | uint16 little-endian, 1 rpm/bit |
| 2~3 | Front Right wheel RPM | uint16 little-endian, 1 rpm/bit |
| 4~5 | Rear Left wheel RPM | uint16 little-endian, 1 rpm/bit |
| 6~7 | Rear Right wheel RPM | uint16 little-endian, 1 rpm/bit |

속도 변환식:

```text
wheel_kph = wheel_rpm * pi * wheel_diameter_m * 0.06
wheel_diameter_m = 0.4597
vehicle_kph = average(middle two wheel_kph values after dropping min and max)
```
### 5.10 Cluster → TMA-1/logger : BMS 상태 요약 `0x18F3FFC0` (HEVEN 정의) · 100ms

> Cluster가 BLE로 직접 수신한 LWS-1608 BMS 표시용 정보를 TMA-1/logger에 전달한다.
> 이 프레임은 **차량 제어용 안전판단에 사용하지 않는다.** BLE가 끊기면 valid bit가 0으로 내려가며, 수신기는 해당 값을 stale/display-only로 취급한다.
> PF=0xF3, PS=0xFF, SA=0xC0. Broadcast 성격이므로 TMA-1/logger는 버스에서 수신만 하면 된다.

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0 | Flags | bit0: BMS data valid, bit1: BLE connected, bit7-2: reserved(0) |
| 1 | SOC | 1 %/bit, 0~100 |
| 2~3 | Pack voltage | 0.1 V/bit, little-endian |
| 4~5 | Pack current | 0.1 A/bit, offset −3200 A, little-endian |
| 6 | BMS temperature | 1 ℃/bit, offset −40 ℃ |
| 7 | Life signal | 0~255 |

### 5.11 Cluster → TMA-1/logger : BMS 상세 요약 `0x18F4FFC0` (HEVEN 정의) · 100ms

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0 | Flags | bit0: BMS data valid, bit1: BLE connected, bit7-2: reserved(0) |
| 1 | SOH | 1 %/bit, 0~100 |
| 2~3 | Remaining capacity | 1 mAh/bit, little-endian |
| 4~5 | Cycle count | 1 cycle/bit, little-endian |
| 6 | Reserved | 0 |
| 7 | Life signal | 0~255 |

---

## 6. 핸드셰이크 & 타임아웃 (EZkontrol 규칙)

```
[전원 ON]
   │  MCU가 0x1801D0EF 로 8바이트 0x55 를 50ms(20Hz)마다 송신
   ▼
VCU가 0x55 수신 ──► VCU가 0x0C01EFD0 로 8바이트 0xAA 응답
   │
   ▼  핸드셰이크 성립
MCU: 메시지 I·II 송신 시작 + VCU 제어명령(메시지 I) 대기·실행
```

**통신 실패 판정 (MCU 측):**
- VCU 제어명령(0x0C01EFD0)을 **10회 연속** 못 받거나
- Life signal **5회 연속** 실패

→ MCU 셧다운 후 핸드셰이크 재시도. (그래서 VCU는 50ms 라이프/제어 프레임을 **절대 끊지 말 것** — 펌웨어의 전용 FreeRTOS 라이프 태스크가 이를 보장.)

---

## 7. 계기 데이터 수신 경로 — **확정: C. VCU 모드 수신 스니핑**

EZkontrol 컨트롤러는 **METER 모드**일 때 피드백을 `0x1801xxEF`(VCU용)가 아니라
`0x180117EF`/`0x180217EF`(METER용, §5.5/5.6)로 보낸다.

~~A. Cluster=METER 직수신~~ / ~~B. VCU 게이트웨이~~ 대신 다음 경로로 확정:

**C. 컨트롤러를 VCU 모드로 설정 → Cluster가 그 VCU행 프레임(`0x1801D0EF/F0`, `0x1802D0EF/F0`)을 같은 버스에서 그대로 수신(스니핑)한다.**

- CAN은 멀티드롭 버스라 VCU 목적지로 보낸 프레임도 버스에 물린 모든 노드(Cluster 포함)에 물리적으로 도달한다.
  Cluster는 그 프레임을 **읽기만** 하면 되고, VCU의 게이트웨이 재방송이나 컨트롤러의 METER 모드 설정이 필요 없다.
- **필수 사전조건**: 두 컨트롤러 모두 EZkontrol 앱에서 VCU 모드로 설정, Controller_R은 SA=0xF0으로 설정.
  (METER 모드로 남아있으면 Cluster는 아무 것도 못 받는다 — `0x1801D0xx`가 아니라 `0x180117xx`로 나가기 때문.)
- Cluster는 컨트롤러 피드백을 **재방송하지 않고 읽기만** 한다. 단, 자체 HMI/BMS 정보는 별도 HEVEN 정의 프레임(`CAN_ID_CLUSTER_CMD`, `CAN_ID_CLUSTER_BMS_STATUS`, `CAN_ID_CLUSTER_BMS_DETAIL`)으로 송신한다.
- 구현: `src/core/can_bus.cpp`의 `poll_rx()`가 `CAN_ID_FB1_L/R`, `CAN_ID_FB2_L/R`을 필터링해 `state`의 `_r` 접미사 필드(Controller_R)와 기본 필드(Controller_L)에 각각 채운다.
- 기어/HV처럼 VCU가 최종 확정해야 하는 표시값은 `CAN_ID_VCU_CLUSTER_STATUS`(0x1801C0D0)를 통해 받는다. SOC는 현재 BLE BMS 직접 수신을 우선 사용한다.

---

## 8. 코드 매핑 (`include/can_protocol.h`)

현재 구현된 것:
```cpp
constexpr uint32_t CAN_ID_TORQUE_L = 0x0C01EFD0;  // VCU→MCU1
constexpr uint32_t CAN_ID_TORQUE_R = 0x0C01F0D0;  // VCU→MCU2
uint16_t torque_to_raw(float amps);   // (amps+3200)*10
float    raw_to_torque(uint16_t raw);
// SA 상수: SA_VCU=0xD0, SA_CLUSTER=0xC0, SA_CONTROLLER_L=0xEF, SA_CONTROLLER_R=0xF0, SA_ENERGY_METER=0x17

// 피드백 ID (VCU행 프레임을 Cluster가 스니핑, §7)
constexpr uint32_t CAN_ID_FB1_L = 0x1801D0EF;   // Part I (전압/전류/속도)
constexpr uint32_t CAN_ID_FB2_L = 0x1802D0EF;   // Part II (온도/상태/에러)
constexpr uint32_t CAN_ID_FB1_R = 0x1801D0F0;
constexpr uint32_t CAN_ID_FB2_R = 0x1802D0F0;
constexpr uint32_t CAN_ID_CLUSTER_CMD = 0x1801D0C0;
constexpr uint32_t CAN_ID_VCU_CLUSTER_STATUS = 0x1801C0D0;
constexpr uint32_t CAN_ID_VCU_WHEEL_SPEEDS = 0x1802C0D0;
constexpr uint32_t CAN_ID_CLUSTER_BMS_STATUS = 0x18F3FFC0;
constexpr uint32_t CAN_ID_CLUSTER_BMS_DETAIL = 0x18F4FFC0;

// 디코더: raw_to_voltage(×0.1), raw_to_current(×0.1,-3200), raw_to_temp(-40), raw_to_speed(1rpm/bit,-32000)
// RX 파싱 구현: src/core/can_bus.cpp poll_rx() — 컨트롤러 피드백, VCU 표시 상태, VCU 휠스피드 프레임을 state에 반영
```

METER 경로(§5.5/5.6, 0.1rpm/bit)는 이 설계에서 사용하지 않는다 — 컨트롤러가 VCU 모드로 고정되므로 필요 없음.

---

## 9. 변경 관리

- 이 파일과 `include/can_protocol.h`는 **글자 단위로 일치**해야 한다.
- 수정 시: ① 이 문서 갱신 → ② 양 레포의 `can_protocol.h` 동기화 → ③ 변경 요약을 팀 공지.
- 새 메시지 ID는 J1939 규칙(PF/PS/SA)에 맞게 할당하고 §4 표에 추가.

# 추가 CAN 정보 및 실차 확인 사항

기준: 이번 Cluster 로컬 구현과 VCU `dev`의 `8498e2f20cfa79f8c71416a25a2676677b59a270`에서 확인한 Car Check v1 계약. 다른 팀에 메시지를 발송하거나 VCU 코드를 변경한 것은 아니다.

## 1. VCU팀에 요청할 내용

현재 Cluster에는 Steering, IMU, 4륜 개별 휠 속도, TV·회생제동·Paddock 진단 프레임을 받는 기능을 연결했습니다. 아래 네 프레임을 새로 만들어 달라는 요청은 필요하지 않습니다. 실제 차량에 해당 송신 버전이 업로드되어 있는지 확인 부탁드립니다.

| CAN ID | 내용 | 이번 Cluster 처리 |
| --- | --- | --- |
| `0x1804C0D0` | 정규화 조향값, valid, life | 부호 있는 int16 ×0.001. 각도가 아님. |
| `0x1805C0D0` | Yaw rate, 가속도 X/Y, valid, life | int16 ×0.01. deg/s, g. |
| `0x1806C0D0` | FL/FR/RL/RR 개별 속도 | uint16 ×0.1 km/h, `0xFFFF`는 무효. |
| `0x1807C0D0` | 요청 수신값, 실제 제어 상태, 차단 이유 | v1 확인 후 표시. 요청과 적용을 구분. |

모두 Extended CAN, DLC 8, Little Endian, 공칭 50 ms/20 Hz이며, Cluster는 300 ms 초과 미수신 시 오래된 정보로 처리합니다. Steering/IMU의 유효성 표시 비트가 없는 구버전은 `UNKNOWN`으로 표시합니다.

추가로 필요한 항목은 다음과 같습니다.

1. **브레이크 압력:** 현재 확인한 VCU에는 보정된 압력 측정값과 CAN 송신 정의가 없습니다. 센서 장착 여부·모델·측정 범위, ADC 입력, 영점/스팬 보정, bar 또는 MPa 단위 값을 먼저 확정해 주세요. 이후 CAN ID, 바이트 위치, 배율, invalid 값, valid 비트, 송신 주기를 공유해 주세요. 기존 Brake Active 또는 0/100% 디지털 값으로 압력을 대신 표시할 수는 없습니다.
2. **조향/IMU 해석:** 조향 좌·우 부호, 정규화 양 끝의 실제 의미, IMU 설치 방향과 차량 X/Y/Z 축·yaw 부호를 공유해 주세요. 현재 물리 단위는 알지만 차량 장착 방향까지 확인된 것은 아닙니다.
3. **개별 휠 속도 유효성:** `0xFFFF` 처리 및 센서 고장 검출 조건, 타이어 둘레 보정이 실제 펌웨어와 일치하는지 확인해 주세요. 유효한 숫자 0이 수신된다는 것만으로 센서 배선 정상까지 입증되지는 않습니다.
4. **적용 상태 해석:** TV Active는 제어 경로와 출력 허용 상태, Regen Active는 반대 부호 명령의 활성 상태입니다. 실제 차동 토크 또는 배터리 충전 전력의 측정값과 같은 의미가 아님을 함께 확인해 주세요.

기어, 스로틀, 브레이크 ON/OFF, HV 활성, Paddock 상태 및 대표 WSS 차속은 기존 수신 경로를 사용합니다. 모터 전압·버스 전류·상 전류·RPM·온도·고장 정보는 모터컨트롤러 직접 송신값을 사용하므로 VCU에 중복 송신을 요청하지 않습니다.

## 2. 전원·에너지미터 담당 확인 사항

| 항목 | 현재 가능한 표시 | 추가 확인 또는 데이터 |
| --- | --- | --- |
| HV SOC | BMS BLE에서 받은 %와 막대 | 현재 BMS의 SOC 유효성 확인 |
| HV/LV 전압 | EM RECORD 수신 시 숫자와 이력 그래프 | LV 측정 지점이 LV 배터리 단자와 같은 전위인지 확인 |
| 전력 크기 | 동일 EM RECORD의 전압×전류 절댓값, kW | 공통 HV 경로 설치 여부, 실제 송신·수신 주기 |
| 충전/방전 방향 | 임시로 전류 0 이상=POWER, 0 미만=CHARGE | 알려진 구동·회생 조건에서 EM 전류 부호 확인. 반대이면 UI 매핑 변경 |
| 충전 막대 기준 | 아직 미확정 | 허용 충전 전력/전류와 별도 표시 범위 |
| LV SOC | 제공되지 않음 | 별도 LV BMS의 SOC 또는 검증된 잔량 추정 입력 필요 |

전압 한 값만으로 LV SOC를 정확한 %로 환산할 수 없습니다. 배터리 화학계·직렬 수·전압 곡선·부하/온도 영향 정보도 없으며, 현재 EM에는 LV 전류·용량·적산 SOC가 없습니다. 따라서 LV SOC 막대와 %를 생성하지 않습니다.

## 3. TMA-1 담당에게 전달할 내용

제공된 `107signals (1).json` 이후 갱신분과 대조해 아래 항목을 보완해 주세요. 이미 추가했다면 중복 생성할 필요는 없습니다. 이번 작업에서는 JSON을 수정하지 않았습니다.

| 대상 | Decoder 기준 |
| --- | --- |
| Steering | `1804C0D0`, Byte0~1 signed LE ×0.001, Byte6 bit0 valid, bit7 validity-present, Byte7 life |
| IMU | `1805C0D0`, Byte0~1 yaw, 2~3 ax, 4~5 ay: signed LE ×0.01. Byte6 bit0 yaw-valid, bit1 accel-valid, bit7 validity-present, Byte7 life |
| 개별 WSS | `1806C0D0`, Byte0~1 FL, 2~3 FR, 4~5 RL, 6~7 RR: unsigned LE ×0.1 km/h. `65535` raw는 invalid |
| Control | `1807C0D0`, Byte0 version=1, Byte1 bits0/1/2 TV/RGN/PDK 요청 수신값 |
| Control 적용 | 같은 ID Byte2 bit0 TV active, bit1 regen available, bit2 regen active, bit3 Paddock active, bit4 output allowed, bit5 Cluster fresh, bit6 brake sensor installed |
| Control 이유 | 같은 ID Byte3 TV 차단 비트, Byte4 regen 차단 비트, Byte7 life. 각 비트는 VCU 계약 문서 참조 |
| EM RECORD | `1CF5FFC1`, Byte0~1 HV V signed ×0.1, 2~3 HV A signed ×0.1, 4~5 LV V signed ×0.01, 6~7 CPU °C signed ×0.01, 모두 LE |
| EM SYNC | `1CF6FFC1`, Byte0~3 uptime uint32 ms, 4~5 record count uint16, Byte6 bits0 sd_ok/1 header_seen/2 uart_alive/3 drops, Byte7 life |
| 기존 항목 보완 | VCU 스로틀 valid/Paddock 적용 상태, BMS valid/BLE/life 및 Detail 프레임은 최신 기존 계약과 대조 |

Decoder만 추가되어 있다고 실차 수신이 확인된 것은 아닙니다. 수신 시각, valid, stale 및 invalid sentinel을 함께 해석해야 합니다. 조건부 필터를 지원하지 않는 도구에서는 분석 단계에서 무효 값을 제외해 주세요.

## 근거

- [VCU Car Check 계약](https://github.com/2026-HEVEN/vcu/blob/8498e2f20cfa79f8c71416a25a2676677b59a270/docs/CAR_CHECK_CAN.md)
- [VCU 송신 코드](https://github.com/2026-HEVEN/vcu/blob/8498e2f20cfa79f8c71416a25a2676677b59a270/src/core/can_bus.cpp)
- [VCU 입력 핀 정의](https://github.com/2026-HEVEN/vcu/blob/8498e2f20cfa79f8c71416a25a2676677b59a270/src/core/board_pins.h)
- Cluster `include/car_check_protocol.h`, `src/modules/car_check_receiver.cpp`, `src/core/check_snapshot.cpp`, `src/core/can_bus.cpp`.
- 사용자가 제공한 EM RECORD/SYNC 사양, `107signals (1).json`, `BMS·모터컨트롤러 전류 기반 10 kW 제한` 문서. 실제 설치·부호·주기는 실차 확인 대상으로 남겨 둡니다.

# HEVEN Cluster 펌웨어

## N 기어 최초 구동 조건 (2026-10-02)

VCU는 부팅 후 처음 구동 준비를 완료할 때 실제 선택기의 N 안정 입력과 기존 스로틀 해제·통신 조건을 요구합니다. 준비 완료 후 D/R 주행 및 일시 오류 복구에는 N 재선택을 요구하지 않습니다.
Car Check의 `START / N CHECK`는 START ADC 입력과 수신 기어의 N 여부(`N YES`, `N NO`, 수신 불명 `N WAIT`)를 보여줍니다. 이는 VCU 구동 허가 표시가 아닙니다.
PCB V3의 GPIO34는 START **입력 감시 전용**이며 Cluster에 HV 릴레이 허가 출력은 없습니다. 따라서 이 펌웨어는 D/R에서 실제 HV 릴레이가 붙는 것을 차단하지 않습니다. 이를 막으려면 LV PCB/시동 회로의 인터록 경로가 필요합니다.

**2026 영광 대회 · 계기 클러스터(Instrument Cluster)** — ESP32가 CAN/BLE로 차량 상태(속도·전압·SOC·온도·에러)를 **받아서** LCD에 표현하고, 계기판 버튼 입력을 VCU로 보냅니다. PCB V3에서는 VESS PWM도 Cluster가 직접 출력합니다. VCU와 달리 **안전 임계가 약한 표시 전용 보드**입니다.

> 🤖 **AI 에이전트/팀원은 [`AGENTS.md`](AGENTS.md)를 먼저 읽으세요** — 무엇을 고쳐도 되고 무엇을 건드리면 안 되는지 규칙이 있습니다.

---

## 한눈에

- **스택**: PlatformIO + Arduino-ESP32, TWAI(CAN), ILI9341 LCD. 화면은 **패널 독립적 1bpp 프레임버퍼**(위젯이 그림) + `display_blit`(ILI9341로 blit)
- **구조**: 잠긴 코어(CAN·프레임버퍼·blit) + 팀원이 채우는 순수 모듈(`src/modules/`). VCU와 **동일한 2층 설계**지만 안전 FSM·50ms 라이프 태스크가 없고 CAN은 **수신(RX) 위주**입니다.
- **상태**: ESP32 빌드 성공, 호스트 native 테스트 111개 통과

## 사용자 문서

- [계기판 기능 및 한계](docs/ui_report/REPORT.md)
- [계기판 사용설명서](docs/ui_report/USER_MANUAL.md)
- [추가 CAN 정보 및 실차 확인 사항](docs/ui_report/CAN_DATA_REQUIRED.md)

## 빠른 시작

```bash
# 1. 이 레포를 깨끗한 위치에 클론
git clone https://github.com/2026-HEVEN/cluster.git
cd cluster

# 2. PlatformIO 설치 (한 번만) — VS Code면 PlatformIO IDE 확장 설치로 대체 가능
uv tool install platformio      # 또는: pipx install platformio

# 3. 노트북에서 테스트 (하드웨어 불필요)
pio test -e native

# 4. 보드에 빌드 & 업로드 (ESP32 연결 상태에서)
pio run -e esp32dev -t upload
```

> 🧪 **테스트가 처음이거나 Windows 사용자라면** → 노션 [펌웨어 테스트 실행 방법](https://www.notion.so/390913e532e68199a9b5e340b73e9e71) 참고. AI에 복붙할 프롬프트 + "이렇게 나오면 성공" 출력 예시 + Windows(WSL2/MinGW) 셋업까지 있습니다. (보드 업로드는 Windows도 그냥 되고, `native` 단위테스트만 host 컴파일러가 필요해요.)

> ℹ️ 모터컨트롤러 값은 MCU→VCU 피드백 프레임을 Cluster가 같은 CAN 버스에서 수신합니다. BMS SOC는 `LWS-1608` BLE BMS에 Cluster ESP32가 직접 연결해 표시용 telemetry로만 읽습니다.
> ℹ️ 계기판 속도 표시는 VCU가 내부 `vehicle_speed_compute()`로 산출한 단일 차량속도 프레임 `0x1803C0D0`을 사용합니다. 이 값은 전륜 기준 보정 로직을 거친 `km/h x 10` 값이며, Cluster는 다시 네 바퀴 평균을 내지 않습니다.
> ℹ️ GPS Lap Start는 현재 GNSS fix 위치를 출발점으로 저장합니다. 이후 VCU 단일 차량속도 `0.5 km/h` 초과가 `150ms` 이상 지속되면 랩타이머를 시작하고, 다시 출발점 반경 `2.0m` 안으로 들어오면 랩을 갱신합니다.


## 현재 하드웨어 핀맵 (PCB V3)

| 구분 | 기능 | ESP32 GPIO | 연결/동작 |
|------|------|------------|-----------|
| CAN | TXD | GPIO13 | CAN 트랜시버 TXD |
| CAN | RXD | GPIO14 | CAN 트랜시버 RXD |
| LCD ILI9341 | CS / RST / DC | GPIO23 / 22 / 21 | LCD 제어 |
| LCD / Touch SPI | MOSI / SCK / MISO | GPIO17 / 18 / 35 | 공용 SPI |
| LCD Touch XPT2046 | T_CS | GPIO16 | Touch chip select |
| GPS ZED-F9P | RX / TX / PPS | GPIO25 / 26 / 27 | 115200 baud NMEA/UBX, RTCM3 및 PPS |
| VESS | PWM | GPIO4 | 반전 MOSFET 구동, 50Hz 고정. 외부 RX-TH에는 1~2ms High 펄스 출력 |
| HMI | TV | GPIO32 | 토글 스위치, PCB 외부 10k 풀업, ON=LOW |
| HMI | Paddock | GPIO33 | 토글 스위치, PCB 외부 10k 풀업, ON=LOW |
| HMI | HOME | GPIO19 | 순간 푸시 버튼, PCB 외부 10k 풀업, ON=LOW |
| HMI | GPS Lap | GPIO5 | 순간 푸시 버튼, PCB 외부 10k 풀업, ON=LOW |
| HMI | Regen bit0 / bit1 | GPIO36 / 39 | 로터리, ON=LOW, 두 핀 모두 외부 풀업 필요 |
| Sense | START_IN | GPIO34 | 시동 전원 감지 ADC. 버튼 출력 핀이 아님 |

GPS TX2는 PCB V3에서 Cluster ESP32 `GPIO25` RX에 연결한다.

Cluster는 부팅 시 ZED-F9P UART1/UART2가 460800 baud로 설정되어 있더라도 UBX 설정으로 115200 baud로 낮춘 뒤 통신한다. NMEA가 끊기면 115200 baud 복구를 다시 시도하므로, 같은 GPS TX에 연결된 외부 로거도 115200 baud로 유지할 수 있다.

RTK 사용 시 Cluster ESP32가 Wi-Fi로 NTRIP caster에 접속하고, 수신한 RTCM3 바이트를 가공 없이 `GPIO26` UART TX로 ZED-F9P RX2에 전달한다. 실제 Wi-Fi/NTRIP 계정정보는 `include/ntrip_secrets.h`에 넣고 Git에는 올리지 않는다. `include/ntrip_secrets.example.h`를 복사해서 사용한다.

Cluster는 ZED-F9P RMC의 Speed Over Ground를 km/h로 변환해 새 RMC마다 Extended CAN `0x18F9FFC0`으로 송신한다. 속도는 0.01 km/h/bit이며 같은 프레임에 RMC fresh, GPS fix, RTK FLOAT/FIXED, speed valid, GGA quality와 RMC age를 포함한다. 상세 byte layout은 `docs/CAN_PROTOCOL.md`를 따른다.

회생제동 입력은 GPIO36/39 두 비트 로터리다. 0단은 OFF, 1~3단은 모두 같은 ON 요청으로 인코딩한다. 실제 회생 가능 여부와 전류 제한은 VCU가 최종 판단한다.

GPIO15는 현재 펌웨어에서 사용하지 않는다. GPIO15는 strapping pin이므로 외부 회로가 부팅 순간 강하게 잡아당기지 않게 주의한다.

GPIO34는 PCB V3의 START_IN 전원 감지 ADC다. 5ms마다 분압된 핀 전압을 읽고, 1.5V 이상에서 ON, 1.0V 이하에서 OFF로 판정하며 20ms 디바운스를 적용한다. 이 값은 Car Check의 VCU/SENSOR 상세 화면에 표시할 뿐 차량 제어에는 사용하지 않는다. 기어는 VCU에서 읽는다.
## 어디서 작업하나

| 폴더 | 내용 | 편집? |
|------|------|-------|
| `src/modules/` | 순수 계산 함수 `xxx_compute()` | ✅ **여기서만** |
| `test/` | 노트북 단위 테스트 | ✅ |
| `src/core/`, `src/logic/`, `include/` | CAN·framebuffer·display_blit·스케줄러·타입 | 🔒 잠김 |
| `platformio.ini`, `src/main.cpp` | 빌드 설정·진입점 | 🔒 잠김 |

새 모듈을 추가하거나 기존 `compute()`를 채우는 법 → [`docs/ADDING_A_MODULE.md`](docs/ADDING_A_MODULE.md)

## 모듈 목록 (FILL-IN)

- `hmi_input` — config 스위치(패독·TC·회생·디버그) → `ClusterCommand` (CAN으로 VCU에 전송)
- `widgets/` — 계기판 화면 위젯 (speed·battery·warnings·gear ...). 1bpp 프레임버퍼에 그림.
  새 위젯 추가법: `docs/ADDING_A_WIDGET.md`

> 표시는 패널 독립적 **1bpp 프레임버퍼**로. 실제 패널 blit(`display_blit`)은 하드웨어 확정 후 구현.

## 문서

| 문서 | 내용 |
|------|------|
| [`AGENTS.md`](AGENTS.md) / `CLAUDE.md` | 작업 규칙 (에이전트·팀원 필독) |
| [`docs/ADDING_A_MODULE.md`](docs/ADDING_A_MODULE.md) | 모듈 추가/작성 절차 |
| [`docs/CAN_PROTOCOL.md`](docs/CAN_PROTOCOL.md) | CAN 메시지 명세 (VCU/Cluster 공유 단일 출처) |

> 전체 설계 원리(compute/update 분리, state 격리, 테스트 등)는 **VCU 레포의 [`ARCHITECTURE.md`](https://github.com/2026-HEVEN/vcu/blob/main/docs/ARCHITECTURE.md)** 와 동일합니다.

## 아직 미구현 (의도된 TODO)

- **VCU 표시 상태 프레임 구현** — VCU가 `0x1801C0D0`으로 확정 기어/HV/브레이크 상태를 보내면 Cluster가 그 값을 우선 표시합니다.
- **BMS BLE 실차 검증** — `LWS-1608` BLE 이름, `FFE0/FFE1/FFE2` 특성, SOC/current 부호를 실제 배터리팩에서 확인해야 합니다.

## 버전 기록 (Changelog)

> 각 버전은 git 태그로도 관리됩니다 → [GitHub Releases](https://github.com/2026-HEVEN/cluster/releases)
> **새 버전 올릴 때:** 아래에 항목 추가 → `git tag vX.Y` → `git push origin vX.Y`.

### v1.1.1 (2026-07-06) — 위젯 레이아웃 렌더 테스트
- `test/test_render_layout`: `app_wiring.cpp`의 위젯 배치를 그대로 재현해 24bit BMP로 덤프하는 시각화 테스트 추가. `pio test -e native -f test_render_layout` 로 실행 → `render_layout.bmp` 생성(Windows 사진 앱/그림판 등 추가 도구 없이 바로 열림), 위젯별 할당 공간을 박스로 표시
- `.gitignore`에 `*.bmp` 추가

### v1.1 (2026-06-29) — 디스플레이 프레임버퍼 재설계
- 패널 독립적 **1bpp 프레임버퍼**(320×160) + 위젯 모듈(speed · battery · warnings · gear) 도입 — 렌더링을 모듈로 넘겨 자유도↑, host 테스트 가능(ASCII 시각화)
- `hmi_input` → **`ClusterCommand`**(paddock · TC · regen · debug) 의미 커맨드 패턴, **패독 모드** 추가
- **제거**: `vess`, `indicators`, `display_render`(U8g2), `io_expander`(MCP23017)
- `display_blit` stub(패널 확정 후 구현), CAN 커맨드 레이아웃(`CAN_PROTOCOL.md` §5.7) 갱신

### v1.0 (2026-06-29) — 초기 Cluster 펌웨어 베이스
- 잠긴 코어(CAN RX · MCP23017 · U8g2 OLED) + 순수 모듈(display · indicators · vess · hmi_input)
- PlatformIO native 테스트, AGENTS/ADDING_A_MODULE/CAN_PROTOCOL 문서

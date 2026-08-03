# HEVEN Cluster 펌웨어

**2026 영광 대회 · 계기 클러스터(Instrument Cluster)** — ESP32가 CAN/BLE로 차량 상태(속도·전압·SOC·온도·에러)를 **받아서** LCD에 표현하고, 계기판 버튼 입력을 VCU로 보냅니다. VESS 신호 경로와 스위칭은 클러스터 ESP32 GPIO 입력에서 분리합니다. VCU와 달리 **안전 임계가 약한 표시 전용 보드**입니다.

> 🤖 **AI 에이전트/팀원은 [`AGENTS.md`](AGENTS.md)를 먼저 읽으세요** — 무엇을 고쳐도 되고 무엇을 건드리면 안 되는지 규칙이 있습니다.

---

## 한눈에

- **스택**: PlatformIO + Arduino-ESP32, TWAI(CAN), ILI9341 LCD. 화면은 **패널 독립적 1bpp 프레임버퍼**(위젯이 그림) + `display_blit`(ILI9341로 blit)
- **구조**: 잠긴 코어(CAN·프레임버퍼·blit) + 팀원이 채우는 순수 모듈(`src/modules/`). VCU와 **동일한 2층 설계**지만 안전 FSM·50ms 라이프 태스크가 없고 CAN은 **수신(RX) 위주**입니다.
- **상태**: ESP32 빌드 그린, 호스트 테스트 60개 통과

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
> ℹ️ 계기판 속도 표시는 VCU가 송신하는 휠스피드 프레임 `0x1802C0D0`을 우선 사용합니다. FL/FR/RL/RR RPM을 km/h로 변환하고, 최솟값/최댓값을 제외한 가운데 두 바퀴 평균을 대표 속도로 표시합니다.
> ℹ️ GPS Lap Start는 현재 GNSS fix 위치를 출발점으로 저장합니다. 이후 휠스피드 기반 차량속도 `0.5 km/h` 초과가 `150ms` 이상 지속되면 랩타이머를 시작하고, 다시 출발점 반경 `0.75m` 안으로 들어오면 랩을 갱신합니다.


## 현재 하드웨어 핀맵

| 구분 | 기능 | ESP32 GPIO | 연결/동작 |
|------|------|------------|-----------|
| CAN | TXD | GPIO18 | CAN 트랜시버 TXD |
| CAN | RXD | GPIO17 | CAN 트랜시버 RXD |
| LCD ILI9341 | CS | GPIO4 | LCD controller CS, SD_CS 아님 |
| LCD ILI9341 | DC | GPIO5 | D/C, A0. GPIO5는 strapping pin이라 외부 pull-up/down 금지 |
| LCD ILI9341 | RST | GPIO16 | LCD reset |
| LCD ILI9341 | SCK | GPIO21 | SPI clock |
| LCD ILI9341 | MOSI | GPIO19 | ESP32 -> LCD/Touch |
| LCD / Touch SPI | MISO / T_DO | GPIO22 | Touch controller -> ESP32, LCD SDO readback optional |
| LCD Touch XPT2046 | T_CS | GPIO23 | Touch chip select, 터치 시 Car Check 화면 토글 |
| GPS ZED-F9P | RX | GPIO35 | GNSS UART TX -> ESP32, 38400 baud NMEA RMC. GPS 탈착 가능 구조면 10k pull-up to 3.3V 권장 |
| HMI | Paddock | GPIO33 | 토글 스위치, INPUT_PULLUP, ON=LOW |
| HMI | TC | GPIO25 | 토글 스위치, INPUT_PULLUP, ON=LOW |
| HMI | Regen rotary bit0 | GPIO27 | 4단 로터리 셀렉터 코드 bit0, INPUT_PULLUP, active LOW |
| HMI | Regen rotary bit1 | GPIO14 | 4단 로터리 셀렉터 코드 bit1, INPUT_PULLUP, active LOW |
| HMI | Debug | GPIO26 | 토글 스위치, INPUT_PULLUP, ON=LOW |
| HMI | GPS Lap Start | GPIO32 | 순간 푸시 버튼, 정상 상태에서 GPS lap start 저장 |
| HMI | Warning Detail | GPIO13 | 순간 푸시 버튼, warning 상세 화면 토글 |
| LV monitor | LV voltage ADC | GPIO34 | 12V 라인을 100k/27k 저항분압 후 입력, Car Check에 LV 전압/LOW/HIGH 표시 |

회생제동 4단 로터리 셀렉터는 `bit1 bit0` 조합으로 `0~3` 단계를 만든다. 두 선 모두 내부 pull-up을 사용하므로 각 코드선은 선택 시 GND로 떨어지는 active-low 배선이다.

| Regen level | bit1(GPIO14) | bit0(GPIO27) |
|-------------|--------------|--------------|
| 0 | HIGH | HIGH |
| 1 | HIGH | LOW |
| 2 | LOW | HIGH |
| 3 | LOW | LOW |

Car Check의 LV 표시는 `11.0V` 미만이면 `LOW`, `15.0V` 이상이면 `HIGH`, 그 사이는 `OK`로 표시한다. 빨간 warning 화면에는 연결하지 않는 정비용 표시이다.

VESS, 기어, 시동 스위치는 클러스터 패널/PCB에 물리 배치할 수 있지만 Cluster ESP32 GPIO에는 연결하지 않는다. 해당 신호는 VESS 회로 또는 VCU/차량 배선 쪽에서 처리한다.
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

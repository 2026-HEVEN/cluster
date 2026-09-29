# 회생 로터리 단계 송신 (2026-09-29)

Cluster는 기존 GPIO 로터리의 2비트 입력을 항상 0/1/2/3으로 송신한다.
ON/OFF 또는 4단계 해석과 원페달/브레이크 작동 조건은 VCU의
`src/modules/realcar_calibration.h`에서만 설정한다.

`0x1801D0C0` Extended, DLC 8:
- Byte1 bit1: 단계가 1..3이면 ON, 0이면 OFF (기존 호환).
- Byte3: 단계 0/1/2/3 → 0xA0/0xA1/0xA2/0xA3.
- 잘못된 로터리 값(3 초과)은 OFF로 송신.
- 나머지 TV/Debug/Paddock 비트·송신 주기는 유지.

구형 VCU는 Byte3를 무시하므로 1/2/3 모두 같은 세기로 작동한다.
단계 시험에는 VCU도 함께 업데이트해야 한다. 계기 ON/OFF 표시를
실제 VCU 적용 단계의 확인으로 사용하지 말 것.

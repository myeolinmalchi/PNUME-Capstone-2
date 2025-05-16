# 2025-1 종합설계과제 (Capstone-2)

## 프로젝트 소개

TBD

## 시스템 모식도

TBD

## 폴더 구조

- `desktop/`: 사용자 데스크탑에서 구동되는 GUI 프로그램
- `host/`: 호스트 PC에서 RKKN 포맷 변환 등에 사용되는 스크립트
- `edge/`: 임베디드 장치에서 구동되는 소프트웨어
    - `sbc/`: RKNN, YOLO11 기반 Real-Time Object Detection 수행
    - `mcu/`: 모터 및 액추에이터 실시간 제어 (closed-loop)

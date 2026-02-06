# BSAPS  
Battery Stress Assessment & Protection System

BSAPS는 배터리의 잔존 수명(SOH)을 정밀하게 추정하는 시스템이 아니라,  
배터리가 거칠게 사용되고 있는 환경(사용 스트레스)을 실시간으로 판단하고  
임계 상황에서 보호 동작까지 수행하는 임베디드 시스템이다.

본 프로젝트는 이론적 모델링보다 실제 하드웨어 환경에서의 동작 안정성과  
상태 기반 제어 흐름을 중심으로 설계되었다.

---

## 1. Problem Statement

배터리 관리 시스템은 일반적으로 전압, 전류, 온도와 같은 물리량을 측정하지만,  
실제 사용 환경에서 배터리가 얼마나 거칠게 사용되고 있는지를  
직관적으로 판단하고 즉각적으로 대응하는 구조는 상대적으로 부족하다.

특히 다음과 같은 상황에서는 배터리 수명이 급격히 저하될 수 있다.

- 장시간 고부하 전류가 인가되는 환경
- 발열이 지속되는 상태에서의 사용
- 보호 로직이 개입하기 전에 반복되는 스트레스 상황

본 프로젝트는 이러한 문제의식에서 출발하여  
배터리의 열화 상태를 추정하는 것이 아니라,  
배터리 수명을 빠르게 소모시키는 사용 스트레스 환경 자체를  
실시간으로 감지하고 보호 동작으로 연결하는 것을 목표로 한다.

---

## 2. Project Goal and Scope

### 2.1 What this project does

- 온도, 전압, 전류 센서를 통해 사용 환경 데이터 수집
- 상태머신(State Machine)을 기반으로 배터리 사용 상태 분류
- 스트레스 조건 감지 시 보호(PROTECT) 상태로 전환
- MOSFET 제어를 통한 부하 자동 차단
- 로그 출력을 통한 상태 전이 및 판단 근거 확인

### 2.2 What this project intentionally does NOT do

- 배터리 SOH(잔존 수명) 정밀 추정
- 복잡한 배터리 수학 모델링
- 머신러닝 기반 예측 알고리즘

이 프로젝트는 정확한 진단보다  
명확한 판단과 즉각적인 보호에 초점을 둔다.

---

## 3. System Overview

시스템은 다음과 같은 흐름으로 동작한다.

1. 입력  
   - 온도 (DS18B20)  
   - 전압 (아날로그 전압 센서)  
   - 전류 (INA219)

2. 처리  
   - 상태머신 기반 판단 로직  
   - 시간 조건과 임계값을 고려한 스트레스 판단

3. 출력  
   - MOSFET 제어를 통한 부하 인가 및 차단  
   - 상태 전이 및 센서 값 로그 출력

---

## 4. State Machine Design

본 시스템은 명확한 동작 흐름과 안정적인 보호 동작을 위해  
상태머신(State Machine) 구조로 설계되었다.

### 4.1 State Definitions

- IDLE  
  정상 대기 상태. 부하가 인가되지 않은 상태

- LOAD_ON  
  사용자 입력에 의해 부하가 인가된 상태

- STRESS  
  전류 및 시간 조건을 기준으로 스트레스가 감지된 상태

- PROTECT  
  임계 스트레스 조건 도달 시 보호 동작 수행  
  MOSFET OFF를 통해 부하 차단

- COOLING  
  보호 이후 냉각 및 안정화 구간

- IDLE (Return)  
  조건 만족 시 초기 상태로 복귀

### 4.2 State Machine Diagram

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> LOAD_ON : Button pressed
    LOAD_ON --> STRESS : High current detected
    STRESS --> PROTECT : Stress threshold exceeded
    PROTECT --> COOLING : Load cut off
    COOLING --> IDLE : System stabilized

이 구조를 통해 상태 전이 조건을 명확히 정의하고,
디버깅 및 향후 확장이 용이하도록 설계하였다.

5. Hardware Configuration
5.1 Main Components
ESP32 Dev Module (ESP32-WROOM-32 계열)

DS18B20 온도 센서

INA219 전류 센서 모듈

아두이노 전압 측정 센서 모듈

저항

4.7kΩ (신호용)

330Ω (부하용)

100Ω 3W (고부하 실험용)

스위치

MOSFET 기반 부하 제어 회로

12V 2A 어댑터 (테스트 전원)

5.2 Test Environment
현재 단계에서는 실제 배터리 대신 12V 어댑터를 사용하여 테스트

버튼 입력으로 부하를 인가하여 전류 스트레스를 의도적으로 발생

스트레스 조건 충족 시 자동 보호 동작 수행

6. Current Implementation Status
ESP32 기반 v1 시스템에서 다음 항목이 구현 및 검증되었다.

상태머신 기반 전체 동작 흐름

온도, 전압, 전류 센서의 안정적 측정

ADC floating 문제 해결

스트레스 조건 감지 로직

보호(PROTECT) 상태 진입 시 MOSFET 자동 차단

로그 출력 기반 상태 추적

정상 동작이 확인된 시점을 기준으로 Baseline을 고정하여
이후 변경 사항은 해당 기준선 대비 변화 중심으로 관리하고 있다.

7. Repository Structure
BSAPS/
├─ firmware/   ESP32 펌웨어 소스 코드
├─ hardware/   회로 구성 및 배선 정보
├─ logs/       테스트 및 동작 로그
├─ docs/       개발 기록 및 문서
└─ README.md
docs/dev_history.md
프로젝트 전반의 개발 과정과 주요 디버깅 이력을 정리한 문서

8. Design Notes
기능 확장보다 시스템 안정성과 판단 흐름의 명확성을 우선한다.

정상 동작 기준선을 명확히 유지하기 위해 Baseline 개념을 사용한다.

하드웨어 특성과 시간 흐름을 고려한 상태머신 설계를 핵심 설계 자산으로 삼는다.

9. Future Work
STM32 기반으로 시스템 포팅

보호 시나리오 및 제어 정밀도 확장

실제 배터리 환경을 고려한 테스트로 확장

10. Summary
BSAPS는 단순한 센서 연동 프로젝트가 아니라,
환경 → 상태 → 판단 → 보호로 이어지는
임베디드 시스템 설계 사고 과정을 정리한 결과물이다.

본 프로젝트를 통해 상태머신 기반 설계,
하드웨어 안정성 확보, 그리고 기준선 중심 개발의 중요성을 체득하였다.
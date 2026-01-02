## 🦾 On-Device AI기반 임베디드 제어 시스템 구축 

### AIoT
**AI (Artificial Intelligence) + IoT (of Things, 사물인터넷)**

### On-Device AI
휴대폰, 테슬라 자율주행차의 장비들에서는 장비 내부에서 고성능의 TPU가 탑재.

고성능의 장비에서는 서버에서는 데이터를 보내고 받아서 AI 연산을 하기보다는, 디바이스상에서 AI 연산을 수행하고 예측
- 가격뿐만 아니라 수행 시간 단축
- ex. 자율주행 차에서 실제 사람 or 모형인지 구별 등 목적성에 맞게 수행

---

## 📝 트랙 명세서
### 기본제공 개발보드
- **NVIDIA Jetson Orin Nano Developer Kit** (생성형 AI를 임베디드 시스템에서 수행하고 시행해볼 수 있는 장점, 85만원 정도)
- **Raspberry Pi 5** (센서 데이터 수집)
- Orin Car (RC Car 크기 정도의 Kit, 자동차에 센서나 카메라 장착)

### On-Device AI 및 AIoT 기술을 활용한 인공지능 모델 구현 및 활용
![프로젝트 명세서](https://i.imgur.com/LOSd1ur.png)

Sub 1 (1w) : **라즈베리파이5 활용한 IoT 시스템 구축** (필수)

Sub 2 (2w) : **NVIDIA Jetson Orin Nano 개발보드를 활용한 인공지능 신경망 구축** (자유주제, 팀마다의 고유 기획 프로젝트 권장)

Sub 3 (3w) : **통합 제어 시스템 - AIoT를 활용한 무인 주행 시스템 개발** (자유주제)
- 임베디드 보드 기반의 IoT 시스템에 대한 범용적 이해
- On-Device에서 활용한 가능한 다양한 형태의 AI 모델 구현
- Flask를 통해 웹에서의 통합 제어 시스템 구축

<br/>

![프로젝트 명세 상세](https://i.imgur.com/fInfvnN.png)

- 무인 주행 시스템 → 시스템 구축
- 웹 서버 구축에서는 Flask뿐만 아니라 Spring 등등 다양하게 개발 가능
- On-Device AI와 Subside AI를 상황에 따라 하이브리드 형식으로 사용하는 것도 가능
- 웹 서버뿐만 아니라 안드로이드 등으로 시스템 개발도 가능

### 프로젝트 내 사용 기술 스택
- 공통 : **Python, OpenCV**
- 라즈베리파이 : **GPIO, Flask**
- 젯슨오린나노 : **Resnet, CNN, YOLO**
- HardWare : **Raspberry Pi 5, Jetson Orin Nano, Orin Car**


### Sub-PJT 1
![프로젝트1 소개](https://i.imgur.com/GbF21Ek.png)

![프로젝트1 아키텍처](https://i.imgur.com/73Ku6BS.png)

<br/>


### Sub-PJT 2
![프로젝트2 소개](https://i.imgur.com/GTCGr3e.png)

![프로젝트2 아키텍처](https://i.imgur.com/w5PdbFi.png)

<br/>

### Sub-PJT 3
![프로젝트3 소개](https://i.imgur.com/utA3pnJ.png)

![프로젝트3 아키텍처](https://i.imgur.com/9AAIfdK.png)

### On-Device AI 활용 레퍼런스
1. **갤럭시 AI** (삼성전자)
통화중 **실시간 통역**, 문자 및 채팅 대화 번역, 노트앱 번역, 웹브라우저 번역
    - 딥러닝
    - 전이학습
    - 지식증류 및 양자화
    - STT, TTS

2. **AIoT 전동 휠체어** (KT)
전후방 카메라 탐지, 주변 영상 수집 기반 지도 생성, 실내 측위 등의 **전동 휠체어 자동주행 서비스**
    - 슬램 (동시적 위치추적 및 지도작성)
    - LiDAR
    - 객체 인식

3. **폴라리스 오피스** (업스테이지)
실시간 문서 번역, 정보 요약, 텍스트 생성 등의 **문서 작성 솔루션**
    - LLM

4. **애플 인텔리전스** (애플)
개인 스케줄 관리, 이메일 분류 및 작성, 메시지 앱 내 이미지 생성 등의 **아이폰, 아이패드, 맥 모두 사용할 수 있는 On-Device AI 적용**
    - LLM
    - STT 및 TTS

5. **비스포크 AI 라인업** (삼성전자)
빅스비 기반 가전 제어, **AI 솔루션 가전 적용**
    - LLM
    - STT 및 TTS

6. **인텔 AI Everywhere** (인텔)
**인텔 코어 울트라 프로세서 탑재한 노트북**으로 생성형AI 활용해 텍스트 입력 이미지 및 영상 생성, 음악 작곡한 노트북 출시
    - LLM

7. **패브릭스(FabriX) 브리타 코파일럿(Brity Copilot)** (삼성SDS)

    패브릭스 : 기업 내부의 시스템과 챗봇을 연결해주는 솔루션

    블리티 코파일럿 : 회의록을 작성하고 요약해서 기업 이메일로 전달해주거나, 보고서를 자동으로 작성 

    → **AI 트랜스포메이션(AX) 지원**

    - LLM
    - SOC (System on Chip)

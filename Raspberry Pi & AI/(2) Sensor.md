## 아날로그 신호

`pinMode( )` : 지정된 핀을 input 또는 output으로 동작하도록 구성.

근데 gpiozero 라이브러리는 쉽게 사용할 수 있도록 wrapping 되어있다

### PWM
Pulse Width Modulation (펄스 폭 변조),
**디지털 신호를 빠르게 스위칭**하여 마치 아날로그처럼 보이도록 출력

### PWMLED 모듈로 PWM 신호 출력
`$vi pwm.py`

`$python3 pwm.py`

- `PWMLED( )` : 객체 생성
- `.value` : 0~1 사이의 값으로 PWM 값 조절

```
from gpiozero import PWMLED
from time import sleep

led = PWMLED(14)

while True:
    led.value = 0
    sleep(1)
    led.value = 0.5
    sleep(1)
    led.value = 1
    sleep(1)
```

**라즈베리파이의 GPIO Pin은 3.3V / 0V만 출력이 가능하다 !** (이 신호를 빠르게 스위칭해서 마치 아날로그 신호처럼 보이게 함)

**1과 0의 비율을 어떻게 주는지에 따라 결과가 달라진다 !** (**Duty Rate** - 80%면 1과 0의 비율 4:1)

- Duty Rate 듀티비
- 주파수 *f* = 1/T (Hz)

- milli (m) : 10^(-3)
- micro (μ) : 10^(-6)
- nano (n) : 10^(-9)

라즈베리파이5 model B의 ARM Cortex-A76 2.4 GHz 같은 경우는
- 1초에 몇번 진동하는가 ? 24억 회
- 한번 진동할 때 몇 초 걸리는가 ? 0.417 ns 

---

## Sensor
**센서** : 정보를 수집하여 수치 값으로 만들어내는 장치

### MEMS
Micro Electro Mechanical Systems, 반도체 제조 공정으로 만드는 초소형 기계로, 초소형 센서 제작할 때 사용

### Driver
모듈을 제어할 수 있는 인터페이스 역할로, MCU는 Driver만 제어하고 Driver는 LED에게 명령 내림
- FND Driver
- 모터 Driver
- LCD Driver
- LED Driver

---

## Sense Hat
센서들이 모여있는 Hat (자이로 센서, 가속도 센서, 기압 센서, 지자기 센서, 온/습도 센서 등 ..)

### 온습도 센서
**실크**에 U3이라고 적혀있음 → `datasheets.raspberrypi.com`에서 sensehat 찾아서 U3 검색 → U3의 부품명인 HTS221의 pdf를 구글에서 검색 → 부품 스펙 가늠

### IMU
Inertial Measurement Unit 관성 측정 장치로 드론/자동차/선박 등 다양한 임베디드 장치에서 사용.

센스햇은 9축 센서이고 실크는 U4 → `datasheets.raspberrypi.com`에서 sensehat 찾아서 U4 검색 → U4의 부품 모델명 pdf 찾기

### 기압센서
실크 : U5 .. 동일한 방식으로

### LED Matrix
Driver를 이용하여 제어 .. U7

### 조이스틱 - 버튼
- 좌/우/위/아래/클릭 5개 버튼
- 실크 : J2
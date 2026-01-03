## Camera

### 라즈베리파이 Camera Module V3
- 해상도 : 12MP (MegePixel)
- 오토 포커싱
- 시야각 : 120도
- IR 필터 제거 : 적외선 빛 감지 기능

라즈베리파이5에 연결할 수 있는 카메라 케이블은 표기가 다름

### 패키지 설치
`$sudo apt-get update`

`$sudo apt-get install fswebcam-y`

`$sudo apt install -y python3-picamera2`

`$sudo reboot`

+) 항상 시간 체크도 해줘야함. 라즈베리파이는 RTC가 없어서 
`$sudo date -s "2026-01-03 21:43:10"`

---

## OpenCV 기초

---

## OpenCV 응용
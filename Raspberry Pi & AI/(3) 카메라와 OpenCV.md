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

대표적인 이미지/영상 처리 Library로, BSD 라이선스로 기업도 무료

```
import cv2
import numpy as np
```

### `.zeros(shape, dtype=data type)`
- 정해진 shape에 맞게 0을 채워넣는 API → 검은색 빈 도화지를 그리기 위해 사용
- shape : 크기와 채널
- dtype : 데이터 타입 의미

### `rectangle(img, pt1, pt2, color, thickness)`
- 사각형 그리는 API
- img : 도형 그릴 이미지
- pt1 : 왼쪽 위 꼭짓점 좌표
- pt2 : 오른쪽 아래 꼭짓점 좌표
- color : 색상 (B,G,R 튜플)
- thickness : 선 두께, 음수는 내부 채움 없음

### `circle(img, center, radius, color, thickness)`
- 원 그리는 API
- img : 도형 그릴 이미지
- center : 중심 좌표 (x, y)
- radius : 반지름
- color : 색상
- thickness : 두께, -1이면 채워진 원

### `putText(img, text, org, fontFace, fontScale, color, thickness)`
- 텍스트 작성하는 API
- img : 텍스트 넣을 이미지
- text : 문자열
- org : 텍스트 시작 좌표 (왼쪽 아래 기준)
- fontFace : 폰트 스타일
- fontScale : 폰트 크기 배율
- color : 색상
- thickness : 글자 두께

### `.imshow(text, img)`
- 윈도우 창에 이미지 출력
- text : 창 이름
- img : 이미지

### `waitKey(delay)`
- 키 누를 때까지 대기
- delay : milliseconds, 0이면 무한 대기

### `.destoryAllWindows()`
- 창 닫기

### `main()` 함수
- 다른 파일에 import할 경우, 호출되지 않음

### `imread(filename)`
- 이미지 파일 읽어오는 함수
- return
    - 파일 잘 읽어오면 NumPy 배열 형태로
    - 파일 읽기 실패하면 None 반환

### `blur(src, ksize)`

<img src="https://i.imgur.com/iZnaz9U.png" alt="blur" width="500px">

- 평균 블러(average blur) 처리 함수
- 이미지에서 노이즈 줄이거나 부드럽게 표현하고자 할 때
- src : 원본 이미지
- ksize : 커널 크기 (width x height)

### `GaussianBlur(src, ksize, sigmaX)`

<img src="https://i.imgur.com/DSpRf1Q.png" alt="gaussianblur" width="500px">

- 가우시안 커널 사용하여 이미지를 부드럽게 블러처리
- 일반 블러보다 더 자연스럽고 노이즈 제거에 효과적
- src : 원본 이미지
- ksize : 커널 크기
- sigmaX : X 방향 가우시안 커널 표준 편차, 0이면 자동 계산

### `cvtColor(src, code)`

<img src="https://i.imgur.com/1cqRrGt.png" alt="gray" width="500px">

- 이미지 색상 공간 변환하는 API
- src : 입력 이미지 (NumPy 배열)
- code : 색상 공간 변환 코드
    - COLOR_BGR2GRAY : grayscale으로 변환
- 많은 OpenCV 알고리즘은 색상이 아닌 **밝기(명암도) 기준**으로 동작
    - Canny Edge Detection, Thresholding, Contour Detection, Corner Detection 등등
- 컬러 이미지보다 **월등히 빠른 계산량**
    - 컬러 이미지는 RGB 3채널이지만, 흑백 이미지는 1채널
    - 처리해야 할 데이터양이 1/3으로 줄어듦

### `threshold(src, thresh, maxval, type)`

<img src="https://i.imgur.com/2ItCg2A.png" alt="threshold" width="500px">

- Grayscale(회색조) 이미지를 Binary(이진, 흑백) 이미지로 변환하는 API
- 임계값(threshold)를 기준으로 픽셀 값을 0 또는 255로 바꿈
- 윤곽선 검출 전 이진화, 마스크 생성, 객체 분할, 문서 스캔 이미지에서 배경 제거에 사용
- src : 입력 영상 (Grayscale)
- thresh : 임계값 (이 값보다 큰지 작은지 판단)
- maxval : 임계 조건을 만족할 때 적용할 최대값 (보통 255)
- type : thresholding 방식
- return
    - retval : 계산된 임계값
    - dst : 결과 이미지

### `morphologyEx(img, operation, kernel)`

<img src="https://i.imgur.com/4wJbvil.png" alt="morphology" width="500px">

- 고급 모폴로지 변환 함수
- 바이너리 마스크, 외곽선 추출, 노이즈 제거, 윤곽 강조 등에 유용
- src : 입력 이미지 (흑백 or 바이너리 이미지)
- operation : 모폴로지 연산 종류
- kernel : 구조화 요소, 주로 np.ones()로 생성

### `Canny(image, threshold1, threshold2)`

<img src="https://i.imgur.com/gsAJpGY.png" alt="canny" width="500px">

- 엣지(윤곽선) 검출 API
- 이미지에서 명확한 경계선 찾을 때
- 가장 빠르고 정확한 결과
- image : 반드시 흑백 이미지
- threshold1 : 하한 임계값, 픽셀 값이 하한 임계값보다 작으면 0
- threshold2 : 상한 임계값, 보통 threshold1의 2~3배, 픽셀 값이 상한 임계값보다 크면 무조건 엣지 처리
- threshold1 < pixel val < threshold2 : 주변에 threshold2가 있으면 엣지 처리 아니면 0
- return : 결과 이미지

---

## OpenCV 응용

### Motion Detecting
영상이나 센서 데이터를 분석하여 물체의 움직임 탐지
- 카메라 기반 모션 감지 : CCTV, 영상 감시, 로봇 비전 등
- 적외선 센서 기반 모션 감지 : 자동문, 출입 자동 조명 등

이미지 프레임의 변화량을 박스로 표시 !

1. 카메라 설정 및 시작
```
import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (224,224)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

picam2.start()
```

2. 전역 변수 및 함수 정의 + 회색조 변화나 및 블러 적용 + 기준 프레임 설정
```
avg = None

def motionDetect(frame):
    global avg

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21,21), 0)

    if avg is None:
        avg = gray.copy().astype("float")
        return frame
```

3. 프레임 업데이트 및 프레임 차이 계산
- `accumulateWeighted()` : 현재 프레임을 기준 프레임과 비교하기 위해 누적 평균을 적용하면서 업데이트 (노이즈 완화)
- `absdiff()` : 현재 프레임과 기준 프레임의 차이 계산 (차이가 클 수록 움직임이 있다)

```
cv2.accumulateWeighted(gray, avg, 0.5)

frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
```

4. 이진 이미지로 변환 + 윤곽 검출
- `threshold()`: 임계값 적용해 차이가 큰 부분을 흰색(255), 나머지는 검정(0)으로 변환
- `dilate`: 팽창처리, low level 모폴로지, 윤곽선을 더 확실하게 만듦
- `findContours()` : 윤곽선 찾는 함수 

```
thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1]
thresh = cv2.dilate(thresh, None, iterations=2)

cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
```

5. 윤곽선 기준으로 모션 판단
```
motion_detected = Flase

for c in cnts:
    if cv2.contourArea(c) < 50:
        continue
    (x, y, w, h) = cv2.boundingRect(c)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (128, 255, 0), 1)
    motion_detected = True

if motion_detected:
    print("Motion Detecting")
else:
    print("No Movement")

return frame
```

6. main() 함수
```
def main():
    while True:
        frame = picam2.capture_array()
        frame = motionDetect(frame)
        cv2.imshow("Motion Detection", frame)

        if cv2.waitKey(1) & oxFF == ord('q'):
            break
    
    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### Color Recognition
```
import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (224, 224)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

picam2.start()
```

1. 색상 인식 함수
```
def colorRecognition(frame):
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2
    
    box_size = 10
    top_left = (cx - box_size, cy - box_size)
    bottom_right = (cx + box_size, cy + box_size)
    cv2.rectangle(frame, top_left, bottom_right, (255, 255, 255), 1)
```

2. 색상 인식을 위한 준비 및 HSV 값 추출
- HSV는 색상(H), 채도(S), 명도(V)로 구성되어 밝기와 색을 독립적으로 처리 가능하고 색 추적 범우 지정이 쉬움
```
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (15, 15))

    hsvValue = hsv[cy, cx]

    text = f"HSV: {hsvValue}"
    cv2.putText(frame, text, (cx - 60, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

    return frame
```

3. main() 함수
```
def main():
    while True:
        frame = picam2.capture_array()
        frame = colorRecognition(frame)
        cv2.imshow("Color Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### Color Tracking
프레임에서 특정 지은 색의 범위에 해당하는 색상 찾으면 방향 표시

1. 인식할 색상의 범위 지정
- 색상 인식은 환경에 민감하므로 검색이 아닌 실험을 통해 구한다
```
import cv2
from picamera2 import Picamera2
import numpy as np

picam2 = Picamera2()
picam2.preview_configuration.main.size = (224, 224)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

picam2.start()

#Yellow #FFFF00
colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])
```

2. 색상 추적 함수
```
def colorTracking(frame):
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2

    box_size = 10
    top_left = (cx - box_size, cy - box_size)
    bottom_right = (cx + box_size, cy + box_size)
    cv2.rectangle(frame, top_left, bottom_right, (255, 255, 255), 1)
```

3. 색상 추적을 위한 마스킹 및 윤곽선 찾기
```
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
```

4. 윤곽선이 1개라도 존재하면 색상 인식
- 가장 큰 윤곽선을 외접한 최소 원의 중심 좌표 및 반지름 구하기
```
    if len(cnts) > 0:
        print("Target Detected")
        
        c = max(cnts, key=cv2.contourArea)
        
        ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)
        
        cv2.rectangle(frame,(int(box_x-radius),int(box_y+radius)),(int(box_x+radius),int(box_y-radius)),(0,255,0),1)
```

5. 중심 좌표로부터 거리 구하기
- 인식된 색상의 중심 좌표(정수)
- tolerance : 허용 오차
- 방향 표기용 변수

- up/down/left/right 방향 구하기
```
        X = int(box_x)
        Y = int(box_y)       
        
        tolerance = 10
        dir_y = ""
        dir_x = ""

        if Y < cy - tolerance:
            dir_y = "up"
        elif Y > cy + tolerance:
            dir_y = "down"

        if X < cx - tolerance:
            dir_x = "left"
        elif X > cx + tolerance:
            dir_x = "right"
```

6. 화면에 방향 표기
```
        if dir_y == "" and dir_x == "":
            direction = "center"
        elif dir_y == "":
            direction = dir_x
        elif dir_x == "":
            direction = dir_y
        else:
            direction = f"{dir_y}-{dir_x}"

        cv2.putText(frame, direction, (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    
    else:
        print("None")
    
    return frame
```

7. main() 함수
```
def main():
    while True:
        frame = picam2.capture_array()
        frame = colorTracking(frame)
        cv2.imshow("Color Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```
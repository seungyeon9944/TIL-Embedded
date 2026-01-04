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
![blur|100](https://i.imgur.com/iZnaz9U.png)

- 평균 블러(average blur) 처리 함수
- 이미지에서 노이즈 줄이거나 부드럽게 표현하고자 할 때
- src : 원본 이미지
- ksize : 커널 크기 (width x height)

### `GaussianBlur(src, ksize, sigmaX)`
![gaussain blur|100](https://i.imgur.com/DSpRf1Q.png)

- 가우시안 커널 사용하여 이미지를 부드럽게 블러처리
- 일반 블러보다 더 자연스럽고 노이즈 제거에 효과적
- src : 원본 이미지
- ksize : 커널 크기
- sigmaX : X 방향 가우시안 커널 표준 편차, 0이면 자동 계산

### `cvtColor(src, code)`
![gray|100](https://i.imgur.com/1cqRrGt.png)

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
![threshold|100](https://i.imgur.com/2ItCg2A.png)

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
![morphology|100](https://i.imgur.com/4wJbvil.png)
- 고급 모폴로지 변환 함수
- 바이너리 마스크, 외곽선 추출, 노이즈 제거, 윤곽 강조 등에 유용
- src : 입력 이미지 (흑백 or 바이너리 이미지)
- operation : 모폴로지 연산 종류
- kernel : 구조화 요소, 주로 np.ones()로 생성

### `Canny(image, threshold1, threshold2)`
![canny|100](https://i.imgur.com/gsAJpGY.png)

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
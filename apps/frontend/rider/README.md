# LiNKY Rider App

라이더용 배송 관리 앱입니다.

## 기능

- 영수증 스캔 (자동/수동)
- 위치 정보 수집 동의
- 사진 촬영
- 배송 진행 상태 확인

## 개발 환경

- Vue 3
- Vite
- JavaScript

## 설치 및 실행

```bash
# 의존성 설치
npm install

# 개발 서버 실행
npm run dev

# 빌드
npm run build
```

## 모바일 테스트

ngrok을 사용하여 모바일에서 테스트할 수 있습니다:

```bash
# ngrok 설치
npm install -g ngrok

# ngrok 설정
ngrok config add-authtoken YOUR_TOKEN

# 개발 서버 실행
npm run dev

# 새 터미널에서 ngrok 실행
ngrok http 5173
```

## API 설정

`vite.config.js`에서 백엔드 서버 주소를 설정하세요:

```javascript
proxy: {
  '/api': {
    target: 'http://YOUR_BACKEND_IP:8080',
    changeOrigin: true,
    rewrite: path => path,
  },
}
```

## 화면 구성

1. HowToUseScreen - 이용방법 안내
2. ScanOptionScreen - 스캔 방식 선택
3. ReceiptScanScreen - 영수증 스캔
4. ManualInputScreen - 수동 입력
5. ManualConfirmScreen - 수동 입력 확인
6. LocationRequestScreen - 위치 정보 동의
7. PhotoCaptureScreen - 사진 촬영
8. CompleteScreen - 완료 화면
9. LoadingModal - 로딩 모달
10. ScanConfirmModal - 스캔 결과 확인

## 라이센스

SSAFY S13P11C102 프로젝트
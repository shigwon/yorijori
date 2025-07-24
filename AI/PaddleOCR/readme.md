# PaddleOCR Docker 실행 가이드

## 환경 정보
- **OS**: Windows (Docker 사용)  
  - 리눅스 버전 도커는 GPU 사용을 위해 추가 프로그램 설치 필요
- **CUDA 버전**: 12.6 ~ 12.9
- **GPU**: NVIDIA 3050Ti (노트북)

## 성능
- **서버 버전**: 추론 시간 약 15초
- **모바일 버전**: 추론 시간 약 3초

## 실행 정보
- **포트 번호**: 8000
- **API 엔드포인트**: `/ocr`
- **실행 명령어[도커 생성 시]**: docker run --gpus all -p 8000:8000 paddleocr
FROM node:18-alpine AS builder

WORKDIR /app

# package.json과 package-lock.json 먼저 복사 (캐싱 최적화)
COPY package*.json ./

# 의존성 설치
RUN npm ci --only=production

# 소스 코드 복사
COPY . .

# 빌드
RUN npm run build

# 실행 단계
FROM nginx:alpine

# nginx 설정 파일 복사
COPY --from=builder /app/dist /usr/share/nginx/html

# 커스텀 nginx 설정 (선택사항)
COPY nginx.conf /etc/nginx/nginx.conf

# 80 포트 노출
EXPOSE 80

# nginx 실행
CMD ["nginx", "-g", "daemon off;"]
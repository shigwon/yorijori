# Vue 앱 빌드만 수행
FROM node:lts-alpine AS build-stage
WORKDIR /pubweb
COPY package*.json ./
RUN npm install
COPY . .
RUN npm run build

# 정적 파일만 결과물로 남기는 스테이지
FROM alpine:latest AS export-stage
WORKDIR /dist
COPY --from=build-stage /pubweb/dist .
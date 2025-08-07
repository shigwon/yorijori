FROM node:lts-alpine AS build-stage
WORKDIR /pubweb
COPY package*.json ./
RUN npm install
COPY . .
RUN npm run build

FROM nginx:stable-alpine AS production-stage
COPY --from=build-stage /pubweb/dist /usr/share/nginx/html

# 올바른 Nginx 설정 파일 복사
COPY nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 7080
CMD ["nginx", "-g", "daemon off;"]
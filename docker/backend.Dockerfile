FROM openjdk:17-slim AS builder

WORKDIR /app

# Gradle wrapper와 build 파일들을 먼저 복사 (캐싱 최적화)
COPY gradle/ gradle/
COPY gradlew build.gradle settings.gradle ./

# 권한 설정
RUN chmod +x ./gradlew

# 의존성 다운로드 (별도 레이어로 캐싱)
RUN ./gradlew dependencies --no-daemon || true

# 소스 코드 복사
COPY src/ src/

# 애플리케이션 빌드
RUN ./gradlew build -x test --no-daemon

# 실행 단계
FROM openjdk:17-jre-slim

# 보안을 위한 non-root 사용자 생성
RUN groupadd -r appuser && useradd -r -g appuser appuser

WORKDIR /app

# 빌드된 JAR 파일 복사
COPY --from=builder /app/build/libs/*.jar app.jar

# 파일 소유권 변경
RUN chown appuser:appuser app.jar

# non-root 사용자로 전환
USER appuser

# 포트 노출
EXPOSE 8000

# JVM 옵션 설정
ENV JAVA_OPTS="-Xmx512m -Xms256m -XX:+UseG1GC -XX:+UseContainerSupport"

# 애플리케이션 실행 (8000 포트로 설정)
ENTRYPOINT ["sh", "-c", "java $JAVA_OPTS -jar app.jar --server.port=8000"]
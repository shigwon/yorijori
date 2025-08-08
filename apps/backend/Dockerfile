FROM openjdk:17
WORKDIR /usr/src/myapp
COPY ./build/libs/linky-0.0.1-SNAPSHOT.jar /app.jar
ENTRYPOINT ["java", "-Dspring.profiles.active=${spring_profiles_active}", "-jar", "/app.jar"]
EXPOSE 8080

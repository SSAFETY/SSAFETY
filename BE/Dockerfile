FROM gradle:8.2.1 as builder

WORKDIR /app

COPY . /app
RUN ./gradlew --version
RUN ./gradlew build --no-daemon
RUN cp ./build/libs/`ls build/libs | grep SNAPSHOT.jar` /app/app.jar

FROM openjdk:17-jdk-slim

COPY --from=builder /app/app.jar /app.jar

EXPOSE 8080

CMD ["java", "-jar", "/app.jar"]

package com.linky.api.stream.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.stream.entity.JoinRequest;
import io.openvidu.java.client.*;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("/api/v1/stream")
public class StreamController {

    private final OpenVidu openVidu;
    private final Map<Integer, Session> sessionMap = new ConcurrentHashMap<>();

    @PostMapping("/join")
    public ResponseEntity<ApiResponseEntity<String>> joinSession(@RequestBody JoinRequest request)
            throws OpenViduJavaClientException, OpenViduHttpException {

        int robotId = request.getRobotId();
        String role = request.getRole();

        Session session = sessionMap.computeIfAbsent(robotId, id -> {
            try {
                return openVidu.createSession(
                        new SessionProperties.Builder()
                                .customSessionId("robot-" + String.valueOf(robotId)) // 문자열로 변환
                                .build()
                );
            } catch (Exception e) {
                log.error(">>> [webRTC] 세션 생성 실패: {}", e.getMessage());
                return null;
            }
        });

        if (session == null) {
            return ApiResponseEntity.failResponseEntity("세션 생성 실패");
        }

        OpenViduRole openViduRole = "PUBLISHER".equalsIgnoreCase(role)
                ? OpenViduRole.PUBLISHER
                : OpenViduRole.SUBSCRIBER;

        ConnectionProperties properties = new ConnectionProperties.Builder()
                .type(ConnectionType.WEBRTC)
                .role(openViduRole)
                .build();

        String token = session.createConnection(properties).getToken();
        return ApiResponseEntity.successResponseEntity(token);
    }
}

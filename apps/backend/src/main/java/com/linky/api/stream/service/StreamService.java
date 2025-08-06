package com.linky.api.stream.service;

import com.linky.api.stream.entity.JoinRequest;
import io.openvidu.java.client.*;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;


@Service
@Slf4j
@RequiredArgsConstructor
public class StreamService {

    private final OpenVidu openVidu;
    private final Map<Integer, Session> sessionMap = new ConcurrentHashMap<>();

    public String joinSession(JoinRequest request) {
        int robotId = request.getRobotId();
        String role = request.getRole();

        // 세션 가져오기 또는 새로 생성
        Session session = sessionMap.computeIfAbsent(robotId, id -> {
            try {
                return openVidu.createSession(
                        new SessionProperties.Builder()
                                .customSessionId("robot-" + id)
                                .build()
                );
            } catch (Exception e) {
                log.error(">>> [webRTC] 세션 생성 실패: {}", e.getMessage());
                return null;
            }
        });

        if (session == null) {
            return null;
        }

        // 역할 설정
        OpenViduRole openViduRole = "PUBLISHER".equalsIgnoreCase(role)
                ? OpenViduRole.PUBLISHER
                : OpenViduRole.SUBSCRIBER;

        // 토큰 생성
        ConnectionProperties properties = new ConnectionProperties.Builder()
                .type(ConnectionType.WEBRTC)
                .role(openViduRole)
                .build();

        try {
            return session.createConnection(properties).getToken();
        } catch (OpenViduJavaClientException | OpenViduHttpException e) {
            log.error(">>> [webRTC] 토큰 생성 실패: {}", e.getMessage());
            return null;
        }
    }
}
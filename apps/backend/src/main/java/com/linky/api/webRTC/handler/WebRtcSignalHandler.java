package com.linky.api.webRTC.handler;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.webRTC.dto.Message;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.jetbrains.annotations.NotNull;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Component
@RequiredArgsConstructor
public class WebRtcSignalHandler extends TextWebSocketHandler {

    private final ObjectMapper objectMapper;
    private final Map<String, Set<WebSocketSession>> roomSessions = new ConcurrentHashMap<>();

    @Override
    public void afterConnectionEstablished(@NotNull WebSocketSession session) {
        log.info(">>> [webRTC] 연결됨 : 세션 아이디 - {}", session.getId());
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage textMessage) throws Exception {

        Message message = objectMapper.readValue(textMessage.getPayload(), Message.class);
        String robotId = String.valueOf(message.getRobotId());

        log.info(">>> [webRTC] 타입 = {}, 수신자 = {}, 로봇 번호 = {}", message.getType(), message.getSender(), robotId);

        roomSessions.computeIfAbsent(robotId, key -> new HashSet<>()).add(session);

        for (WebSocketSession peer : roomSessions.get(robotId)) {
            if(!peer.getId().equals(session.getId()) && peer.isOpen()) {
                peer.sendMessage(textMessage);
            }
        }

    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {

        roomSessions.forEach((roomId, sessions) -> {
            sessions.remove(session);
            if(sessions.isEmpty()) {
                roomSessions.remove(roomId);
            }
        });
        log.info(">>> [webRTC] 클라이언트 접속 해제 : 세션 - {}, 상태 - {}", session, status);
    }
}

package com.linky.api.streaming.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.streaming.dto.StreamingImage;
import lombok.RequiredArgsConstructor;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

@RequiredArgsConstructor
@RestController
@RequestMapping("api/v1/streaming")
public class StreamingController {

    private static final Map<Integer, List<SseEmitter>> robotSubscribers = new ConcurrentHashMap<>();

    @GetMapping(value="/subscribe/{robotId}", produces= MediaType.TEXT_EVENT_STREAM_VALUE)
    public SseEmitter subscribe(@PathVariable("robotId") int  robotId) {
        SseEmitter emitter = new SseEmitter(0L);
        robotSubscribers
                .computeIfAbsent(robotId, k -> new CopyOnWriteArrayList<>()).add(emitter);

        emitter.onCompletion(() -> removeEmitter(robotId, emitter));
        emitter.onTimeout(() -> removeEmitter(robotId, emitter));
        emitter.onError((e) -> removeEmitter(robotId, emitter));
        return emitter;
    }

    public void removeEmitter(int robotId, SseEmitter emitter) {
        List<SseEmitter> list = robotSubscribers.get(robotId);
        if(list != null) {
            list.remove(emitter);
        }
    }

    public static void broadcastImage(int robotId, String image) {
        List<SseEmitter> list = robotSubscribers.get(robotId);
        if (list == null) {
            return;
        }

        for (SseEmitter emitter : list) {
            try {
                emitter.send(SseEmitter.event().name("robotImage").data(image));
            } catch(IOException e) {
                list.remove(emitter);
            }
        }
    }
}

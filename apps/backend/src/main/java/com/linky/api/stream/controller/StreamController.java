package com.linky.api.stream.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.stream.entity.JoinRequest;
import com.linky.api.stream.service.StreamService;
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
    private final StreamService streamService;

    @PostMapping("/join")
    public ResponseEntity<ApiResponseEntity<String>> joinSession(@RequestBody JoinRequest request)
            throws OpenViduJavaClientException, OpenViduHttpException {

        String token = streamService.joinSession(request);
        if (token == null) {
            return ApiResponseEntity.failResponseEntity("세션 생성 실패");
        }
        return ApiResponseEntity.successResponseEntity(token);
    }
}

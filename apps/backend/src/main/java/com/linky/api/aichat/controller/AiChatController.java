package com.linky.api.aichat.controller;

import com.linky.api.aichat.dto.request.AiChatRequestDto;
import com.linky.api.aichat.service.AiChatService;
import com.linky.api.common.response.entity.ApiResponseEntity;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/v1/ai/chat")
@RequiredArgsConstructor
public class AiChatController {

    private final AiChatService aiChatService;

    @PostMapping
    public ResponseEntity<ApiResponseEntity<String>> getAiResponse(@RequestBody AiChatRequestDto aiChatRequestDto) {
        String gmsResponseDto = aiChatService.getGmsResponse(aiChatRequestDto.message());
        return ApiResponseEntity.successResponseEntity(gmsResponseDto);
    }
}

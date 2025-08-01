package com.linky.api.inquiry.controller;


import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;
import java.util.Set;

@RestController
@RequestMapping("/api/v1/inquiries")
@RequiredArgsConstructor
public class InquiryRestController {

    private final ChatService chatService;

    @GetMapping("/{orderCode}/messages")
    public ResponseEntity<ApiResponseEntity<List<InquiryMessage>>> getChatMessages(@PathVariable String orderCode) {
        return ApiResponseEntity.successResponseEntity(chatService.getMessagesByOrderCode(orderCode));
    }

    @GetMapping("/rooms")
    public ResponseEntity<ApiResponseEntity<Set<String>>> getActiveChatRooms() {
        return ApiResponseEntity.successResponseEntity(chatService.getActiveChatRooms());
    }
}

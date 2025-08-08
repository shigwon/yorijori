package com.linky.api.inquiry.controller;


import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.inquiry.dto.ChatMessagePageRequestDto;
import com.linky.api.inquiry.dto.ChatMessagePageResponseDto;
import com.linky.api.inquiry.dto.ChatRoomSummaryDto;
import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

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

    @GetMapping("/rooms/summary")
    public ResponseEntity<ApiResponseEntity<List<ChatRoomSummaryDto>>> getChatRoomsWithSummary() {
        return ApiResponseEntity.successResponseEntity(chatService.getActiveChatRoomsWithSummary());
    }

    @GetMapping("/{orderCode}/messages/paged")
    public ResponseEntity<ApiResponseEntity<ChatMessagePageResponseDto>> getChatMessagesPaged(
            @PathVariable String orderCode,
            @RequestParam(defaultValue = "1") int page,
            @RequestParam(defaultValue = "50") int size) {

        ChatMessagePageRequestDto request = new ChatMessagePageRequestDto(page, size);
        return ApiResponseEntity.successResponseEntity(
                chatService.getMessagesByOrderCode(orderCode, request)
        );
    }
}

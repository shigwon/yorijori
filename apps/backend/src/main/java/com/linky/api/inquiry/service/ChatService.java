package com.linky.api.inquiry.service;

import com.linky.api.inquiry.dto.ChatMessagePageRequestDto;
import com.linky.api.inquiry.dto.ChatMessagePageResponseDto;
import com.linky.api.inquiry.dto.ChatRoomSummaryDto;
import com.linky.api.inquiry.dto.InquiryMessage;

import java.util.List;
import java.util.Set;

public interface ChatService {

    void saveMessage(String orderCode, InquiryMessage message);
    List<InquiryMessage> getMessagesByOrderCode(String orderCode);
    ChatMessagePageResponseDto getMessagesByOrderCode(String orderCode, ChatMessagePageRequestDto request);
    Set<String> getActiveChatRooms();
    List<ChatRoomSummaryDto> getActiveChatRoomsWithSummary();
}

package com.linky.api.inquiry.controller;

import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.messaging.handler.annotation.DestinationVariable;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Controller;

import java.time.LocalDateTime;

@Controller
@RequiredArgsConstructor
@Slf4j
public class InquiryController {

    private final ChatService chatService;
    private final SimpMessagingTemplate simpMessagingTemplate;

    @MessageMapping("/inquiry/{orderCode}/send")
    public void handleInquiry(@DestinationVariable String orderCode, InquiryMessage message) {
        InquiryMessage lastMessage = new InquiryMessage(
                message.sender(),
                message.content(),
                LocalDateTime.now()
        );

        log.info("Inquiry send message: {}", lastMessage.toString());

        chatService.saveMessage(orderCode, lastMessage);

        simpMessagingTemplate.convertAndSend("/topic/inquiries/" + orderCode, lastMessage);
    }
}

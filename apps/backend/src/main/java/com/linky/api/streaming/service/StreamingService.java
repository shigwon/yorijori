package com.linky.api.streaming.service;

import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.streaming.dto.StreamingImage;
import lombok.RequiredArgsConstructor;
import org.springframework.messaging.handler.annotation.DestinationVariable;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class StreamingService {

    private final SimpMessagingTemplate simpMessagingTemplate;

    public void sendStreamingImage(StreamingImage streamingImage) {
        simpMessagingTemplate.convertAndSend("/topic/streamingImage/"
                + streamingImage.getRobotId(), streamingImage.getImage());
    }

}

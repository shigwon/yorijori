package com.linky.api.message.service;

import net.nurigo.sdk.message.response.SingleMessageSentResponse;
import org.springframework.stereotype.Service;


public interface MessageService {
    public SingleMessageSentResponse messageSend(String tel, String url);
}

package com.linky.api.message.service.impl;

import com.linky.api.message.config.MessageConfig;
import com.linky.api.message.service.MessageService;
import lombok.RequiredArgsConstructor;
import net.nurigo.sdk.message.model.Message;
import net.nurigo.sdk.message.request.SingleMessageSendingRequest;
import net.nurigo.sdk.message.response.SingleMessageSentResponse;
import net.nurigo.sdk.message.service.DefaultMessageService;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class MessageServiceImpl implements MessageService {
    private final DefaultMessageService defaultMessageService;

    @Value("${SERVER_TEL}")
    String serverTel;

    public SingleMessageSentResponse messageSend(String tel, String url) {
        Message message = new Message();

        String customerTel = tel.replace("-", "");
        message.setFrom(serverTel);
        message.setTo(customerTel);
        message.setText(
                "📦 안녕하세요, 한강 배달 로봇 *링키(Linky)*입니다!\n\n" +
                        "원하시는 배달 위치를 아래 URL을 통해 지정해주세요. 🚚💨\n\n" +
                        "📍 위치 지정 링크: " + url + "\n\n" +
                        "정확한 위치를 설정해주시면 빠르고 정확하게 배달해드릴게요. 감사합니다! 😊"
        );
        SingleMessageSentResponse response = this.defaultMessageService.sendOne(new SingleMessageSendingRequest(message));
        System.out.println(response);
        return response;
    }

}

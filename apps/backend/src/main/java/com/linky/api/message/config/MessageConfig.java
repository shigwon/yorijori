package com.linky.api.message.config;

import net.nurigo.sdk.NurigoApp;
import net.nurigo.sdk.message.service.DefaultMessageService;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class MessageConfig {

    @Value("${SMS_API_KEY}")
    private String apiKey;

    @Value("${SMS_SECRET_KEY}")
    private String secretKey;

    @Bean
    public DefaultMessageService defaultMessageService() {
        return NurigoApp.INSTANCE.initialize(apiKey, secretKey, "https://api.solapi.com");
    }
}

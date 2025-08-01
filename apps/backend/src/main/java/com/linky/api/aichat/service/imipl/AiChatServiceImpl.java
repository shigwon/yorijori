package com.linky.api.aichat.service.imipl;

import com.linky.api.aichat.dto.request.GmsRequestDto;
import com.linky.api.aichat.dto.response.GmsResponseDto;
import com.linky.api.aichat.service.AiChatService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestClient;

import java.util.List;
import java.util.Map;

@Service
@RequiredArgsConstructor
@Slf4j
public class AiChatServiceImpl implements AiChatService {

    @Value("${spring.ai.openai.api-key}")
    private String apiKey;

    @Value("${spring.ai.openai.chat.options.model}")
    private String model;

    @Value("${spring.ai.openai.chat.options.temperature}")
    private String temperature;

    @Value("${spring.ai.openai.chat.options.max-tokens}")
    private String maxTokens;

    private final RestClient restClient;

    @Override
    public String getGmsResponse(String message) {
        try {
            GmsRequestDto request = new GmsRequestDto(
                    model,
                    List.of(
                            Map.of("role", "system", "content", "Answer in Korean"),
                            Map.of("role", "user", "content", message)
                    ),
                    Integer.parseInt(maxTokens),
                    Double.parseDouble(temperature)
            );

            GmsResponseDto response = restClient.post()
                    .header("Authorization", "Bearer " + apiKey)
                    .body(request)
                    .retrieve()
                    .body(GmsResponseDto.class);

            @SuppressWarnings("unchecked")
            Map<String, Object> firstChoice = (Map<String, Object>) response.choices().get(0);

            @SuppressWarnings("unchecked")
            Map<String, Object> gmsResponse = (Map<String, Object>) firstChoice.get("message");

            return (String) gmsResponse.get("content");

        } catch (Exception e) {
            log.error("GMS API 호출 실패: ", e);
            return "AI 서비스 오류가 발생했습니다.";
        }
    }
}

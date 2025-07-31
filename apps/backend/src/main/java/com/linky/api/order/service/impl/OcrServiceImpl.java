package com.linky.api.order.service.impl;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.protobuf.ByteString;
import com.linky.api.order.service.OcrService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.util.LinkedMultiValueMap;
import org.springframework.util.MultiValueMap;
import org.springframework.web.client.RestTemplate;
import org.springframework.http.*;
import org.springframework.core.io.ByteArrayResource;

import java.nio.charset.StandardCharsets;
import java.util.Map;

@Slf4j
@Service
public class OcrServiceImpl implements OcrService {

    private final RestTemplate restTemplate = new RestTemplate();
    private final ObjectMapper objectMapper = new ObjectMapper(); // JSON 파싱용

    @Override
    public Map<String, String> sendImageToOcr(String image) {
        try {
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);

            Map<String, String> jsonBody = Map.of("image", image);
            HttpEntity<Map<String, String>> requestEntity = new HttpEntity<>(jsonBody, headers);

            String ocrUrl = "http://localhost:8000/ocr";
            ResponseEntity<String> response = restTemplate.postForEntity(ocrUrl, requestEntity, String.class);

            if (response.getStatusCode().is2xxSuccessful() && response.getBody() != null) {
                Map<String, Object> raw = objectMapper.readValue(response.getBody(), new TypeReference<>() {});
                Map<String, String> result = (Map<String, String>) raw.get("result");
                return result;
            } else {
                return Map.of();
            }
        } catch (Exception e) {
            log.error("OCR 호출 실패", e);
            return Map.of();
        }
    }
}

package com.linky.api.order.service.impl;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.order.service.OcrService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.util.LinkedMultiValueMap;
import org.springframework.util.MultiValueMap;
import org.springframework.web.client.RestTemplate;
import org.springframework.http.*;
import org.springframework.core.io.ByteArrayResource;

import java.util.Map;

@Slf4j
@Service
public class OcrServiceImpl implements OcrService {

    private final RestTemplate restTemplate = new RestTemplate();
    private final ObjectMapper objectMapper = new ObjectMapper(); // JSON 파싱용

    @Override
    public Map<String, String> sendImageToOcr(byte[] imageBytes, String fileName) {
        try {
            MultiValueMap<String, Object> body = new LinkedMultiValueMap<>();
            ByteArrayResource byteArrayResource = new ByteArrayResource(imageBytes) {
                @Override
                public String getFilename() {
                    return fileName;
                }
            };

            body.add("file", byteArrayResource);

            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.MULTIPART_FORM_DATA);

            HttpEntity<MultiValueMap<String, Object>> requestEntity = new HttpEntity<>(body, headers);

            // 2. 요청 전송
            String ocrUrl = "http://localhost:8000/ocr";
            ResponseEntity<String> response = restTemplate.postForEntity(ocrUrl, requestEntity, String.class);

            if (response.getStatusCode().is2xxSuccessful() && response.getBody() != null) {
                // 3. JSON 파싱
                Map<String, Object> raw = objectMapper.readValue(response.getBody(), new TypeReference<>() {});
                Map<String, String> result = (Map<String, String>) raw.get("result");

                log.info("OCR 결과 수신: {}", result);
                return result;
            } else {
                log.warn("OCR 서버 응답 오류: {}", response.getStatusCode());
                return Map.of();
            }

        } catch (Exception e) {
            log.error("OCR 서버 호출 실패", e);
            return Map.of();
        }
    }
}

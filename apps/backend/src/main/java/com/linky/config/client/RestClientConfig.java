package com.linky.config.client;

import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.MediaType;
import org.springframework.http.client.ClientHttpRequestInterceptor;
import org.springframework.web.client.RestClient;

@Configuration
@Slf4j
public class RestClientConfig {

    @Value("${spring.ai.openai.base-url}")
    private String baseUrl;

    @Bean
    public RestClient gmsRestClient() {
        return RestClient.builder()
                .baseUrl(baseUrl)
                .defaultHeader("Content-Type", MediaType.APPLICATION_JSON_VALUE)
                .defaultHeader("Accept", MediaType.APPLICATION_JSON_VALUE)
                .defaultHeader("Accept-Charset", "UTF-8")
                .defaultHeader("Accept-Encoding", "gzip, deflate")
                .defaultHeader("User-Agent", "Linky-AI-Service/1.0")
                .defaultHeader("Cache-Control", "no-cache")
                .requestInterceptor(gmsRequestInterceptor())
                .build();
    }

    private ClientHttpRequestInterceptor gmsRequestInterceptor() {
        return (request, body, execution) -> {
            if (!request.getHeaders().containsKey("Content-Type")) {
                request.getHeaders().add("Content-Type", MediaType.APPLICATION_JSON_VALUE);
            }
            if (!request.getHeaders().containsKey("Accept")) {
                request.getHeaders().add("Accept", MediaType.APPLICATION_JSON_VALUE);
            }

            log.info("=== GMS API Request ===");
            log.info("URL: {}", request.getURI());
            log.info("Method: {}", request.getMethod());
            log.info("Headers: {}", request.getHeaders());
            log.info("Body Length: {}", body.length);

            var response = execution.execute(request, body);

            log.info("=== GMS API Response ===");
            log.info("Status: {}", response.getStatusCode());
            log.info("Response Headers: {}", response.getHeaders());

            return response;
        };
    }
}

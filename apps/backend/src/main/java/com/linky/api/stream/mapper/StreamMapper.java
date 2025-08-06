package com.linky.api.stream.mapper;

import com.linky.api.stream.entity.JoinRequest;
import com.linky.api.stream.entity.JoinResponse;
import org.springframework.stereotype.Component;

@Component
public class StreamMapper {

    public JoinResponse toEntity(boolean result, String data) {
        return JoinResponse.builder()
                .result(result)
                .data(data)
                .build();
    }
}

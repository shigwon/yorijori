package com.linky.api.stream.entity;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class JoinResponse {
    boolean result;
    String data;
}

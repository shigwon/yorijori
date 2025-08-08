package com.linky.api.streaming.dto;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class StreamingImage {
    int robotId;
    String image;
}

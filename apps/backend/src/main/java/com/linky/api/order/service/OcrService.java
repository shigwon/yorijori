package com.linky.api.order.service;

import com.google.protobuf.ByteString;

import java.util.Map;

public interface OcrService {
    Map<String, String> sendImageToOcr(String image);
}

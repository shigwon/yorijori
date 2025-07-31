package com.linky.api.order.service;

import com.google.protobuf.ByteString;

import java.util.Map;

public interface OcrService {
    public Map<String, String> sendImageToOcr(byte[] rawImageData);
}

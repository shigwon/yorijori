package com.linky.api.order.service;

import java.util.Map;

public interface OcrService {
    public Map<String, String> sendImageToOcr(byte[] imageBytes, String fileName);
}

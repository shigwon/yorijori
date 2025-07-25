package com.linky.api.test.service;

import com.linky.api.test.grpc.TestValidationResponse;
import org.springframework.stereotype.Service;

@Service
public class TestValidationService {

    public TestValidationResponse validationToken(String token) {

        boolean isValid = "valid-token".equals(token);
        long userId = isValid ? 123L : 0L;

        return TestValidationResponse.newBuilder()
                .setIsValid(isValid)
                .setUserId(userId)
                .setAccount(isValid ? "linkyUser" : "")
                .setErrorCode(isValid ? "" : "INVALID_TOKEN")
                .setMessage(isValid ? "Token is valid." : "Token validation failed.")
                .build();
    }
}

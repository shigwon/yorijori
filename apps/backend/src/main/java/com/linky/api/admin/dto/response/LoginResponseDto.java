package com.linky.api.admin.dto.response;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class LoginResponseDto {

    private boolean success;
    private String name;
    private String message;
    private String token; // JWT 토큰 추가 (필요시)
}
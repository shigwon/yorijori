package com.linky.api.admin.mapper;

import com.linky.admin.grpc.LoginRequest;
import com.linky.api.admin.entity.Admin;
import org.springframework.stereotype.Component;

@Component
public class AdminMapper {

    public Admin toEntity(LoginRequest request) {
        return Admin.builder()
                .email(request.getEmail())
                .password(request.getPassword())
                .build();
    }
}
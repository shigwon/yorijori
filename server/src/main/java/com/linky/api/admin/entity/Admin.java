package com.linky.api.admin.entity;

import com.linky.order.grpc.OrderState;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Builder
@Data
@AllArgsConstructor
@NoArgsConstructor
public class Admin {

    int id;
    String email;
    String password;
    String name;
}
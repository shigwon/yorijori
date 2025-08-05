package com.linky.api.robot.entity;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Builder
@Data
@AllArgsConstructor
@NoArgsConstructor
public class Robot {
    private int id;
    private String code;
    private String status;
    private LocalDateTime lastChargeTime;
}
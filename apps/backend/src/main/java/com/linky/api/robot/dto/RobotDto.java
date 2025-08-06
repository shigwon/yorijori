package com.linky.api.robot.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class RobotDto {
    private int id;
    private String code;
    private String status;
    private String statusDisplayName;
    private LocalDateTime lastChargeTime;
}
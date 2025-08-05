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
public class RobotWithLocationDto {
    // 기본 정보 (MySQL에서)
    private int id;
    private String code;
    private String status;
    private String statusDisplayName;
    private LocalDateTime lastChargeTime;

    // 위치 정보 (Redis에서)
    private Double latitude;
    private Double longitude;
    private boolean locationAvailable;
    private String locationStatus;
    private String locationDisplay;
}
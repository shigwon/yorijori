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
public class LocationHistoryDto {
    private Long id;
    private Integer robotId;
    private String robotCode; // Robot 정보 조인
    private Double latitude;
    private Double longitude;
    private LocalDateTime recordedAt;
    private String locationDisplay;
    private Double distanceFromPrevious; // 이전 위치와의 거리 (선택적)
}
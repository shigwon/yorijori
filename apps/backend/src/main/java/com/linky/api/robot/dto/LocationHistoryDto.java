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
    private String robotCode;
    private Double latitude;
    private Double longitude;
    private LocalDateTime recordedAt;
    private String locationDisplay;
}
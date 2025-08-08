package com.linky.api.admin.dto.response;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import java.time.LocalDateTime;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class RobotLocationStatisticsDto {
    private long totalRobots;
    private long onlineRobots;
    private long offlineRobots;
    private double onlineRate;
    private LocalDateTime lastUpdated;

    // 추가 통계
    private long movingRobots;
    private long stationaryRobots;
}
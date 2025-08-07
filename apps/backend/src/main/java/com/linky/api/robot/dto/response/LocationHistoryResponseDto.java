package com.linky.api.robot.dto.response;

import com.linky.api.robot.dto.LocationHistoryDto;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;
import java.util.List;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class LocationHistoryResponseDto {
    private List<LocationHistoryDto> locations;
    private long totalCount;
    private int currentPage;
    private int totalPages;
    private boolean hasNext;
    private boolean hasPrevious;

    // 추가 통계 정보
    private Integer robotId;
    private String robotCode;
    private LocalDateTime periodStart;
    private LocalDateTime periodEnd;
}
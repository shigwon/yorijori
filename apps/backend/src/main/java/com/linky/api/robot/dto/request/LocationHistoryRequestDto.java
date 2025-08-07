package com.linky.api.robot.dto.request;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import jakarta.validation.constraints.Min;
import jakarta.validation.constraints.NotNull;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class LocationHistoryRequestDto {

    @NotNull(message = "로봇 ID는 필수입니다")
    private Integer robotId;

    @Min(value = 1, message = "페이지는 1 이상이어야 합니다")
    private int page = 1;

    @Min(value = 1, message = "페이지 크기는 1 이상이어야 합니다")
    private int pageSize = 20;

    private String startDate;
    private String endDate;
}
package com.linky.api.admin.dto.request;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import jakarta.validation.constraints.Max;
import jakarta.validation.constraints.Min;
import jakarta.validation.constraints.NotNull;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class MonthlyOrderCountRequestDto {

    @NotNull(message = "년도는 필수입니다")
    @Min(value = 2020, message = "년도는 2020년 이상이어야 합니다")
    @Max(value = 2099, message = "년도는 2099년 이하여야 합니다")
    private Integer year;

    @NotNull(message = "월은 필수입니다")
    @Min(value = 1, message = "월은 1 이상이어야 합니다")
    @Max(value = 12, message = "월은 12 이하여야 합니다")
    private Integer month;
}
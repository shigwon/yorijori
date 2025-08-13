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
public class OrderLogDto {

    private String orderCode;
    private String state;
    private LocalDateTime startTime;
    private LocalDateTime endTime;
}
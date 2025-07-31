package com.linky.api.admin.model;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class DailyCountModel {
    private int dayOfWeek;
    private String dayName;
    private int count;
}
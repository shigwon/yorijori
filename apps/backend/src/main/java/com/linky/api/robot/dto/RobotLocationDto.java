package com.linky.api.robot.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class RobotLocationDto {
    int robotId;
    double latitude;
    double longitude;
}

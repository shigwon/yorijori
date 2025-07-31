package com.linky.api.robot.dto;

import lombok.Data;

@Data
public class RobotLocationDto {
    int robotId;
    double latitude;
    double longitude;
}

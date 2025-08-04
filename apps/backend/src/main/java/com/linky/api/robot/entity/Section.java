package com.linky.api.robot.entity;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class Section {
    int sectionNum;
    String sectionStatus;
}

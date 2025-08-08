package com.linky.api.robot.entity;

import jakarta.persistence.*;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import java.time.LocalDateTime;

@Entity
@Table(name = "robot_location_history", schema = "public")
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class RobotLocationHistory {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "robot_id", nullable = false)
    private Integer robotId;

    @Column(nullable = false, precision = 10)
    private Double latitude;

    @Column(nullable = false, precision = 10)
    private Double longitude;

    @Column(name = "recorded_at", nullable = false)
    private LocalDateTime recordedAt;

    @Column(name = "created_at", nullable = false)
    private LocalDateTime createdAt;

    @PrePersist
    protected void onCreate() {
        if (createdAt == null) {
            createdAt = LocalDateTime.now();
        }
        if (recordedAt == null) {
            recordedAt = LocalDateTime.now();
        }
    }
}
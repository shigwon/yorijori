package com.linky.api.robot.repository;

import com.linky.api.robot.entity.RobotLocationHistory;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.Optional;

@Repository
public interface RobotLocationHistoryRepository extends JpaRepository<RobotLocationHistory, Long> {

    Page<RobotLocationHistory> findByRobotIdOrderByRecordedAtDesc(
            Integer robotId, Pageable pageable);

    Page<RobotLocationHistory> findByRobotIdAndRecordedAtBetweenOrderByRecordedAtDesc(
            Integer robotId, LocalDateTime startDate, LocalDateTime endDate, Pageable pageable);

    Optional<RobotLocationHistory> findFirstByRobotIdOrderByRecordedAtDesc(Integer robotId);

    @Query("DELETE FROM RobotLocationHistory h WHERE h.recordedAt < :beforeDate")
    void deleteByRecordedAtBefore(@Param("beforeDate") LocalDateTime beforeDate);
}
package com.linky.api.robot.repository;

import com.linky.api.robot.entity.RobotLocationHistory;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

@Repository
public interface RobotLocationHistoryRepository extends JpaRepository<RobotLocationHistory, Long> {

    /**
     * 로봇별 위치 이력 조회 (페이징)
     */
    Page<RobotLocationHistory> findByRobotIdOrderByRecordedAtDesc(
            Integer robotId,
            Pageable pageable
    );

    /**
     * 로봇별 특정 기간 위치 이력 조회 (페이징)
     */
    Page<RobotLocationHistory> findByRobotIdAndRecordedAtBetweenOrderByRecordedAtDesc(
            Integer robotId,
            LocalDateTime startDate,
            LocalDateTime endDate,
            Pageable pageable
    );

    /**
     * 로봇의 최신 위치 조회
     */
    Optional<RobotLocationHistory> findFirstByRobotIdOrderByRecordedAtDesc(Integer robotId);

    /**
     * 모든 로봇의 최신 위치 조회 (서브쿼리 사용)
     */
    @Query(value = """
        SELECT h.* FROM robot_location_history h
        INNER JOIN (
            SELECT robot_id, MAX(recorded_at) as max_recorded_at
            FROM robot_location_history
            GROUP BY robot_id
        ) latest ON h.robot_id = latest.robot_id 
        AND h.recorded_at = latest.max_recorded_at
        ORDER BY h.robot_id ASC
        """, nativeQuery = true)
    List<RobotLocationHistory> findLatestLocationForAllRobots();

    /**
     * 특정 시간 이전 데이터 삭제 (배치 정리용)
     */
    @Query("DELETE FROM RobotLocationHistory h WHERE h.recordedAt < :beforeDate")
    void deleteByRecordedAtBefore(@Param("beforeDate") LocalDateTime beforeDate);

    /**
     * 로봇별 위치 데이터 개수 조회
     */
    long countByRobotId(Integer robotId);

    /**
     * 특정 기간 로봇별 위치 데이터 개수 조회
     */
    long countByRobotIdAndRecordedAtBetween(
            Integer robotId,
            LocalDateTime startDate,
            LocalDateTime endDate
    );
}
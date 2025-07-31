package com.linky.api.admin.service.impl;

import com.linky.api.admin.entity.Admin;
import com.linky.api.admin.model.DailyCountModel;
import com.linky.api.admin.model.HourlyCountModel;
import com.linky.api.admin.repository.AdminRepository;
import com.linky.api.admin.service.AdminService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.temporal.TemporalAdjusters;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
public class AdminServiceImpl implements AdminService {

    private final AdminRepository adminRepository;

    private static final String[] DAY_NAMES = {
            "", "일요일", "월요일", "화요일", "수요일", "목요일", "금요일", "토요일"
    };

    @Override
    public String login(String email, String password) {
        Admin admin = adminRepository.findByEmailAndPassword(email, password);
        return admin != null ? admin.getName() : null;
    }

    @Override
    public int getDailyOrderCount(LocalDate date) {
        LocalDateTime startDateTime = date.atStartOfDay();
        LocalDateTime endDateTime = date.atTime(LocalTime.MAX);

        return adminRepository.getOrderCountByPeriod(startDateTime, endDateTime);
    }

    @Override
    public int getWeeklyOrderCount(LocalDate startDate) {
        // 월요일 시작 기준으로 주간 계산
        LocalDate weekStart = startDate.with(TemporalAdjusters.previousOrSame(java.time.DayOfWeek.MONDAY));
        LocalDate weekEnd = weekStart.plusDays(6);

        LocalDateTime startDateTime = weekStart.atStartOfDay();
        LocalDateTime endDateTime = weekEnd.atTime(LocalTime.MAX);

        return adminRepository.getOrderCountByPeriod(startDateTime, endDateTime);
    }

    @Override
    public int getMonthlyOrderCount(int year, int month) {
        LocalDate monthStart = LocalDate.of(year, month, 1);
        LocalDate monthEnd = monthStart.with(TemporalAdjusters.lastDayOfMonth());

        LocalDateTime startDateTime = monthStart.atStartOfDay();
        LocalDateTime endDateTime = monthEnd.atTime(LocalTime.MAX);

        return adminRepository.getOrderCountByPeriod(startDateTime, endDateTime);
    }

    @Override
    public int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate) {
        LocalDateTime startDateTime = startDate.atStartOfDay();
        LocalDateTime endDateTime = endDate.atTime(LocalTime.MAX);

        return adminRepository.getOrderCountByPeriod(startDateTime, endDateTime);
    }

    @Override
    public List<HourlyCountModel> getHourlyOrderCountInDay(LocalDate date) {
        LocalDateTime startDateTime = date.atStartOfDay();
        LocalDateTime endDateTime = date.atTime(LocalTime.MAX);

        List<Map<String, Object>> rawData = adminRepository.getHourlyOrderCountInDay(startDateTime, endDateTime);
        List<HourlyCountModel> result = new ArrayList<>();

        // 0시부터 23시까지 모든 시간대 초기화 (0건으로)
        for (int hour = 0; hour < 24; hour++) {
            result.add(HourlyCountModel.builder()
                    .hour(hour)
                    .count(0)
                    .build());
        }

        // 실제 데이터로 업데이트
        for (Map<String, Object> row : rawData) {
            int hour = ((Number) row.get("hour")).intValue();
            int count = ((Number) row.get("count")).intValue();

            result.set(hour, HourlyCountModel.builder()
                    .hour(hour)
                    .count(count)
                    .build());
        }

        return result;
    }

    @Override
    public List<DailyCountModel> getDailyOrderCountInWeek(LocalDate startDate) {
        LocalDate weekStart = startDate.with(TemporalAdjusters.previousOrSame(java.time.DayOfWeek.MONDAY));
        LocalDate weekEnd = weekStart.plusDays(6);
        LocalDateTime startDateTime = weekStart.atStartOfDay();
        LocalDateTime endDateTime = weekEnd.atTime(LocalTime.MAX);

        List<Map<String, Object>> rawData = adminRepository.getDailyOrderCountInWeek(startDateTime, endDateTime);
        List<DailyCountModel> result = new ArrayList<>();

        // 월요일(2)부터 일요일(1)까지 모든 요일 초기화 (0건으로)
        // MySQL DAYOFWEEK: 일요일=1, 월요일=2, ..., 토요일=7
        for (int dayOfWeek = 2; dayOfWeek <= 7; dayOfWeek++) { // 월~토
            result.add(DailyCountModel.builder()
                    .dayOfWeek(dayOfWeek)
                    .dayName(DAY_NAMES[dayOfWeek])
                    .count(0)
                    .build());
        }
        result.add(DailyCountModel.builder() // 일요일
                .dayOfWeek(1)
                .dayName(DAY_NAMES[1])
                .count(0)
                .build());

        // 실제 데이터로 업데이트
        for (Map<String, Object> row : rawData) {
            int mysqlDayOfWeek = ((Number) row.get("dayOfWeek")).intValue();
            int count = ((Number) row.get("count")).intValue();

            // MySQL 요일을 리스트 인덱스로 변환
            int index;
            if (mysqlDayOfWeek == 1) { // 일요일
                index = 6; // 리스트의 마지막
            } else { // 월요일(2) ~ 토요일(7)
                index = mysqlDayOfWeek - 2; // 0 ~ 5
            }

            result.set(index, DailyCountModel.builder()
                    .dayOfWeek(mysqlDayOfWeek)
                    .dayName(DAY_NAMES[mysqlDayOfWeek])
                    .count(count)
                    .build());
        }

        return result;
    }
}
package com.linky.api.admin.service.impl;

import com.linky.api.admin.dto.response.OrderLogDto;
import com.linky.api.admin.entity.Admin;
import com.linky.api.admin.model.DailyCountModel;
import com.linky.api.admin.model.HourlyCountModel;
import com.linky.api.admin.repository.AdminRepository;
import com.linky.api.admin.service.AdminService;
import jakarta.servlet.http.HttpSession;
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
    private static final String ADMIN_SESSION_KEY = "ADMIN_INFO";
    private static final String[] DAY_NAMES = {
            "", "일요일", "월요일", "화요일", "수요일", "목요일", "금요일", "토요일"
    };

    @Override
    public String loginWithSession(String email, String password, HttpSession session) {
        try {
            Admin admin = adminRepository.findByEmailAndPassword(email, password);

            if (admin != null) {
                // 세션에 관리자 정보 저장
                session.setAttribute(ADMIN_SESSION_KEY, admin);
                session.setAttribute("ADMIN_EMAIL", admin.getEmail());
                session.setAttribute("ADMIN_NAME", admin.getName());
                session.setAttribute("LOGIN_TIME", System.currentTimeMillis());

                // 세션 타임아웃 설정 (30분)
                session.setMaxInactiveInterval(30 * 60);

                log.info("관리자 세션 로그인 성공: {} (Session ID: {})",
                        admin.getEmail(), session.getId());

                return admin.getName();
            }

            return null;
        } catch (Exception e) {
            log.error("세션 로그인 처리 중 오류 발생", e);
            return null;
        }
    }

    @Override
    public boolean logoutWithSession(HttpSession session) {
        try {
            if (session != null) {
                String adminEmail = (String) session.getAttribute("ADMIN_EMAIL");
                String sessionId = session.getId();

                session.invalidate();

                log.info("관리자 세션 로그아웃 성공: {} (Session ID: {})",
                        adminEmail, sessionId);

                return true;
            }
            return false;
        } catch (Exception e) {
            log.error("세션 로그아웃 처리 중 오류 발생", e);
            return false;
        }
    }

    @Override
    public boolean isValidSession(HttpSession session) {
        if (session == null) {
            return false;
        }

        try {
            Admin adminInfo = (Admin) session.getAttribute(ADMIN_SESSION_KEY);
            return adminInfo != null;
        } catch (Exception e) {
            log.warn("세션 유효성 검사 중 오류 발생", e);
            return false;
        }
    }

    @Override
    public int getDailyOrderCount(LocalDate date) {
        LocalDateTime startDateTime = date.atStartOfDay();
        LocalDateTime endDateTime = date.atTime(LocalTime.MAX);

        return adminRepository.getOrderCountByPeriod(startDateTime, endDateTime);
    }

    @Override
    public int getWeeklyOrderCount(LocalDate startDate) {
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

        for (int hour = 0; hour < 24; hour++) {
            result.add(HourlyCountModel.builder()
                    .hour(hour)
                    .count(0)
                    .build());
        }

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

        for (int dayOfWeek = 2; dayOfWeek <= 7; dayOfWeek++) {
            result.add(DailyCountModel.builder()
                    .dayOfWeek(dayOfWeek)
                    .dayName(DAY_NAMES[dayOfWeek])
                    .count(0)
                    .build());
        }
        result.add(DailyCountModel.builder()
                .dayOfWeek(1)
                .dayName(DAY_NAMES[1])
                .count(0)
                .build());

        // 실제 데이터로 업데이트
        for (Map<String, Object> row : rawData) {
            int mysqlDayOfWeek = ((Number) row.get("dayOfWeek")).intValue();
            int count = ((Number) row.get("count")).intValue();

            int index;
            if (mysqlDayOfWeek == 1) {
                index = 6;
            } else {
                index = mysqlDayOfWeek - 2;
            }

            result.set(index, DailyCountModel.builder()
                    .dayOfWeek(mysqlDayOfWeek)
                    .dayName(DAY_NAMES[mysqlDayOfWeek])
                    .count(count)
                    .build());
        }

        return result;
    }

    @Override
    public List<OrderLogDto> getOrderLogs(String state, LocalDateTime startTime, LocalDateTime endTime, Integer robotId) {
        // Repository에서 데이터 조회
        List<Map<String, Object>> rawData = adminRepository.getOrderLogs(state, startTime, endTime, robotId);

        // Map을 OrderLogDto로 변환
        return rawData.stream()
                .map(this::convertToOrderLogDto)
                .toList();
    }

    private OrderLogDto convertToOrderLogDto(Map<String, Object> row) {
        return OrderLogDto.builder()
                .orderCode((String) row.get("orderCode"))
                .state((String) row.get("state"))
                .startTime((LocalDateTime) row.get("startTime"))
                .endTime((LocalDateTime) row.get("endTime"))
                .build();
    }
}
package com.linky.api.admin.service.impl;

import com.linky.api.admin.entity.Admin;
import com.linky.api.admin.repository.AdminRepository;
import com.linky.api.admin.service.AdminService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.temporal.TemporalAdjusters;

@Slf4j
@Service
@RequiredArgsConstructor
public class AdminServiceImpl implements AdminService {

    private final AdminRepository adminRepository;

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
}
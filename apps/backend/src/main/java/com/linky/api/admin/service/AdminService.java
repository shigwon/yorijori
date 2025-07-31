package com.linky.api.admin.service;

import com.linky.admin.grpc.DailyCount;
import com.linky.admin.grpc.HourlyCount;

import java.time.LocalDate;
import java.util.List;

public interface AdminService {

    String login(String email, String password);
    int getDailyOrderCount(LocalDate date);
    int getWeeklyOrderCount(LocalDate startDate);
    int getMonthlyOrderCount(int year, int month);
    int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate);

    List<HourlyCount> getHourlyOrderCountInDay(LocalDate date);
    List<DailyCount> getDailyOrderCountInWeek(LocalDate startDate);
}
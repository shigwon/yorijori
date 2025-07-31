package com.linky.api.admin.service;

import com.linky.api.admin.model.DailyCountModel;
import com.linky.api.admin.model.HourlyCountModel;

import java.time.LocalDate;
import java.util.List;

public interface AdminService {

    String login(String email, String password);
    int getDailyOrderCount(LocalDate date);
    int getWeeklyOrderCount(LocalDate startDate);
    int getMonthlyOrderCount(int year, int month);
    int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate);

    List<HourlyCountModel> getHourlyOrderCountInDay(LocalDate date);
    List<DailyCountModel> getDailyOrderCountInWeek(LocalDate startDate);
}
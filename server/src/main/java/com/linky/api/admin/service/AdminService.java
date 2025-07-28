package com.linky.api.admin.service;

import java.time.LocalDate;

public interface AdminService {

    int getDailyOrderCount(LocalDate date);
    int getWeeklyOrderCount(LocalDate startDate);
    int getMonthlyOrderCount(int year, int month);
    int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate);
}
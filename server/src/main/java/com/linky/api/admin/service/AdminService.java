package com.linky.api.admin.service;

import java.time.LocalDate;

public interface AdminService {

    String login(String email, String password);
    int getDailyOrderCount(LocalDate date);
    int getWeeklyOrderCount(LocalDate startDate);
    int getMonthlyOrderCount(int year, int month);
    int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate);
}
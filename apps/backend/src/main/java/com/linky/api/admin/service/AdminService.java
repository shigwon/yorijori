package com.linky.api.admin.service;

import com.linky.api.admin.dto.response.OrderLogDto;
import com.linky.api.admin.model.DailyCountModel;
import com.linky.api.admin.model.HourlyCountModel;
import jakarta.servlet.http.HttpSession;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

public interface AdminService {

    String loginWithSession(String email, String password, HttpSession session);
    boolean logoutWithSession(HttpSession session);
    boolean isValidSession(HttpSession session);   int getDailyOrderCount(LocalDate date);
    int getWeeklyOrderCount(LocalDate startDate);
    int getMonthlyOrderCount(int year, int month);
    int getOrderCountByPeriod(LocalDate startDate, LocalDate endDate);

    List<HourlyCountModel> getHourlyOrderCountInDay(LocalDate date);
    List<DailyCountModel> getDailyOrderCountInWeek(LocalDate startDate);
    List<OrderLogDto> getOrderLogs(String state, LocalDateTime startTime, LocalDateTime endTime, Integer robotId);

}
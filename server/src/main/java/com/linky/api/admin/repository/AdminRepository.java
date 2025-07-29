package com.linky.api.admin.repository;

import org.apache.ibatis.annotations.Mapper;

import java.time.LocalDateTime;

@Mapper
public interface AdminRepository {
    int getOrderCountByPeriod(LocalDateTime startDateTime, LocalDateTime endDateTime);
}

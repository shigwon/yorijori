package com.linky.api.admin.repository;

import com.linky.api.admin.entity.Admin;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param

import java.time.LocalDateTime;

@Mapper
public interface AdminRepository {
    Admin findByEmailAndPassword(@Param("email") String email, @Param("password") String password);
    int getOrderCountByPeriod(@Param("startDateTime") LocalDateTime startDateTime,
                              @Param("endDateTime") LocalDateTime endDateTime);
}

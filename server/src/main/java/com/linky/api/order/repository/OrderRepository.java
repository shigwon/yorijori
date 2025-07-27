package com.linky.api.order.repository;

import org.apache.ibatis.annotations.Mapper;
import org.springframework.data.repository.query.Param;

@Mapper
public interface OrderRepository {
    public int updateDeliveryState(int id, String state);
}

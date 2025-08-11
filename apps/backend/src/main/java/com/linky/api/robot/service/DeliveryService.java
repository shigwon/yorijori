package com.linky.api.robot.service;

public interface DeliveryService {
    void resetTimer(int robotId);
    void removeTimer(int robotId);
    void interruptTimer(int robotId);
    void sendOrderList(int robotId);
};

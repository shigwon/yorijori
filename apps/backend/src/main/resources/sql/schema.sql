CREATE DATABASE IF NOT EXISTS linky
    DEFAULT CHARACTER SET = utf8mb4
    COLLATE = utf8mb4_0900_ai_ci;

USE linky;

CREATE TABLE IF NOT EXISTS `Robots` (
    `id`          INT           NOT NULL COMMENT '로봇식별번호',
    `code`        VARCHAR(20)   NULL COMMENT '로봇코드',
    `status`      VARCHAR(20)   NULL COMMENT '로봇상태',
    `time`        INT           NULL COMMENT '주행시간',
    `count_food`  INT           NULL COMMENT '음식배정수',
    PRIMARY KEY (`id`)
);

CREATE TABLE IF NOT EXISTS `orders` (
    `id`                  INT             NOT NULL AUTO_INCREMENT COMMENT '주문식별번호',
    `robot_id`            INT             NOT NULL COMMENT '로봇식별번호',
    `code`                VARCHAR(20)     NULL COMMENT '주문번호',
    `tel`                 VARCHAR(20)     NOT NULL COMMENT '전화번호',
    `customer_latitude`   DECIMAL(10,6)   NULL COMMENT '고객위치위도',
    `customer_longitude`  DECIMAL(10,6)   NULL COMMENT '고객위치경도',
    `state`               ENUM('READY_FOR_PICKUP', 'DELIVERING', 'DELIVERED') NULL COMMENT '배달상태',
    `start_time`          DATETIME        NULL COMMENT '배달출발시간',
    `end_time`            DATETIME        NULL COMMENT '배달완료시간',
    `face_image_url`      VARCHAR(255)    NULL COMMENT '얼굴사진',
    `food_image_url`      VARCHAR(255)    NULL COMMENT '음식사진',
    `space_num`           INT             NULL COMMENT '음식함번호',
    `created_at`          DATETIME        NULL DEFAULT NOW() COMMENT '생성일시',
    PRIMARY KEY (`id`),
    CONSTRAINT `fk_orders_robot_id` FOREIGN KEY (`robot_id`) REFERENCES `Robots`(`id`)
);

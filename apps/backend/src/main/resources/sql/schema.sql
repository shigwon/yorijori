CREATE DATABASE IF NOT EXISTS linky
    DEFAULT CHARACTER SET = utf8mb4
    COLLATE = utf8mb4_0900_ai_ci;

USE linky;

CREATE TABLE IF NOT EXISTS `robots` (
    `id`          INT           NOT NULL COMMENT '로봇식별번호',
    `code`        VARCHAR(20)   NULL COMMENT '로봇코드',
    `status`      ENUM('WORKING', 'WAITING') NULL COMMENT '로봇상태',
    `last_charge_time`	DATETIME     NULL COMMENT '마지막 충전 시각',
    PRIMARY KEY (`id`)
    );

CREATE TABLE IF NOT EXISTS `orders` (
    `id`                  INT             NOT NULL AUTO_INCREMENT COMMENT '주문식별번호',
    `robot_id`            INT             NOT NULL COMMENT '로봇식별번호',
    `code`                VARCHAR(20)     NULL COMMENT '주문번호' UNIQUE,
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

CREATE TABLE IF NOT EXISTS `reviews` (
    `id`	INT	NOT NULL AUTO_INCREMENT PRIMARY KEY,
    `order_id`	INT	NOT NULL,
    `rating`	INT	NULL,
    `content`	TEXT	NULL,
    `created_at`	DATETIME	NULL,
    CONSTRAINT `fk_reviews_orders_id` FOREIGN KEY (`order_id`) REFERENCES `orders`(`id`)
    );

CREATE TABLE IF NOT EXISTS `admins` (
    `id`          INT          NOT NULL AUTO_INCREMENT COMMENT '관리자식별번호',
    `email`       VARCHAR(50)  NOT NULL COMMENT '이메일',
    `password`    VARCHAR(60)  NOT NULL COMMENT '해시된 비밀번호 (bcrypt)',
    `name`        VARCHAR(20)  NOT NULL COMMENT '관리자명',
    `created_at`  DATETIME     NULL DEFAULT NOW() COMMENT '생성일시',
    `updated_at`  DATETIME     NULL DEFAULT NOW() ON UPDATE NOW() COMMENT '수정일시',
    PRIMARY KEY (`id`),
    UNIQUE KEY `uk_admins_email` (`email`)
    );

package com.linky.api.order.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.message.service.MessageService;
import com.linky.api.mqtt.service.MqttService;
import com.linky.api.order.dto.request.CreateOrderRequestDto;
import com.linky.api.order.dto.request.RunImageToOcrRequestDto;
import com.linky.api.order.dto.request.UpdateLocationRequestDto;
import com.linky.api.order.dto.response.RunImageToOcrResponseDto;
import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.service.OcrService;
import com.linky.api.order.service.OrderService;
import com.linky.api.robot.entity.Section;
import com.linky.api.robot.service.DeliveryService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.Map;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("api/v1/orders")
public class OrderController {
    private final OcrService ocrService;
    private final OrderService orderService;
    private final DeliveryService deliveryService;
    private final MessageService messageService;
    private final MqttService mqttService;
    private final OrderMapper orderMapper;

    @PostMapping("/ocr")
    public ResponseEntity<ApiResponseEntity<RunImageToOcrResponseDto>> runImageToOcr(@RequestBody RunImageToOcrRequestDto runImageToOcrRequest) {

        Map<String, String> ocrResult = ocrService.sendImageToOcr(runImageToOcrRequest.image());
        RunImageToOcrResponseDto runImageToOcrResponse = new RunImageToOcrResponseDto(
                ocrResult.getOrDefault("code", ""),
                ocrResult.getOrDefault("tel", "")
        );
        return ApiResponseEntity.successResponseEntity(runImageToOcrResponse);
    }

    @PostMapping("/create")
    public ResponseEntity<ApiResponseEntity<Object>> createOrder(@RequestBody CreateOrderRequestDto createOrderRequest) {

        boolean success = orderService.createOrder(createOrderRequest);
        if(success) {
            int orderId = orderService.searchOrderId(createOrderRequest);
            Order order = orderMapper.toEntity(createOrderRequest);
            int sectionNum = orderService.searchOrderCreateActivate(order.getRobotId());
            messageService.messageSend(order.getTel(), "https://naver.com?orderId=" + orderId +"&robotId=" + order.getRobotId() + "&code=" + order.getCode() + "&sectionNum=" + sectionNum);
            mqttService.sendCloseSection(order.getRobotId(), Section.builder().sectionNum(sectionNum).sectionStatus("CLOSE").build());
            return ApiResponseEntity.successResponseEntity("주문 생성에 성공했습니다.");
        } else {
            return ApiResponseEntity.failResponseEntity("주문 생성에 실패했습니다.");
        }
    }

    @PatchMapping("/{orderId}/location")
    public ResponseEntity<ApiResponseEntity<Object>> updateLocation(@PathVariable("orderId") int orderId, @RequestBody UpdateLocationRequestDto updateLocationRequest) {

        boolean success = orderService.updateLocation(orderId, updateLocationRequest);
        if(success) {
            int orderUpdateCount = orderService.searchOrderUpdateCount(updateLocationRequest.robotId());
            if(orderUpdateCount >= 3) {
                deliveryService.interruptTimer(updateLocationRequest.robotId());
            } else {
                deliveryService.resetTimer(updateLocationRequest.robotId());
            }
            return ApiResponseEntity.successResponseEntity("고객 위치가 성공적으로 전송되었습니다.");
        }
        return ApiResponseEntity.failResponseEntity("고객 위치 업데이트에 실패했습니다.");
    }
}

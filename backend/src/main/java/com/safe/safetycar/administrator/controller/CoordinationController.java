package com.safe.safetycar.administrator.controller;

import com.safe.safetycar.administrator.dto.CoordinationDto;
import com.safe.safetycar.administrator.service.SseService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;


@RestController
@RequiredArgsConstructor
@Slf4j
@Tag(name = "이벤트 감지 컨트롤러", description = "좌표 수신 API")
public class CoordinationController {

    private final RestTemplate restTemplate;
    private final SseService sseService;

    @Operation(summary = "좌표 수신", description = "쓰러진 사람에 대해 좌표 값을 수신합니다.")
    @PostMapping("/coordinate")
    public ResponseEntity<String> receiveCoordinate(@RequestBody CoordinationDto coordinationDto) {
        float x = coordinationDto.getX();
        float y = coordinationDto.getY();
        sseService.sendCoordinate(x, y);

        return ResponseEntity.status(HttpStatus.OK).body("x 좌표: " + x + ", y 좌표: " + y);
    }


    @GetMapping(value = "/sse", produces = MediaType.TEXT_EVENT_STREAM_VALUE)
    public SseEmitter subscribe(@RequestParam String clientId) {
        return sseService.subscribe(clientId);
    }
}

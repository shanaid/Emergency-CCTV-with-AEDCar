package com.safe.safetycar.administrator.service;

import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Service
@EnableScheduling
public class SseService {
    private final Map<String, SseEmitter> emitters = new ConcurrentHashMap<>();
    private final ObjectMapper objectMapper = new ObjectMapper(); // JSON 변환을 위한 ObjectMapper

    public SseEmitter subscribe(String clientId) {
        SseEmitter emitter = new SseEmitter(30000L); // 타임아웃을 30초로 설정

        // 초기 메시지 전송 (더미 데이터)
        try {
            String initialMessage = objectMapper.writeValueAsString(Map.of("type", "initial", "message", "연결이 성공적으로 이루어졌습니다."));
            emitter.send(initialMessage);
        } catch (IOException e) {
            emitter.completeWithError(e);
        }

        emitters.put(clientId, emitter);

        emitter.onCompletion(() -> emitters.remove(clientId));
        emitter.onTimeout(() -> {
            emitters.remove(clientId);
            System.out.println("Timeout for client: " + clientId);
        });

        return emitter;
    }

    @Scheduled(fixedRate = 20000) // 20초마다 실행
    public void sendKeepAlive() {
        for (String clientId : emitters.keySet()) {
            SseEmitter emitter = emitters.get(clientId);
            if (emitter != null) {
                try {
                    String keepAliveMessage = objectMapper.writeValueAsString(Map.of("type", "keep-alive", "message", "keep-alive"));
                    emitter.send(keepAliveMessage);
                } catch (IOException e) {
                    emitters.remove(clientId); // 오류 발생 시 emitter 제거
                    System.out.println("클라이언트 연결 종료: " + clientId);
                }
            }
        }
    }



    public void sendCoordinate(float x, float y) {
        for (Map.Entry<String, SseEmitter> entry : emitters.entrySet()) {
            SseEmitter emitter = entry.getValue();
            try {
                String coordinateMessage = objectMapper.writeValueAsString(Map.of("type", "coordinate", "x", x, "y", y));
                emitter.send(coordinateMessage);
            } catch (IOException e) {
                emitters.remove(entry.getKey());
            }
        }
    }
}

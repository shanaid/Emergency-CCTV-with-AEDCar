package com.safe.safetycar.config;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.scheduling.annotation.EnableAsync;
import org.springframework.scheduling.concurrent.ThreadPoolTaskExecutor;

@Configuration
@EnableAsync
public class TreadConfig {

    @Bean
    public ThreadPoolTaskExecutor taskExecutor() {
        ThreadPoolTaskExecutor executor = new ThreadPoolTaskExecutor();
        executor.setCorePoolSize(20); // 코어 스레드 풀 크기 설정
        executor.setMaxPoolSize(30); // 최대 스레드 풀 크기 설정
        executor.setQueueCapacity(500); // 작업 큐 용량 설정
        executor.setThreadNamePrefix("AsyncThread-"); // 스레드 이름 접두사 설정

        // 작업이 완료된 후 스레드 풀이 종료될 때까지 대기할 시간 설정 (단위: 초)
        executor.setAwaitTerminationSeconds(60);

        executor.initialize();
        return executor;
    }

}

package com.safe.safetycar;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableAsync;

@SpringBootApplication
@EnableAsync
public class SafetycarApplication {

    public static void main(String[] args) {
        SpringApplication.run(SafetycarApplication.class, args);
    }

}

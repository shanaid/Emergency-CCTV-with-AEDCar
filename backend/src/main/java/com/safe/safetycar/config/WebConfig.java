package com.safe.safetycar.config;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.CorsRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebConfig implements WebMvcConfigurer {
    @Value("${react.server.address}")
    private String reactServerAddress;

    @Override
    public void addCorsMappings(CorsRegistry registry) {
        registry.addMapping("/sse")
                .allowedOrigins(reactServerAddress) // 리액트 앱의 주소
                .allowedMethods("GET", "POST")
                .allowCredentials(true);
    }
}
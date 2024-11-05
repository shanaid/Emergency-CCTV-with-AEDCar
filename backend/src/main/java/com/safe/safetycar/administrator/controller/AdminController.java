package com.safe.safetycar.administrator.controller;

import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

@Slf4j
@RestController
public class AdminController {

    /**
     * API 테스트용 코드입니다.
     * @return
     */
    @GetMapping("/test")
    public ResponseEntity<String> test() {
        System.out.println("TEST");
        return ResponseEntity.ok("Success");
    }
}

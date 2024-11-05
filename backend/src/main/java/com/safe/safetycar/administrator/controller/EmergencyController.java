package com.safe.safetycar.administrator.controller;

import com.safe.safetycar.administrator.service.EmergencyService;
import jakarta.validation.Valid;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/sms")

public class EmergencyController {

    private final EmergencyService emergencyService;
    public EmergencyController(@Autowired EmergencyService emergencyService) {
        this.emergencyService = emergencyService;
    }

    @PostMapping("/send")
    public ResponseEntity<?> sendSMS(@RequestParam String space){
        emergencyService.SendSms(space);
        return ResponseEntity.ok("신고했습니다.");
    }

}

package com.safe.safetycar.administrator.service;


import com.safe.safetycar.config.SMSConfig;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class EmergencyService {
    private final SMSConfig smsConfig;

    //의존성 주입
    public EmergencyService(@Autowired SMSConfig smsConfig) {
        this.smsConfig = smsConfig;
    }

    public void SendSms(String space) {
        String phoneNum = "01030479192"; // SmsrequestDto에서 전화번호를 가져온다.
        smsConfig.sendSMS(phoneNum, space); // SMS 인증 유틸리티를 사용하여 SMS 발송
    }


}

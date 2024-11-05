package com.safe.safetycar.streaming.udp.filter;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.service.CameraService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.integration.core.MessageSelector;
import org.springframework.messaging.Message;
import org.springframework.stereotype.Service;

@Service
public class UDPFilter implements MessageSelector {

    @Autowired
    private CameraService cameraService;


    @Override
    public boolean accept(Message<?> message) {
        return cameraService.checkWhite((String)message.getHeaders().get("ip_address"));
    }
}

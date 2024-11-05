package com.safe.safetycar.streaming.udp;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.Image.Image;
import com.safe.safetycar.streaming.Image.ImageManager;
import com.safe.safetycar.streaming.socket.manager.WebSocketManager;
import com.safe.safetycar.streaming.udp.filter.UDPFilter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.integration.annotation.MessageEndpoint;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessagingException;
import org.springframework.messaging.handler.annotation.Headers;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.Map;

@MessageEndpoint
public class UdpInboundMessageHandler {
    @Autowired
    private UDPFilter udpFilter;
    @Autowired
    private WebSocketManager wsm;
    @Autowired
    private ImageManager imageManager;

    private final static LogManager logManager = new LogManager(UdpInboundMessageHandler.class);

    public UdpInboundMessageHandler() {
        logManager.setInterval(LogManager.LOG_TYPE.INFO, 200, "image received");
    }

    //headerMap 내용 example
    //{ip_packetAddress=/127.0.0.1:58011, ip_address=127.0.0.1, id=6626e9b4-fac2-e7d2-d2a0-afd7ce5fa366, ip_port=58011, ip_hostname=127.0.0.1, timestamp=1727833051957}

    /**
     * UDP 메세지를 수신 받는다.
     * @param message
     * @param headerMap
     * @throws IOException
     */
    @ServiceActivator(inputChannel = "inboundChannel")
    public void handleMessage(Message message, @Headers Map<String, Object> headerMap) throws IOException {
        if(!udpFilter.accept(message)) {
            logManager.sendLog("ACCESS DENIED", LogManager.LOG_TYPE.WARN);
            return;
        }
        ByteArrayInputStream bis = new ByteArrayInputStream((byte[])message.getPayload());
        //endflag를 byte로 받으면 아래 if문에서 정상적으로 식별이 되지 않는다. 카메라에서 255(모든 비트를 1)으로 설정하고 전송하는데 if문에서 음수로 판단하는 것 같다...
        //c++에서는 BYTE는 unsigned char(0 ~ 255) 으로 선언되어있다. 하지만 자바에서는 byte의 표현범위가 -127 ~ 128이기 때문에 음수로 인식되어 아래 if문에서 항상 거짓이게 된다...
        //해결 방법은 다음과 같다.
        //1. int형으로 변환하면 원하는 값을 얻을 수 있다.
        //2. 음수로 인식되는 범위라면 부호 비트를 반전하기
        //3. char 자료형을 사용하기
        // 가장 간단한 1번을 사용하기로 하였다.
        
        //endflag 동작 원리 - SYNC 맞추기
        //송신측에서 프레임을 세그먼트로 나누어서 보낼 때 endflag 255 또는 0을 전송한다.
        //새로운 프레임을 전송할 때 이전에 보낸 endflag를 반전시켜서 전송하며 이를 반복한다.
        //수신측에서는 이전 프레임의 플레그와 비교해서 반전이 일어났다면 이전 프레임이 모두 수신되었다고 판단하고 클라이언트로 전송을 한다.
        //수신과 송신이 비동기적으로 나타나므로 수신용 byte[]와 송신용 byte[]로 나누어 관리하여 전송하여 무결성을 높인다.
        //자세한 로직은 ImageManager, Image 클래스를 참고
        int endflag = bis.read();
        byte cameraId = (byte)bis.read();
        byte segNum = (byte)bis.read();

        if(segNum >= Image.MAX_SEG_NUM) {
            logManager.sendLog("segNum is greater than MAX_SEG_NUM", LogManager.LOG_TYPE.ERROR);
            return;
        }

        if(endflag != imageManager.getFlag(cameraId)){
            imageManager.setFlag(cameraId, endflag);
//            imageManager.read(cameraId).setNextCacheIdx();
            logManager.sendInterval();  //수신되고 있음을 주기적으로 표시 (매 프레임마다 출력하지 않기)
            wsm.sendFrame(cameraId);
        }
        imageManager.write(bis, cameraId, segNum);
//        System.out.println(segNum);
    }
}

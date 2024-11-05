package com.safe.safetycar.streaming.socket.controller;

import com.safe.safetycar.response_template.BaseResponseTemplate;
import com.safe.safetycar.streaming.Image.ImageManager;
import com.safe.safetycar.streaming.response.DisconnectResponse;
import com.safe.safetycar.streaming.socket.manager.WebSocketManager;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
public class FrameController {

    @Value("${react.server.address}")
    private String reactServerAddress;
    @Autowired
    private ImageManager imageManager;
    @Autowired
    private WebSocketManager wsm;

    @CrossOrigin(origins = "https://j11b209.p.ssafy.io/")
    @GetMapping("/test2")
    public ResponseEntity<String> test() {
        System.out.println("TEST");
        return ResponseEntity.ok("Success");
    }

    @CrossOrigin(origins = "https://j11b209.p.ssafy.io/")
    @GetMapping(
            value = "/{cameraId}",
            produces = MediaType.IMAGE_JPEG_VALUE
    )
    public @ResponseBody byte[] getFrame(@PathVariable("cameraId") String cameraId) {
//        InputStream in = new ByteArrayInputStream(UdpInboundMessageHandler.camera_data_assembled[Integer.parseInt(cameraId)]);
//        return UdpInboundMessageHandler.camera_data_assembled[Integer.parseInt(cameraId)];
        return imageManager.read(Byte.parseByte(cameraId)).getPrevData();
    }

    @GetMapping("/clientCount")
    public @ResponseBody ResponseEntity<BaseResponseTemplate> getClientCount() {

        return new ResponseEntity<>(new BaseResponseTemplate(wsm.getClientSize() + "", 200), HttpStatus.BAD_REQUEST);
    }
}

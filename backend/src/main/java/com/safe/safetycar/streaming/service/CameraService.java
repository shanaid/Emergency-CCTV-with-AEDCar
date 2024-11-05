package com.safe.safetycar.streaming.service;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.Image.ImageManager;
import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.ConcurrentHashMap;

@Service
public class CameraService {

    private LogManager logManager = new LogManager(CameraService.class);
    private ConcurrentHashMap<String, Byte> whiteList = new ConcurrentHashMap<>();
    @Autowired
    private ImageManager imageManager;

    /**
     * 주어진 아이피가 연결된 목록에 존재하는지 여부
     * @param ip
     * @return 해당 아이피가 연결되어 있다면 true, 그렇지 않다면 false
     */
    public boolean checkWhite(String ip) {
//        logManager.sendLog("try : "+ ip, LogManager.LOG_TYPE.WARN);
//        logManager.sendLog(Arrays.toString(whiteList.toArray()), LogManager.LOG_TYPE.INFO);

        return whiteList.containsKey(ip);
    }

    /**
     * 카메라를 등록한다. 새로운 카메라라면 아이디를 발급하여 반환하고 새로운 카메라를 위한 공간을 할당한다.
     * @param ip    카메라 아이피
     * @return      생성된 카메라의 번호
     */
    public byte addCamera(String ip) {
        if(!whiteList.containsKey(ip)) {
            whiteList.put(ip, imageManager.initCamera());   //카메라를 위한 공간 할당 및 번호 등록
        } else logManager.sendLog("Camera Already Registered!", LogManager.LOG_TYPE.WARN);

        logManager.sendLog(whiteList.toString(), LogManager.LOG_TYPE.INFO);

        return whiteList.get(ip);
    }

    public int getSize() {
        return whiteList.size();
    }

    public boolean removeCamera(String ip) {
        Byte index = whiteList.remove(ip);
        if(index != null) {
            logManager.sendLog("Camera Removed!" + ip + " - " + index, LogManager.LOG_TYPE.INFO);
            imageManager.remove(index);
            return true;
        }
        return false;
    }

}

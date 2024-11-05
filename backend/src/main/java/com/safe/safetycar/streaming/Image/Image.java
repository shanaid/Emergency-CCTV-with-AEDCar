package com.safe.safetycar.streaming.Image;

import com.safe.safetycar.log.LogManager;
import lombok.Getter;
import lombok.Setter;

import java.io.ByteArrayInputStream;

@Getter
public class Image {
    //미리 공간을 열어놓기 640*480 크기의 jpg를 테스트해본결과 약 60000 바이트가 나올때가 있고 20000 바이트가 될때가 있다.
    //최악의 경우를 가정해서 넉넉하게 공간을 만들어놓기
    public static int MTU = 1500;   //Maximum Transmission Unit
    public static short UDP_HEADER_SIZE = 28;   //Maximum Transmission Unit
    public static short INFO_SIZE = 4;

    public static int IMG_SEG_SIZE = MTU - (UDP_HEADER_SIZE + INFO_SIZE);
    public static short MAX_SEG_NUM = 150;
    public static short HEADER_SIZE = 1;     //카메라 정보를 담을 커스텀 헤더 크기
    private static final int MAX_CACHE = 2;

    private byte[][] data = new byte[MAX_CACHE][(MAX_SEG_NUM * IMG_SEG_SIZE) + HEADER_SIZE];
    private static int cacheIdx = 0;
    @Setter
    private int flag = 0;

    @Setter
    private boolean isOpen = false;

    public Image(byte id) {
        for(int i = 0; i < MAX_CACHE; i++) {data[i][0]=id;}
        isOpen = true;
        cacheIdx = 0;
    }

    public byte[] getPrevData() {
        return data[getPrevCacheIdx()];
    }

    public byte[] getCurrentData() {
        return data[flag == 0 ? 1 : 0];
//        return data[0];
    }

    public void setNextCacheIdx() {
        cacheIdx = (cacheIdx + 1) % MAX_CACHE;
    }

    public int getPrevCacheIdx() {
        return (cacheIdx - 1 + MAX_CACHE) % MAX_CACHE;
    }
    public int getNextCacheIdx() {
        return (cacheIdx + 1) % MAX_CACHE;
    }

    /**
     * 주어진 세그먼트를 입력받는다.
     * @param bis       바이트 데이터
     * @param segNum    읽을 데이터 번호
     * @return          정상적으로 읽었다면 읽은 만큼의 바이트 그렇지 않다면  -1, ByteArrayInputStream.read()와 같다.
     */
    public int write(ByteArrayInputStream bis, byte segNum) {
        int idx = bis.read();

//        idx = idx == cacheIdx ? cacheIdx : getPrevCacheIdx();
//        if((idx + 1) % MAX_CACHE == cacheIdx) idx = getPrevCacheIdx();
//        else if((idx - 1 + MAX_CACHE) % MAX_CACHE == cacheIdx) idx = getNextCacheIdx();
        return bis.read(data[flag == 0 ? 0 : 1], (segNum * IMG_SEG_SIZE) + HEADER_SIZE, IMG_SEG_SIZE);
//        return bis.read(data[0], (segNum * IMG_SEG_SIZE) + HEADER_SIZE, IMG_SEG_SIZE);
    }

}

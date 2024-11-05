package com.safe.safetycar.log;

import lombok.AllArgsConstructor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LogManager {
    private Logger LOGGER = null;
    IntervalData interval = null;

    public LogManager(Class<?> clazz) {
        LOGGER = LoggerFactory.getLogger(clazz);
    }

    /**
     * 반복될 메세지 정의
     * 1000으로 설정 시 해당 로그를 1000번 마다 1번 출력한다.
     * 너무 많은 로그가 발생해서 디버그가 어렵지만 해당 기능이 일정 주기로 동작하고 있음을 로그창에서 확인하기 위해 사용
     * @param type          로그 타입
     * @param interval      출력 빈도
     * @param log           메세지
     */
    public void setInterval(LOG_TYPE type, int interval, String log) {
        this.interval = new IntervalData(type, interval, 0, log);
    }

    /**
     * 빈도 로그 출력
     */
    public void sendInterval() {
        if(this.interval == null) return;
        this.interval.send(LOGGER);

    }


    /**
     * 로그를 생성합니다.
     * @param log   출력할 메세지
     * @param type  로그 타입
     */
    public void sendLog(String log, LOG_TYPE type) {
        type.log(LOGGER, log);
    }



    /**
     * 로그 타입을 정의합니다.
     */
    public enum LOG_TYPE {
        WARN {
            void log(Logger logger, String log) { logger.warn(log); }
        },
        ERROR {
            void log(Logger logger, String log) { logger.error(log); }
        },
        INFO {
            void log(Logger logger, String log) { logger.info(log); }
        };
        
        abstract void log(Logger logger, String log);
    }


}

/**
 * 반복되는 메세지를 관리하는 클래스
 */
@AllArgsConstructor
class IntervalData {
    LogManager.LOG_TYPE type;
    int interval_count;
    int count;
    String log_msg;

    /**
     * 로그가 반복된 횟수를 검사하여 로그 발생 시점을 검사한다.
     * 만약 로그 발생 시점이라면 true 반환 후 관련 변수를 초기화한다.
     * @return 발생 시점 여부
     */
    boolean send(Logger logger) {
        if(isLogger_Available()) {
            count = 0;
            type.log(logger, log_msg);
            return true;
        } else {
            count++;
            return false;
        }
    }

    /**
     * 로그가 얼마나 반복되었는지 체크
     * @return 로그가 임계치에 도달하면 true
     */
    boolean isLogger_Available() {
        return count >= interval_count;
    }
}


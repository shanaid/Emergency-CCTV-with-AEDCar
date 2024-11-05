package com.safe.safetycar.filter;

import com.safe.safetycar.log.LogManager;
import jakarta.servlet.*;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.util.PatternMatchUtils;

import java.io.IOException;

@Slf4j
public class AccessFilter implements Filter {

    private static LogManager logManager = new LogManager(AccessFilter.class);

    @Value("${cctv.client.token}")
    private String TOKEN;

    private static final String[] accessRequireURIs = {"/test2", "/test3"};

    @Override
    public void doFilter(ServletRequest servletRequest, ServletResponse servletResponse, FilterChain filterChain) throws IOException, ServletException {
//        logManager.sendLog("DO Filter!", LogManager.LOG_TYPE.INFO);
        logManager.sendLog(servletRequest.getAttributeNames().toString(), LogManager.LOG_TYPE.INFO);

        //TODO : token check


        filterChain.doFilter(servletRequest, servletResponse);
    }

    private boolean isFilterURI(String requestURI) {
        return PatternMatchUtils.simpleMatch(accessRequireURIs, requestURI);
    }
}

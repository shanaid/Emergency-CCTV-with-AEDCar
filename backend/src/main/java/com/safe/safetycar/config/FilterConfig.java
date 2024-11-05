package com.safe.safetycar.config;

import com.safe.safetycar.filter.AccessFilter;
import org.springframework.boot.web.servlet.FilterRegistrationBean;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class FilterConfig {

    @Bean
    public FilterRegistrationBean firstFilterRegister() {
        FilterRegistrationBean registrationBean = new FilterRegistrationBean(new AccessFilter());
        registrationBean.setOrder(1);
        return registrationBean;
    }
}

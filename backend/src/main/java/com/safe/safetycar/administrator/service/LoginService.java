package com.safe.safetycar.administrator.service;

import com.safe.safetycar.administrator.entity.UserConfig;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class LoginService {

    private final UserConfig userConfig;

    public boolean login(String password) {
        return userConfig.getPassword().equals(password);
    }
}

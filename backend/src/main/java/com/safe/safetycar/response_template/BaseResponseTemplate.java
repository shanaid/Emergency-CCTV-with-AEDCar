package com.safe.safetycar.response_template;

import lombok.*;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class BaseResponseTemplate {
    String message;
    int status_code;
}

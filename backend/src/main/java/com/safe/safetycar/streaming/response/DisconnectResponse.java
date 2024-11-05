package com.safe.safetycar.streaming.response;

import com.safe.safetycar.response_template.BaseResponseTemplate;
import lombok.Getter;
import lombok.Setter;


@Getter
@Setter
public class DisconnectResponse extends BaseResponseTemplate {
    public DisconnectResponse(String msg, int status) { super(msg, status); }
}

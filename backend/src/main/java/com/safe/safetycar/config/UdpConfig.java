package com.safe.safetycar.config;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import com.safe.safetycar.streaming.udp.filter.UDPFilter;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.core.task.TaskExecutor;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.config.EnableIntegration;
import org.springframework.integration.dsl.IntegrationFlow;
import org.springframework.integration.filter.MessageFilter;
import org.springframework.integration.ip.dsl.Udp;
import org.springframework.integration.ip.udp.UnicastReceivingChannelAdapter;
import org.springframework.messaging.MessageChannel;
import org.springframework.scheduling.concurrent.ThreadPoolTaskExecutor;

import java.util.concurrent.ThreadPoolExecutor;

@Configuration
@org.springframework.integration.config.EnableIntegration
public class UdpConfig {

    @Value("${udp.channel}")
    private String channel;
    @Value("${udp.port}")
    private Integer port;

    @Value("${udp.task-executor.core-pool-size}")
    private Integer corePoolSize;
    @Value("${udp.task-executor.max-pool-size}")
    private Integer maxPoolSize;
    @Value("${udp.task-executor.queue-capacity}")
    private Integer queueSize;

    private LogManager logManager = new LogManager(UdpConfig.class);

    @Bean
    public MessageChannel inboundChannel() {
        return new DirectChannel();
    }

//    @Bean
//    public MessageFilter registerFilter() {
//        logManager.sendLog("registerFilter", LogManager.LOG_TYPE.INFO);
//
//         return new MessageFilter(new UDPFilter());
//    }

    @Bean(name = "udpReceivingAdapter")
    public UnicastReceivingChannelAdapter udpReceivingAdapter() {

        UnicastReceivingChannelAdapter adapter = new UnicastReceivingChannelAdapter(port);
        adapter.setOutputChannel(inboundChannel());
        adapter.setTaskExecutor(getTaskExecutor());
        adapter.setOutputChannelName(channel);
        return adapter;
    }

    // this task executor will define how many concurrent connection UDP can handle
    TaskExecutor getTaskExecutor() {

        ThreadPoolTaskExecutor ioExec = new ThreadPoolTaskExecutor();
        ioExec.setCorePoolSize(corePoolSize);
        ioExec.setMaxPoolSize(maxPoolSize);
        ioExec.setQueueCapacity(queueSize);
        ioExec.setThreadNamePrefix("io-");
        ioExec.setRejectedExecutionHandler(new ThreadPoolExecutor.AbortPolicy());
        ioExec.initialize();
        return ioExec;
    }

}

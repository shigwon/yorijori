//    package com.linky.config.webRTC;
//
//    import com.linky.api.webRTC.handler.WebRtcSignalHandler;
//    import org.springframework.context.annotation.Bean;
//    import org.springframework.context.annotation.Configuration;
//    import org.springframework.web.socket.config.annotation.EnableWebSocket;
//    import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
//    import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;
//
//    @Configuration
//    @EnableWebSocket
//    public class webRtcConfig implements WebSocketConfigurer {
//
//        @Bean
//        public WebRtcSignalHandler webRtcSignalHandler() {
//            return new WebRtcSignalHandler();
//        }
//
//        @Override
//        public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
//            registry.addHandler(webRtcSignalHandler(), "/signal")
//                    .setAllowedOrigins("*");
//        }
//
//
//    }

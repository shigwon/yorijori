package com.linky.api.k8s;

import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@Slf4j
public class K8sController {

    @GetMapping("/startup")
    public ResponseEntity<Object> startup() {
        log.info("Startup Application");
        return ResponseEntity.ok().build();
    }

    @GetMapping("/readiness")
    public ResponseEntity<Object> readiness() {
        log.info("Readiness Application");
        return ResponseEntity.ok().build();
    }
    @GetMapping("/liveness")
    public ResponseEntity<Object> liveness() {
        log.info("Liveness Application");
        return ResponseEntity.ok().build();
    }
}

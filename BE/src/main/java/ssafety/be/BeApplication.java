package ssafety.be;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.ConfigurableApplicationContext;
import org.springframework.context.annotation.ComponentScan;
import ssafety.be.service.KakaoMapService;

@SpringBootApplication
public class BeApplication {
    public static void main(String[] args) {
        SpringApplication.run(BeApplication.class, args);
    }
}


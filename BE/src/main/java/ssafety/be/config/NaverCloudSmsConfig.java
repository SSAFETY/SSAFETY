package ssafety.be.config;

/*
NaverCloudSmsConfig 클래스는 Naver Cloud SMS API의 설정 정보를 담는 객체입니다.
이 클래스는 `@Configuration` 어노테이션을 사용하여 Spring의 빈으로 등록되며,
`@ConfigurationProperties` 어노테이션을 이용하여 `application.yml` 파일에 정의된 `naver-cloud-sms` 접두사로 시작하는 프로퍼티들을 매핑합니다.

클래스 멤버 변수들은 Naver Cloud SMS API의 접근 키, 비밀 키, 서비스 ID, 그리고 SMS를 발송하는 기기의 전화번호를 저장합니다.
Getter, Setter, EqualsAndHashCode 등의 메서드는 Lombok의 `@Data` 어노테이션을 사용하여 자동으로 생성됩니다.
*/

import lombok.Data;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.context.annotation.Configuration;

@Configuration
@ConfigurationProperties(prefix = "naver-cloud-sms")
@Data
public class NaverCloudSmsConfig {

    // Naver Cloud SMS API의 접근 키(access key)로 인증에 사용됩니다.
    private String accesskey;

    // Naver Cloud SMS API의 비밀 키(secret key)로 인증에 사용됩니다.
    private String secretkey;

    // Naver Cloud SMS API의 서비스 ID(service ID)로 SMS를 발송할 때 사용됩니다.
    private String serviceId;

    // Naver Cloud SMS API를 통해 SMS를 발송하는 기기의 전화번호로, 발신자 번호로 사용됩니다.
    private String phone;
}


/**
 * AWS S3에 연결하기 위한 AmazonS3 클라이언트 빈을 생성하고, 필요한 인증 정보와 지역 정보를 주입합니다.
 * 이 클래스는 AWS S3 연동을 위한 기본적인 설정을 제공하며, AWS S3에 접근하여 파일을 업로드하고 다운로드하는데 사용됩니다.
 */

package ssafety.be.config;

import com.amazonaws.auth.AWSCredentials;
import com.amazonaws.auth.AWSStaticCredentialsProvider;
import com.amazonaws.auth.BasicAWSCredentials;
import com.amazonaws.regions.Regions;
import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.AmazonS3ClientBuilder;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class S3Config {
    @Value("${cloud.aws.credentials.accessKey}")
    private String accessKey;

    @Value("${cloud.aws.credentials.secretKey}")
    private String secretKey;

    @Value("${cloud.aws.region.static}")
    private String region;

    /**
     * AmazonS3 빈을 생성하여 반환합니다.
     * AmazonS3 클라이언트는 AWS S3 서비스와의 연결을 위한 핵심 클래스입니다.
     * 인증 정보와 지역 정보를 설정하여 AmazonS3 클라이언트를 생성하고, 이를 Spring의 빈으로 등록합니다.
     *
     * @return AmazonS3 클라이언트 객체
     */
    @Bean
    public AmazonS3 amazonS3() {
        AWSCredentials awsCredentials = new BasicAWSCredentials(accessKey, secretKey);

        return AmazonS3ClientBuilder.standard()
                .withRegion(Regions.fromName(region))
                .withCredentials(new AWSStaticCredentialsProvider(awsCredentials))
                .build();
    }
}

package ssafety.be.service;

/**
 * SmsService 클래스는 SMS 발송과 관련된 비즈니스 로직을 처리하는 서비스입니다.
 * 사용자의 전화번호로 인증번호를 발송하고, 사용자가 입력한 인증번호를 확인하는 기능을 제공합니다.
 * 주요 기능:
 * - 인증번호 발송: 주어진 SmsRequestDto에 포함된 전화번호로 6자리 난수 인증번호를 발송합니다.
 *   발송 결과는 ApiResponseDto<SmsResponseDto> 형태로 반환됩니다.
 * - 사용자 입력 인증번호 확인: 사용자가 입력한 인증번호를 받아 해당 전화번호로 발송된 인증번호와 비교하여 확인합니다.
 *   확인 결과는 ApiResponseDto<SmsResponseDto> 형태로 반환됩니다.
 * - 서명 생성: API 요청에 필요한 서명 값을 생성합니다.
 * - 6자리 난수 인증번호 생성: 6자리의 난수 인증번호를 생성합니다.
 * 필드 설명:
 * - config: NaverCloudSmsConfig 클래스의 인스턴스로, 네이버 클라우드 SMS API 설정 정보를 제공합니다.
 * - verificationCodeRepository: VerificationCode 엔티티와 관련된 데이터베이스 작업을 처리하는 리포지토리 클래스입니다.
 * - SMS_API_URL: 네이버 클라우드 SMS API의 URL 주소입니다.
 */

import java.io.UnsupportedEncodingException;
import java.security.InvalidKeyException;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;

import org.apache.tomcat.util.codec.binary.Base64;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ssafety.be.config.NaverCloudSmsConfig;
import ssafety.be.dto.SmsRequestDto;

@Service
@Slf4j
@RequiredArgsConstructor
public class SmsService {

    private final NaverCloudSmsConfig config;

    private static final String SMS_API_URL = "https://sens.apigw.ntruss.com/sms/v2/services/{serviceId}/messages";

    /**
     * 주어진 SmsRequestDto에 포함된 전화번호로 인증번호를 발송합니다.
     * @param request SMS 발송 요청 정보를 담고 있는 SmsRequestDto 객체
     * @return 인증번호 발송 결과를 ApiResponseDto로 포장하여 반환합니다.
     */
    public String sendSms(SmsRequestDto request) {
        // 요청 전화번호
        String phoneNumber = request.getPhoneNumber();

        // 현재시간
        long currentTime = System.currentTimeMillis();

        // 헤더 세팅
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        headers.set("x-ncp-apigw-timestamp", String.valueOf(currentTime));
        headers.set("x-ncp-iam-access-key", config.getAccesskey());
        try {
            // 서명 생성
            headers.set("x-ncp-apigw-signature-v2", getSignature(String.valueOf(currentTime)));
        } catch (Exception e) {
            throw new RuntimeException("서명 생성에 실패했습니다: " + e.getMessage());
        }

        // SMS 메시지 내용 생성
        Map<String, String> messageContent = new HashMap<>();
        messageContent.put("to", phoneNumber);
        messageContent.put("content", phoneNumber + "번 님의 신고가 정상적으로 처리되었습니다.");

        List<Map<String, String>> messages = new ArrayList<>();
        messages.add(messageContent);

        // SMS 발송 API 요청에 필요한 데이터 구성
        Map<String, Object> body = new HashMap<>();
        body.put("type", "SMS");
        body.put("contentType", "COMM");
        body.put("countryCode", "82");
        body.put("from", config.getPhone());
        body.put("content", phoneNumber + "님의 신고가 정상적으로 처리되었습니다.");
        body.put("messages", messages);

        HttpEntity<Map<String, Object>> entity = new HttpEntity<>(body, headers);

        // SMS 발송 API 요청
        RestTemplate restTemplate = new RestTemplate();
        ResponseEntity<String> response = restTemplate.postForEntity(SMS_API_URL, entity, String.class, config.getServiceId());

        return "메시지 전송 완료";
    }

    /**
     * 6자리의 난수 인증번호를 생성하고 반환합니다.
     * @return 생성된 6자리 난수 인증번호
     */
    public String getSignature(String time) throws NoSuchAlgorithmException, UnsupportedEncodingException, InvalidKeyException {
        String space = " ";
        String newLine = "\n";
        String method = "POST";
        String url = "/sms/v2/services/" + config.getServiceId() + "/messages";

        String message = new StringBuilder()
                .append(method)
                .append(space)
                .append(url)
                .append(newLine)
                .append(time)
                .append(newLine)
                .append(config.getAccesskey())
                .toString();

        SecretKeySpec signingKey = new SecretKeySpec(config.getSecretkey().getBytes("UTF-8"), "HmacSHA256");
        Mac mac = Mac.getInstance("HmacSHA256");
        mac.init(signingKey);

        byte[] rawHmac = mac.doFinal(message.getBytes("UTF-8"));
        return Base64.encodeBase64String(rawHmac);
    }
}


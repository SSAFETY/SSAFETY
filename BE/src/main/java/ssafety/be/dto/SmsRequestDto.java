package ssafety.be.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class SmsRequestDto {
    /**
     * SMS를 발송할 전화번호를 나타내는 필드입니다.
     * SMS 발송 요청 시 해당 전화번호가 여기에 저장됩니다.
     */
    private String phoneNumber;
}

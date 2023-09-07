package ssafety.be.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.util.Date;

@Data
@AllArgsConstructor
@Builder
public class SearchDto {
    private String city;
    private String gu;
    private String aiResult;
    private Date date;
}

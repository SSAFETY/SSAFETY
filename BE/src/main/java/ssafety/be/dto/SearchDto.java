package ssafety.be.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@Builder
@NoArgsConstructor
public class SearchDto {
    private String state;
    private String depth3;
    private String aiResult;
    private String date;
}

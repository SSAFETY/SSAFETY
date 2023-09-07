package ssafety.be.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Builder
public class ReportDto {
    private LocalDateTime creationTime;
    private String videoUrl;
    private double gpsLatitude;
    private double gpsLongitude;
    private String aiResult;
    private String vehicleNumber;
}

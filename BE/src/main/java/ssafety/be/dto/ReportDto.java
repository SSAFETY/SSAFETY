package ssafety.be.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@Builder
@NoArgsConstructor
public class ReportDto {
    private String videoUrl;
    private double gpsLatitude;
    private double gpsLongitude;
    private String aiResult;
    private String vehicleNumber;

    // Setter 메서드들
    public void setVideoUrl(String videoUrl) {
        this.videoUrl = videoUrl;
    }

    public void setGpsLatitude(double gpsLatitude) {
        this.gpsLatitude = gpsLatitude;
    }

    public void setGpsLongitude(double gpsLongitude) {
        this.gpsLongitude = gpsLongitude;
    }

    public void setAiResult(String aiResult) {
        this.aiResult = aiResult;
    }

    public void setVehicleNumber(String vehicleNumber) {
        this.vehicleNumber = vehicleNumber;
    }
}
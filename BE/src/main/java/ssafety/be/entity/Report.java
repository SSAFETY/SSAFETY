package ssafety.be.entity;

import com.fasterxml.jackson.annotation.JsonFormat;
import jakarta.persistence.*;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.hibernate.annotations.CreationTimestamp;

import java.time.LocalDateTime;

@Entity
@Table
@Data
@NoArgsConstructor
/*
 * `Report` 엔티티 클래스는 교통 데이터 보고서를 나타냅니다.
 * 이 엔티티는 생성 시간, 촬영 영상 URL, GPS 정보, AI 판단 결과, 위반 차량 번호를 포함합니다.
 */
public class Report {
    /**
     * 보고서의 주요 식별자(primary key)로 데이터베이스에서 자동으로 생성됩니다.
     */
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    /**
     * 생성 시간은 보고서가 생성된 일시를 나타냅니다.
     */
    @CreationTimestamp
    @Column(nullable = false)
    private LocalDateTime creationTime = LocalDateTime.now();

    /**
     * 촬영 영상의 URL은 보고서에 연결된 영상 파일의 경로를 나타냅니다.
     */
    private String videoUrl;

    /**
     * GPS 위도 정보는 촬영 위치의 위도를 나타냅니다.
     */
    private double gpsLatitude;

    /**
     * GPS 경도 정보는 촬영 위치의 경도를 나타냅니다.
     */
    private double gpsLongitude;

    /**
     * GPS경도를 통해 얻은 주소를 나타냅니다.
     */
    private String gpsLocation;

    @Column
    private String state;

    /**
     * 도시 정보를 나타냅니다.
     */
    @Column(nullable = false)
    private String city;

    /**
     * 구 정보를 나타냅니다.
     */
    @Column(nullable = false)
    private String depth3;

    /**
     * 동 정보를 나타냅니다.
     */
    private String depth4;

    /**
     * 상세 주소 정보를 나타냅니다.
     */
    private String detail;

    /**
     * AI 판단 결과는 촬영된 데이터에 대한 인공지능의 분석 결과를 나타냅니다.
     */
    @Column(nullable = false)
    private String aiResult;

    /**
     * 위반 차량 번호는 보고서에 기록된 차량의 번호판을 나타냅니다.
     */
    private String vehicleNumber;

    @Column(nullable = false)
    private String phoneNum = "01095212415";
}

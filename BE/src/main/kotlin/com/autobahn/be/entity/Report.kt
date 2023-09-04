package com.autobahn.be.entity

import jakarta.persistence.*
import java.time.LocalDateTime

/**
 * `Report` 엔티티 클래스는 교통 데이터 보고서를 나타냅니다.
 * 이 엔티티는 생성 시간, 촬영 영상 URL, GPS 정보, AI 판단 결과, 위반 차량 번호를 포함합니다.
 */
@Entity
class Report(
    /**
     * 생성 시간은 보고서가 생성된 일시를 나타냅니다.
     */
    @Column(name = "creation_time", nullable = false)
    var creationTime: LocalDateTime,

    /**
     * 촬영 영상의 URL은 보고서에 연결된 영상 파일의 경로를 나타냅니다.
     */
    @Column(name = "video_url", nullable = false)
    var videoUrl: String,

    /**
     * GPS 위도 정보는 촬영 위치의 위도를 나타냅니다.
     */
    @Column(name = "gps_latitude", nullable = false)
    var gpsLatitude: Double,

    /**
     * GPS 경도 정보는 촬영 위치의 경도를 나타냅니다.
     */
    @Column(name = "gps_longitude", nullable = false)
    var gpsLongitude: Double,

    /**
     * AI 판단 결과는 촬영된 데이터에 대한 인공지능의 분석 결과를 나타냅니다.
     */
    @Column(name = "ai_result", nullable = false)
    var aiResult: String,

    /**
     * 위반 차량 번호는 보고서에 기록된 차량의 번호판을 나타냅니다.
     */
    @Column(name = "vehicle_number", nullable = false)
    var vehicleNumber: String
) {
    /**
     * 보고서의 주요 식별자(primary key)로 데이터베이스에서 자동으로 생성됩니다.
     */
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    var id: Long? = null
}

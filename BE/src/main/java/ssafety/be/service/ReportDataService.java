package ssafety.be.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import ssafety.be.dto.ReportDto;
import ssafety.be.entity.Report;
import ssafety.be.repository.ReportRepository;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.ZoneId;
import java.util.Date;
import java.util.List;

/**
 * ReportDataService는 교통 데이터 보고서 관련 서비스를 제공합니다.
 */
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class ReportDataService {
    private final ReportRepository reportRepository;
    private final KakaoMapService kakaoMapService;

    /**
     * ReportDto를 사용하여 Report 엔티티를 생성하고 저장합니다.
     *
     * @param reportDto ReportDto 객체
     */
    @Transactional
    public void saveReport(ReportDto reportDto) throws Exception {
        try {
            // ReportDto에서 필요한 정보를 추출하여 Report 엔티티를 생성합니다.
            LocalDateTime creationTime = reportDto.getCreationTime();
            String videoUrl = reportDto.getVideoUrl();
            double gpsLatitude = reportDto.getGpsLatitude();
            double gpsLongitude = reportDto.getGpsLongitude();
            String aiResult = reportDto.getAiResult();
            String vehicleNumber = reportDto.getVehicleNumber();

            // GPS 좌표를 사용하여 주소를 조회합니다.
            String[] address = kakaoMapService.getAddress(String.valueOf(gpsLatitude), String.valueOf(gpsLongitude));

            // 주소 정보를 이용하여 Report 엔티티를 생성합니다.
            Report report = new Report();
            report.setCreationTime(creationTime);
            report.setVideoUrl(videoUrl);
            report.setGpsLatitude(gpsLatitude);
            report.setGpsLongitude(gpsLongitude);
            report.setGpsLocation(String.join(" ", address)); // 주소 정보 합치기
            if (!address[0].endsWith("울")) {
                report.setState(address[0]); // 도
                report.setCity(address[1]); // 시
                report.setDepth3(address[2]); // 면
                report.setDepth4(address[3]); // 로
                report.setDetail(address[4]); // detail
            } else {
                report.setCity(address[0]); // 시
                report.setDepth3(address[1]); // 구
                report.setDepth4(address[2]); // 동
                report.setDetail(address[3]); // detail
            }
            report.setAiResult(aiResult);
            report.setVehicleNumber(vehicleNumber);

            // Report 엔티티를 저장합니다.
            reportRepository.save(report);
        } catch (Exception e) {
            e.printStackTrace();
            throw new Exception("신고를 저장하는 중에 오류가 발생했습니다.");
        }
    }

    /**
     * 지정된 조건에 따라 교통 데이터 보고서를 검색합니다.
     *
     * @param state    지역 대분류
     * @param depth3   지역 소분류
     * @param aiResult AI 판단 결과
     * @param date     날짜
     * @return 검색된 보고서 목록
     */
    public List<Report> findReportsByConditions(String state, String depth3, String aiResult, Date date) {
        LocalDateTime startDateTime = convertToLocalDateTime(date).with(LocalTime.MIN);
        LocalDateTime endDateTime = convertToLocalDateTime(date).with(LocalTime.MAX);

        return reportRepository.findBycityAndDepth3AndAiResultAndCreationTimeBetween(state, depth3, aiResult, startDateTime, endDateTime);
    }

    /**
     * Date 객체를 LocalDateTime 객체로 변환합니다.
     *
     * @param date 변환할 Date 객체
     * @return LocalDateTime 객체
     */
    private LocalDateTime convertToLocalDateTime(Date date) {
        return date.toInstant().atZone(ZoneId.systemDefault()).toLocalDateTime();
    }
}
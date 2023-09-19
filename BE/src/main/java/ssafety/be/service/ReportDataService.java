package ssafety.be.service;

import lombok.RequiredArgsConstructor;
import org.springframework.data.jpa.domain.Specification;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import ssafety.be.dto.ReportDto;
import ssafety.be.entity.Report;
import ssafety.be.repository.ReportRepository;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
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
    private final ReportSpecification reportSpecification;

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
     * @param city    지역 대분류
     * @param depth3   지역 소분류
     * @param aiResult AI 판단 결과
     * @param dateStr     날짜
     * @return 검색된 보고서 목록
     */
    public List<Report> findReportsByConditions(String city, String depth3, String aiResult, String dateStr) {
        System.out.println("도시 : " + city + " 구 : " + depth3 + " ai결과 : " + aiResult + " 날짜 : " + dateStr);
        LocalDateTime startTime = null;
        LocalDateTime endTime = null;

        if (dateStr != null && !dateStr.isEmpty()) {
            // "2023-09-18" 형식의 문자열을 LocalDate로 변환
            DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd");
            LocalDate date = LocalDate.parse(dateStr, formatter);

            // 시작 시간 설정 (자정)
            startTime = date.atStartOfDay();

            // 종료 시간 설정 (23:59:59)
            endTime = date.atTime(23, 59, 59);

            System.out.println("시작 시간: " + startTime);
            System.out.println("종료 시간: " + endTime);
        }

        Specification<Report> spec = reportSpecification.findByConditions(city, depth3, aiResult, startTime, endTime);
        System.out.println(spec);
        return reportRepository.findAll(spec);
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
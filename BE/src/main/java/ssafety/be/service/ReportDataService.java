package ssafety.be.service;

import lombok.RequiredArgsConstructor;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
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
import java.util.Optional;

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
    public void saveReport(ReportDto reportDto) {
        try {
            // 필수 필드 유효성 검사
            validateReportDto(reportDto);

            // GPS 좌표를 사용하여 주소를 조회합니다.
            String[] address = kakaoMapService.getAddress(String.valueOf(reportDto.getGpsLatitude()), String.valueOf(reportDto.getGpsLongitude()));

            // 주소 정보를 이용하여 Report 엔티티를 생성합니다.
            Report report = new Report();
            report.setVideoUrl(reportDto.getVideoUrl());
            report.setGpsLatitude(reportDto.getGpsLatitude());
            report.setGpsLongitude(reportDto.getGpsLongitude());
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
            report.setAiResult(reportDto.getAiResult());
            report.setVehicleNumber(reportDto.getVehicleNumber());

            // Report 엔티티를 저장합니다.
            reportRepository.save(report);
        } catch (Exception e) {
            // 예외 처리
            e.printStackTrace();
            throw new IllegalArgumentException("신고를 저장하는 중에 오류가 발생했습니다.", e);
        }
    }

    private void validateReportDto(ReportDto reportDto) throws IllegalArgumentException {
        if (reportDto.getVideoUrl() == null || reportDto.getVideoUrl().isEmpty()) {
            throw new IllegalArgumentException("videoUrl은 필수 입력 항목입니다.");
        }
        if (reportDto.getAiResult() == null || reportDto.getAiResult().isEmpty()) {
            throw new IllegalArgumentException("aiResult는 필수 입력 항목입니다.");
        }
        if (reportDto.getVehicleNumber() == null || reportDto.getVehicleNumber().isEmpty()) {
            throw new IllegalArgumentException("vehicleNumber는 필수 입력 항목입니다.");
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
    public Page<Report> findReportsByConditions(String city, String depth3, String aiResult, String dateStr , Pageable pageable) {
        System.out.println("도시 : " + city + " 구 : " + depth3 + " ai결과 : " + aiResult + " 날짜 : " + dateStr);
        LocalDateTime startTime = null;
        LocalDateTime endTime = null;

        if (dateStr != null && !dateStr.isEmpty()) {
            DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd");
            LocalDate date = LocalDate.parse(dateStr, formatter);

            // 시작 시간 설정 (자정)
            startTime = date.atStartOfDay();

            // 종료 시간 설정 (23:59:59)
            endTime = date.atTime(23, 59, 59);
        }

        Specification<Report> spec = reportSpecification.findByConditions(city, depth3, aiResult, startTime, endTime);
        return reportRepository.findAll(spec, pageable);
    }

    public Page<Report> getAll(Pageable pageable) {
        Page<Report> reportList = reportRepository.getAll(pageable);
        // 정렬 조건이 추가된 Pageable 객체를 사용하여 데이터를 조회합니다.
        System.out.println(pageable);
        return reportList;
    }

    public Optional<Report> findById(Long id) {
        return reportRepository.findById(id);
    }

    public List<Report> getData() {
        return reportRepository.findAll();
    }


    /**
     * Date 객체를 LocalDateTime 객체로 변환합니다.
     *
     * @param date 변환할 Date 객체
     * @return LocalDateTime 객체
     */
    private LocalDateTime convertToLocalDateTime(Date date) {
        return date.toInstant().atZone(ZoneId.systemDefault()).toLocalDateTime();
    }}
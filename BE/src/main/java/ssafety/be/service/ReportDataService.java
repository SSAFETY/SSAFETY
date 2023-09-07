package ssafety.be.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
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
    private final S3Service s3Service;

    /**
     * 지정된 조건에 따라 교통 데이터 보고서를 검색합니다.
     *
     * @param city     도시 이름
     * @param gu       구 이름
     * @param aiResult AI 판단 결과
     * @param date     날짜
     * @return 검색된 보고서 목록
     */
    public List<Report> findReportsByConditions(String city, String gu, String aiResult, Date date) {
        LocalDateTime startDateTime = convertToLocalDateTime(date).with(LocalTime.MIN);
        LocalDateTime endDateTime = convertToLocalDateTime(date).with(LocalTime.MAX);

        return reportRepository.findByCityAndGuAndAiResultAndCreationTimeBetween(city, gu, aiResult, startDateTime, endDateTime);
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

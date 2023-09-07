package ssafety.be.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import ssafety.be.entity.Report;
import java.time.LocalDateTime;
import java.util.List;

public interface ReportRepository extends JpaRepository<Report, Long> {
    
    //조건에 따른 값들 불러오기
    //조건이 없으면 전체 데이터 불러옴
    List<Report> findByCityAndGuAndAiResultAndCreationTimeBetween(String city, String gu, String aiResult, LocalDateTime startDateTime, LocalDateTime endDateTime);
}


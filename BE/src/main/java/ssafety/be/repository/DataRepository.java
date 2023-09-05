package ssafety.be.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import ssafety.be.entity.Data;

import java.time.LocalDateTime;
import java.util.List;

@Repository
public interface DataRepository extends JpaRepository<Data, Long> {

    // 검색어를 포함한 주소로 데이터 검색
    List<Data> findByGpsLocationContainingIgnoreCase(String keyword);

    // 특정 날짜 범위 내의 데이터 검색
    List<Data> findByCreationTimeBetween(LocalDateTime startDate, LocalDateTime endDate);

    // 특정 AI 판단 결과로 데이터 검색
    List<Data> findByAiResult(String aiResult);
}

package ssafety.be.service;

import jakarta.persistence.criteria.Predicate;
import org.springframework.data.jpa.domain.Specification;
import org.springframework.stereotype.Service;
import org.springframework.util.StringUtils;
import ssafety.be.entity.Report;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Service
public class ReportSpecification {

    public static Specification<Report> findByConditions(
            String city, String gu, String aiResult, LocalDateTime startTime, LocalDateTime endTime) {
        return (root, query, criteriaBuilder) -> {
            List<Predicate> predicates = new ArrayList<>();

            // 조건 추가: 도시(city)가 null 또는 빈 문자열이 아니면 도시 조건을 추가
            if (!StringUtils.isEmpty(city)) {
                predicates.add(criteriaBuilder.equal(criteriaBuilder.lower(root.get("city")), city.toLowerCase()));
            }

            // 조건 추가: 구(gu)가 null 또는 빈 문자열이 아니면 구 조건을 추가
            if (!StringUtils.isEmpty(gu)) {
                predicates.add(criteriaBuilder.equal(criteriaBuilder.lower(root.get("depth3")), gu.toLowerCase()));
            }

            // 조건 추가: AI 결과(aiResult)가 null 또는 빈 문자열이 아니면 AI 결과 조건을 추가
            if (!StringUtils.isEmpty(aiResult)) {
                predicates.add(criteriaBuilder.equal(criteriaBuilder.lower(root.get("aiResult")), aiResult.toLowerCase()));
            }

            // 조건 추가: 시작 시간(startTime)과 종료 시간(endTime)이 null이 아니면 시간 범위 조건을 추가
            if (startTime != null && endTime != null) {
                predicates.add(criteriaBuilder.between(root.get("creationTime"), startTime, endTime));
            }

            return criteriaBuilder.and(predicates.toArray(new Predicate[0]));
        };
    }
}


package ssafety.be.repository;

import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.domain.Specification;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.JpaSpecificationExecutor;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;
import ssafety.be.entity.Report;

import java.util.List;
import java.util.Optional;

@Repository
public interface ReportRepository extends JpaRepository<Report, Long>, JpaSpecificationExecutor<Report> {

    List<Report> findAll(Specification<Report> spec);

//    Page<Report> findAll(Specification<Report> spec, Pageable pageable);

    @Query(value = "SELECT a FROM Report a ORDER BY a.id DESC")
    Page<Report> getAll(Pageable pageable);

    Optional<Report> findById(Long id);
}
package ssafety.be.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import ssafety.be.repository.ReportRepository;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class ReportDataService {
    private final ReportRepository reportRepository;
    private final S3Service s3Service;

}

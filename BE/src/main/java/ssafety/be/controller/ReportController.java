package ssafety.be.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.data.web.PageableDefault;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import ssafety.be.dto.ReportDto;
import ssafety.be.dto.SearchDto;
import ssafety.be.dto.SmsRequestDto;
import ssafety.be.entity.Report;
import ssafety.be.service.KakaoMapService;
import ssafety.be.service.ReportDataService;
import ssafety.be.service.SmsService;

import java.util.List;
import java.util.Optional;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api")
public class ReportController {
    private final KakaoMapService kakaoMapService;
    private final ReportDataService reportDataService;
    private final SmsService smsService;

    @PostMapping("/report")
    public ResponseEntity<String> getDataFromRos(@RequestBody ReportDto data) {
        try {
            // saveReport 메서드를 호출하여 Report 엔티티를 생성하고 저장합니다.
            reportDataService.saveReport(data);
            // 처리가 완료되면 응답을 반환합니다.
            return new ResponseEntity<>("신고가 성공적으로 처리되었습니다.", HttpStatus.OK);
        } catch (Exception e) {
            e.printStackTrace();
            return new ResponseEntity<>("요청 처리 중 오류가 발생했습니다.", HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    @GetMapping("/getAll")
    public Page<Report> getAllReports(@PageableDefault(size = 10) Pageable pageable) {
        // 페이지 번호 추출
        int pageNumber = pageable.getPageNumber();

        // 새로운 Pageable 객체 생성 (페이지 번호만 변경)
        Pageable newPageable = PageRequest.of(pageNumber, pageable.getPageSize(), Sort.by(Sort.Order.desc("id")));

        // 데이터를 조회할 때 새로운 Pageable 객체 사용
        return reportDataService.getAll(newPageable);
    }

    @GetMapping("/getData")
    public List<Report> getData() {
        return reportDataService.getData();
    }

    @GetMapping("/detailData")
    public Optional<Report> detailData(@RequestParam Long id) {
        return reportDataService.findById(id);
    }

    // 컨트롤러 메서드에서 Pageable 매개변수 제거
    @PostMapping("/searchReports")
    public List<Report> searchReports(@RequestBody SearchDto request) {
        return reportDataService.findReportsByConditions(request.getCity(), request.getDepth3(), request.getAiResult(), request.getDate());
    }

    @GetMapping("/getAddress")
    public String getAddress(@RequestParam String latitude, @RequestParam String longitude) {
        return kakaoMapService.getAddress(latitude, longitude);
    }

    @PostMapping("/success")
    public String sendSuccess(@RequestBody SmsRequestDto request) {
        return smsService.sendSuccess(request);
    }

    @PostMapping("/fail")
    public String sendFail(@RequestBody SmsRequestDto request) {
        return smsService.sendFail(request);
    }
}
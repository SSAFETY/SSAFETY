package ssafety.be.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import ssafety.be.dto.ReportDto;
import ssafety.be.dto.SearchDto;
import ssafety.be.entity.Report;
import ssafety.be.service.KakaoMapService;
import ssafety.be.service.ReportDataService;

import java.util.List;

@RestController
@RequiredArgsConstructor
public class ReportController {
    private final KakaoMapService kakaoMapService;
    private final ReportDataService reportDataService;

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

    @PostMapping("/searchReports")
    public List<Report> searchReports(@RequestBody SearchDto request) {
        return reportDataService.findReportsByConditions(request.getState(), request.getDepth3(), request.getAiResult(), request.getDate());
    }

    @GetMapping("/getAddress")
    public String[] getAddress(@RequestParam String latitude, @RequestParam String longitude) {
        return kakaoMapService.getAddress(latitude, longitude);
    }
}

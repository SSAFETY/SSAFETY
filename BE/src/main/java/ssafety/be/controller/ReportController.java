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

    @PostMapping("/data")
    public ResponseEntity<String> getDataFromRos(@RequestBody ReportDto data) {
        try {
            // JSON 데이터를 처리하는 로직을 여기에 작성합니다.
            // request 객체를 사용하여 JSON 데이터에 접근할 수 있습니다.

            // 예를 들어, JSON 데이터에서 필요한 정보를 추출하거나 데이터베이스에 저장하는 등의 작업을 수행할 수 있습니다.

            // 처리가 완료되면 응답을 반환합니다.
            return new ResponseEntity<>("요청이 성공적으로 처리되었습니다.", HttpStatus.OK);
        } catch (Exception e) {
            e.printStackTrace();
            return new ResponseEntity<>("요청 처리 중 오류가 발생했습니다.", HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    @PostMapping("/searchReports")
    public List<Report> searchReports(@RequestBody SearchDto request) {
        // DTO 객체로 매핑된 요청을 이용하여 검색을 수행합니다.
        return reportDataService.findReportsByConditions(request.getCity(), request.getGu(), request.getAiResult(), request.getDate());
    }

    @GetMapping("/getAddress")
    public String[] getAddress(@RequestParam String latitude, @RequestParam String longitude) {
        String result[] = kakaoMapService.getAddress(latitude, longitude);
        return result;
    }
}

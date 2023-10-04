package ssafety.be.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import ssafety.be.service.CctvService;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/cctv")
public class CctvController {

    private final CctvService cctvService;

    @GetMapping("/data")
    public String getCCTVData() {
        return cctvService.fetchCCTVData();
    }
}

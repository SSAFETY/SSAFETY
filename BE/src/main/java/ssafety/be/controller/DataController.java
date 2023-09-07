package ssafety.be.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import ssafety.be.service.KakaoMapService;

@RestController
@RequiredArgsConstructor
public class DataController {
    private final KakaoMapService kakaoMapService;

    @GetMapping("/getAddress")
    public String[] getAddress(@RequestParam String latitude, @RequestParam String longitude) {
        String result[] = kakaoMapService.getAddress(latitude, longitude);
        return result;
    }
}

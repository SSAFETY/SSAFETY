package ssafety.be.service;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

@Service
public class CctvService {

    @Value("${cctv.apiKey}")
    private String apiKey;

    public String fetchCCTVData() {
        try {
            // Open API 호출 URL
            String apiUrl = "https://openapi.its.go.kr:9443/cctvInfo" +
                    "?apiKey=" + apiKey +
                    "&type=ex" +
                    "&cctvType=1" +
                    "&minX=127.100000" +
                    "&maxX=128.890000" +
                    "&minY=34.100000" +
                    "&maxY=39.100000" +
                    "&getType=json";

            RestTemplate restTemplate = new RestTemplate();
            return restTemplate.getForObject(apiUrl, String.class);
        } catch (Exception e) {
            e.printStackTrace();
            return "Failed to fetch CCTV data.";
        }
    }
}


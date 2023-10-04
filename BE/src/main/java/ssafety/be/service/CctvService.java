package ssafety.be.service;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

@Service
public class CctvService {

    @Value("0db19b82b4d84ef3846c147df0b0028d")
    private String apiKey;

    public String fetchCCTVData() {
        try {
            // Open API 호출 URL
            String apiUrl = "https://openapi.its.go.kr:9443/cctvInfo" +
                    "?apiKey=" + apiKey +
                    "&type=ex" +
                    "&cctvType=1" +
                    "&minX=126.734086" +
                    "&maxX=127.269311" +
                    "&minY=37.413294" +
                    "&maxY=37.715133" +
                    "&getType=json";

            RestTemplate restTemplate = new RestTemplate();
            return restTemplate.getForObject(apiUrl, String.class);
        } catch (Exception e) {
            e.printStackTrace();
            return "Failed to fetch CCTV data.";
        }
    }
}
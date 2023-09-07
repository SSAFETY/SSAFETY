package ssafety.be.service;

import org.springframework.beans.factory.annotation.Value;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.HttpClientErrorException;
import org.springframework.web.client.RestTemplate;

import org.springframework.util.LinkedMultiValueMap;
import org.springframework.util.MultiValueMap;

/**
 * 카카오맵 API를 사용하여 경위도 좌표를 주소로 변환하는 서비스 클래스입니다.
 */
@Service
public class KakaoMapService {

    @Value("${kakao.api.key}")
    private String apiKey;

    /**
     * 주어진 경위도 좌표를 사용하여 주소를 조회하고, 주소를 배열로 반환합니다.
     *
     * @param lati 경도
     * @param longi 위도
     * @return 주소 정보를 담은 배열 또는 오류 메시지를 담은 배열
     */
    public String[] getAddress(String lati, String longi) {
        try {
            final String API_URL = "https://dapi.kakao.com/v2/local/geo/coord2address.json?x=" + longi + "&y=" + lati + "&input_coord=WGS84";

            HttpHeaders headers = new HttpHeaders();
            headers.add("Authorization", "KakaoAK " + apiKey);

            MultiValueMap<String, String> parameters = new LinkedMultiValueMap<String, String>();
            parameters.add("x", longi);
            parameters.add("y", lati);
            parameters.add("input_coord", "WGS84");

            RestTemplate restTemplate = new RestTemplate();
            ResponseEntity<String> result = restTemplate.exchange(API_URL, HttpMethod.GET, new HttpEntity(headers), String.class);

            if (result.getStatusCode().is4xxClientError()) {
                String errorMessage = result.getBody();
                System.err.println("Kakao API 요청 오류: " + errorMessage);
                return new String[]{"error"};
            }

            JsonParser jsonParser = new JsonParser();
            JsonObject jsonObject = (JsonObject) jsonParser.parse(result.getBody());

            JsonArray jsonArray = (JsonArray) jsonObject.get("documents");
            if(jsonArray == null) {
                return new String[]{"잘못된 좌표입니다."};
            }
            else if (jsonArray.size() == 0) {
                return new String[]{"해당 좌표에 대한 주소 정보가 없습니다."};
            }

            JsonObject local = (JsonObject) jsonArray.get(0);
            JsonObject jsonArray1 = (JsonObject) local.get("address");
            String localAddress = jsonArray1.get("address_name").getAsString();
            String[] address = localAddress.split(" ");
            return address;
        } catch (HttpClientErrorException e) {
            return new String[]{"Kakao API 요청 오류: " + e.getMessage()};
        } catch (Exception e) {
            return new String[]{"오류가 발생했습니다: " + e.getMessage()};
        }
    }
}
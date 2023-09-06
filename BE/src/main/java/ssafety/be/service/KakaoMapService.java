package ssafety.be.service;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.Charset;

@Service
public class KakaoMapService {

    @Value("${kakao.api.key}")
    private static String apiKey;

    /**
     * 경위도 정보로 주소를 불러오는 메소드
     * @throws UnsupportedEncodingException
     */
    public static String coordToAddr(double longitude, double latitude){
        String url = "https://dapi.kakao.com/v2/local/geo/coord2address.json?x="+longitude+"&y="+latitude;
        String addr = "";
        try{
            addr = getRegionAddress(getJSONData(url));
            //LOGGER.info(addr);
        }catch(Exception e){
            System.out.println("주소 api 요청 에러");
            e.printStackTrace();
        }
        return addr;
    }

    /**
     * REST API로 통신하여 받은 JSON형태의 데이터를 JsonObject로 파싱하는 메소드
     */
    private static JsonObject getJSONData(String apiUrl) throws Exception {
        HttpURLConnection conn = null;
        JsonObject jsonResponse = null;

        // 인증키 - KakaoAK하고 한 칸 띄워주셔야해요!
        String auth = "KakaoAK 4dcde7448c280266730f45df159cd184";

        // URL 설정
        URL url = new URL(apiUrl);

        conn = (HttpURLConnection) url.openConnection();

        // Request 형식 설정
        conn.setRequestMethod("GET");
        conn.setRequestProperty("X-Requested-With", "curl");
        conn.setRequestProperty("Authorization", auth);

        // Response 받기
        int responseCode = conn.getResponseCode();
        if (responseCode == 400) {
            System.out.println("400:: 해당 명령을 실행할 수 없음");
        } else if (responseCode == 401) {
            System.out.println("401:: Authorization이 잘못됨");
        } else if (responseCode == 500) {
            System.out.println("500:: 서버 에러, 문의 필요");
        } else { // 성공 후 응답 JSON 데이터 받기
            InputStreamReader reader = new InputStreamReader(conn.getInputStream(), Charset.forName("UTF-8"));
            JsonElement jsonElement = JsonParser.parseReader(reader);
            if (jsonElement.isJsonObject()) {
                jsonResponse = jsonElement.getAsJsonObject();
            }
        }

        return jsonResponse;
    }

    /**
     * JsonObject에서 주소값(address_name)만 추출하는 메소드
     */
    private static String getRegionAddress(JsonObject jsonObject) {
        String value = "";

        if (jsonObject != null) {
            JsonArray documentsArray = jsonObject.getAsJsonArray("documents");
            if (documentsArray != null && documentsArray.size() > 0) {
                JsonObject subJobj = documentsArray.get(0).getAsJsonObject();
                JsonObject roadAddress = subJobj.getAsJsonObject("road_address");

                if (roadAddress == null) {
                    JsonObject subsubJobj = subJobj.getAsJsonObject("address");
                    value = subsubJobj.get("address_name").getAsString();
                } else {
                    value = roadAddress.get("address_name").getAsString();
                }

                if (value.equals("") || value == null) {
                    subJobj = documentsArray.get(1).getAsJsonObject();
                    subJobj = subJobj.getAsJsonObject("address");
                    value = subJobj.get("address_name").getAsString();
                }
            }
        }
        return value;
    }
}


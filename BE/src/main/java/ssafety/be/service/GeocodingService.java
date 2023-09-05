package ssafety.be.service;

import com.google.maps.GeoApiContext;
import com.google.maps.GeocodingApi;
import com.google.maps.model.GeocodingResult;
import com.google.maps.model.LatLng;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

/**
 * GeocodingService 클래스는 Google Geocoding API를 사용하여 좌표를 주소로 변환하는 서비스를 제공합니다.
 */
@Service
public class GeocodingService {

    @Value("${google.maps.api.key}")
    private String apiKey;

    /**
     * 주어진 위도와 경도를 사용하여 좌표를 주소로 변환합니다.
     *
     * @param latitude  위도 값을 나타내는 double
     * @param longitude 경도 값을 나타내는 double
     * @return 좌표에 대한 주소를 나타내는 문자열
     * @throws Exception 변환 중에 예외가 발생할 경우
     */
    public String convertCoordinatesToAddress(double latitude, double longitude) throws Exception {
        GeoApiContext context = new GeoApiContext.Builder()
                .apiKey(apiKey)
                .build();

        LatLng location = new LatLng(latitude, longitude); // LatLng 객체 생성

        GeocodingResult[] results = GeocodingApi.reverseGeocode(context, location).await();

        if (results.length > 0) {
            return results[0].formattedAddress;
        } else {
            return "주소를 찾을 수 없습니다.";
        }
    }
}

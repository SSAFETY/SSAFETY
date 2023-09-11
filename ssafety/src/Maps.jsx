import React, { useEffect, useState } from 'react';
import { MapContainer, TileLayer, GeoJSON } from 'react-leaflet';
import 'leaflet/dist/leaflet.css'; // Leaflet CSS 파일을 가져와야 합니다.

function App() {
  const [seoulGeojson, setSeoulGeojson] = useState(null);

  useEffect(() => {
    // 서울시 경계의 GeoJSON 파일 경로
    const seoulGeojsonPath = `${process.env.PUBLIC_URL}/data/Seoul_Gu.geojson`;

    fetch(seoulGeojsonPath)
      .then((response) => response.json())
      .then((data) => {
        setSeoulGeojson(data);
      });
  }, []);

  const seoulBounds = [
    [37.425962, 126.764740], // 서울시 남서쪽 경계 좌표
    [37.701940, 127.183589], // 서울시 북동쪽 경계 좌표
  ];

  return (
    <div className="App">
      <MapContainer
        bounds={seoulBounds}
        style={{ height: '600px', width: '800px' }}
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        />
        {seoulGeojson && (
          <GeoJSON
            data={seoulGeojson}
            style={(feature) => ({
              fillColor: feature.properties.SI_NM === '서울특별시' ? 'white' : 'blue', // 서울시인 경우 흰색, 아닌 경우 파란색
              color: 'blue', // 테두리 색상
              weight: 2, // 테두리 두께
              opacity: 1, // 테두리 투명도
              fillOpacity: 0.5, // 채우기 투명도
            })}
          />
        )}
      </MapContainer>
    </div>
  );
}

export default App;

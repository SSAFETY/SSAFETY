import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import mapogu from '../mapData/mapo.json';
import '../css/DetailGu.css'; // 여기서 CSS 파일을 불러옵니다.
import Swal from 'sweetalert2';
import { initializeApp } from 'firebase/app';
import { getAnalytics } from 'firebase/analytics';
import { getFirestore, doc, setDoc, onSnapshot, collection } from 'firebase/firestore'; 

const firebaseConfig = {
  apiKey: "AIzaSyCe8s_k1g8-g2qRvgv3i0lJwFuVLRAMJtU",
  authDomain: "ssafy9th-19ed3.firebaseapp.com",
  databaseURL: "https://ssafy9th-19ed3-default-rtdb.firebaseio.com",
  projectId: "ssafy9th-19ed3",
  storageBucket: "ssafy9th-19ed3.appspot.com",
  messagingSenderId: "219156140787",
  appId: "1:219156140787:web:007d653d7bae027a1f9d86",
  measurementId: "G-M7W3LJDE33"
};

const app = initializeApp(firebaseConfig);
const analytics = getAnalytics(app);

const GuMap = () => {
  const chart = useRef(null);
  const featureData = feature(mapogu, mapogu.objects['mapo']);
  const [selectedVehicle, setSelectedVehicle] = useState('car1');
  const [selectedRoute, setSelectedRoute] = useState('1');
  const [location, setLocation] = useState('sangam');
  const [pins, setPins] = useState({});
  const projection = d3.geoMercator().scale(1).translate([0, 0]);
  const svgRef = useRef(null);
  const [carData, setCarData] = useState({});

  useEffect(() => {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const path = d3.geoPath().projection(projection);
    const bounds = path.bounds(featureData);
    const dx = bounds[1][0] - bounds[0][0];
    const dy = bounds[1][1] - bounds[0][1];
    const x = (bounds[0][0] + bounds[1][0]) / 2;
    const y = (bounds[0][1] + bounds[1][1]) / 2;
    const scale = 1.5 / Math.max(dx / width, dy / height);
    const translate = [width / 2 - scale * x, height / 2 - scale * y + 220];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).selectAll('svg').remove(); // 기존의 SVG 제거
    const newSvg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = newSvg.append('g');

    mapLayer
      .selectAll('path')
      .data(featureData.features)
      .enter()
      .append('path')
      .filter((d) => d.properties.temp === '마포구 상암동')
      .attr('d', path)
      .style('transition', 'transform 0.2s');

    svgRef.current = newSvg; // 새로운 SVG를 참조합니다.
  }, [featureData, projection]);

  useEffect(() => {
    const firestore = getFirestore(app);
    const carsCollection = collection(firestore, 'car');

    const unsubscribe = onSnapshot(carsCollection, (querySnapshot) => {
      const newData = {};
      querySnapshot.forEach((doc) => {
        newData[doc.id] = doc.data();
      });
      setCarData(newData);
    });

    return () => unsubscribe();
  }, []);

  useEffect(() => {
    const updatePins = () => {
      const mapLayer = svgRef.current.select('g'); // 지도 레이어 선택

      mapLayer.selectAll('.pin').remove();

      Object.keys(carData).forEach((vehicle) => {
        const { gps_x, gps_y } = carData[vehicle];
        console.log(`Vehicle: ${vehicle}, GPS_X: ${gps_x}, GPS_Y: ${gps_y}`);
        const [x, y] = projection([gps_x, gps_y]);

        mapLayer
          .append('circle')
          .attr('class', 'pin')
          .attr('cx', x)
          .attr('cy', y)
          .attr('r', 5)
          .style('fill', 'red'); // 원하는 색상 설정
      });
    };

    const interval = setInterval(updatePins, 100);

    return () => clearInterval(interval);
  }, [carData, projection]);

  const handleSendData = () => {
    if (selectedVehicle.trim() !== '' && selectedRoute.trim() !== '') {
      const vehicleData = {
        path: selectedRoute,
        location: location
      };

      const firestore = getFirestore(app);
      const docRef = doc(firestore, 'car', selectedVehicle);

      setDoc(docRef, vehicleData, { merge: true })
        .then(() => {
          Swal.fire('차량 경로가 설정되었습니다.')
        })
        .catch((error) => {
          console.error('데이터 전송 중 오류 발생:', error);
        });

      setSelectedRoute('path1');
      setLocation('sangam');
    } else {
      Swal.fire('차량 번호와 경로를 선택해주세요.');
    }
  };

  return (
    <div className="vehicle-board">
      <div className="container">
        <div className="map-container">
          <div className="gumap" ref={chart}></div>
        </div>
        <div className="form-container"> {/* 경로 설정 폼 컨테이너 추가 */}
          <div className="input-container">
            <select value={selectedVehicle} onChange={(e) => setSelectedVehicle(e.target.value)}>
              <option value="car1">차량 1</option>
              <option value="car2">차량 2</option>
              <option value="car3">차량 3</option>
            </select>
            <select value={selectedRoute} onChange={(e) => setSelectedRoute(e.target.value)}>
              <option value="1">경로 1</option>
              <option value="2">경로 2</option>
              <option value="3">경로 3</option>
            </select>
            <button onClick={handleSendData}>추가</button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default GuMap;

import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import mapogu from '../mapData/mapo.json';
import '../css/DetailGu.css';
import Swal from 'sweetalert2';
import { initializeApp } from 'firebase/app';
import { getAnalytics } from 'firebase/analytics';
import { getFirestore, doc, setDoc, collection, getDocs } from 'firebase/firestore';

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
  const projection = d3.geoMercator().scale(1).translate([0, 0]);
  const svgRef = useRef(null);
  const [carData, setCarData] = useState({});
  const carImageURL = 'https://a102.s3.ap-northeast-2.amazonaws.com/png-transparent-car-top-view-blue-convertible-coupe-illustration-compact-car-blue-plan-thumbnail.png'; // 자동차 이미지 URL

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
    const translate = [width / 2 - scale * x + 80, height / 2 - scale * y + 235];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = svg.append('g');

    mapLayer
      .selectAll('path')
      .data(featureData.features)
      .enter()
      .append('path')
      .filter((d) => d.properties.temp === '마포구 상암동')
      .attr('d', path)
      .style('fill', 'skyblue')
      .style('transition', 'transform 0.2s');

    svg.append('g').attr('class', 'pin-group');
    svgRef.current = svg;
  }, [featureData]);

  useEffect(() => {
    const fetchDataAndUpdatePins = async () => {
      try {
        const firestore = getFirestore(app);
        const carsCollection = collection(firestore, 'car');
        const queryDocs = await getDocs(carsCollection);
  
        const newData = {};
        queryDocs.forEach((doc) => {
          newData[doc.id] = doc.data();
        });
  
        setCarData(newData);
  
        // 핀 업데이트 처리
        const pinGroup = svgRef.current.select('.pin-group');
        pinGroup.selectAll('.pin').remove();
  
        Object.keys(newData).forEach((vehicle) => {
          const { gps_x, gps_y } = newData[vehicle];
          const [x, y] = projection([parseFloat(gps_x), parseFloat(gps_y)]);
  
          // 자동차 이미지 핀을 추가합니다.
          pinGroup
            .append('image')
            .attr('x', x - 20) // 이미지의 중심을 핀의 위치로 조절
            .attr('y', y - 20) // 이미지의 중심을 핀의 위치로 조절
            .attr('width', 30) // 이미지 너비
            .attr('height', 30) // 이미지 높이
            .attr('xlink:href', carImageURL) // 이미지 URL
            .style('background', skyblue);
        });
      } catch (error) {
        console.error('데이터 가져오기 또는 핀 업데이트 오류:', error);
      }
    };
  
    // 0.1초(100ms)마다 데이터를 가져오고 핀 업데이트 처리를 함께 수행
    const interval = setInterval(fetchDataAndUpdatePins, 100);
  
    // 컴포넌트가 언마운트될 때 인터벌을 클리어합니다.
    return () => clearInterval(interval);
  }, [projection]);

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
          Swal.fire('차량 경로가 설정되었습니다.');
        })
        .catch((error) => {
          console.error('데이터 전송 중 오류 발생:', error);
        });

      setSelectedRoute('path1');
      setLocation('sangam');
    }
  };

  return (
    <div className="vehicle-board">
      <div className="container">
        <div className="gumap" ref={chart}></div>
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
  );
};

export default GuMap;

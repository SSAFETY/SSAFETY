import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import mapogu from '../mapData/mapo.json';
import '../css/BusMap.css';
import Swal from 'sweetalert2';
import { initializeApp } from 'firebase/app';
import { getAnalytics } from 'firebase/analytics';
import { getFirestore, doc, setDoc, onSnapshot, collection } from 'firebase/firestore';
import { FormControl, InputLabel, MenuItem, Select, Button, List, ListItem, ListItemText } from '@mui/material';

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

const BusMap = () => {
  const chart = useRef(null);
  const featureData = feature(mapogu, mapogu.objects['mapo']);
  const [selectedVehicle, setSelectedVehicle] = useState('car1');
  const [selectedRoute, setSelectedRoute] = useState('1');
  const [location, setLocation] = useState('sangam');
  const [pins, setPins] = useState({});
  const projection = d3.geoMercator().scale(1).translate([0, 0]);
  const svgRef = useRef(null);
  const [carData, setCarData] = useState({});

  const getAddress = async (gps_x, gps_y) => {
    try {
      const apiUrl = `https://j9a102.p.ssafy.io/api/getAddress?latitude=${gps_y}&longitude=${gps_x}`;
      const response = await fetch(apiUrl);
      
      if (!response.ok) {
        throw new Error('API 요청이 실패했습니다.');
      }

      const dataArray = await response.json();
      const data = dataArray[0] + dataArray[1] + dataArray[2] + dataArray[3];
      return data;
    } catch (error) {
      console.error('주소 정보를 가져오는 중 오류 발생:', error);
      return null;
    }
  };

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
    const translate = [width / 2 - scale * x, height / 2 - scale * y + 240];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).selectAll('svg').remove();
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

    svgRef.current = newSvg;
  }, [featureData, projection]);

  useEffect(() => {
    const firestore = getFirestore(app);
    const carsCollection = collection(firestore, 'car');

    const unsubscribe = onSnapshot(carsCollection, (querySnapshot) => {
      const newData = {};
      querySnapshot.forEach((doc) => {
        const data = doc.data();
        newData[doc.id] = {
          ...data,
          velocity: data.velocity || "0.0"
        };
      });
      setCarData(newData);
    });

    return () => unsubscribe();
  }, []);

  useEffect(() => {
    const updatePins = async () => {
      const mapLayer = svgRef.current.select('g');
      
      // 이전 핀을 모두 지우기
      mapLayer.selectAll('.pin').remove();
      
      const updatedCarData = {};
  
      for (const vehicle of Object.keys(carData)) {
        const { gps_x, gps_y } = carData[vehicle];
        const addressData = await getAddress(gps_x, gps_y);
  
        updatedCarData[vehicle] = {
          ...carData[vehicle],
          address: addressData || '주소를 찾을 수 없음',
        };
        
        const [x, y] = projection([gps_x, gps_y]);
  
        mapLayer
          .append('circle')
          .attr('class', 'pin')
          .attr('cx', x)
          .attr('cy', y)
          .attr('r', 5)
          .style('fill', 'red')
          .append('title');
      }
  
      setCarData(updatedCarData);
    };
  
    const interval = setInterval(updatePins, 1000);
  
    return () => clearInterval(interval);
  }, [projection, carData]);
  

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
        <div className="form-container">
          <div className="input-container">
            <div className="form-control" style={{ marginBottom: '20px' }}>
              <FormControl variant="outlined">
                <InputLabel htmlFor="vehicle-select">차량</InputLabel>
                <Select
                  value={selectedVehicle}
                  onChange={(e) => setSelectedVehicle(e.target.value)}
                  label="차량"
                  inputProps={{
                    name: 'vehicle',
                    id: 'vehicle-select',
                  }}
                >
                  <MenuItem value="car1">차량 1</MenuItem>
                  <MenuItem value="car2">차량 2</MenuItem>
                  <MenuItem value="car3">차량 3</MenuItem>
                </Select>
              </FormControl>
            </div>
            <div className="form-control" style={{ marginBottom: '20px' }}>
              <FormControl variant="outlined">
                <InputLabel htmlFor="route-select">경로</InputLabel>
                <Select
                  value={selectedRoute}
                  onChange={(e) => setSelectedRoute(e.target.value)}
                  label="경로"
                  inputProps={{
                    name: 'route',
                    id: 'route-select',
                  }}
                >
                  <MenuItem value="1">경로 1</MenuItem>
                  <MenuItem value="2">경로 2</MenuItem>
                  <MenuItem value="3">경로 3</MenuItem>
                </Select>
              </FormControl>
            </div>
            <Button variant="contained" onClick={handleSendData}>
              추가
            </Button>
          </div>
          <div className="form-control" style={{ marginBottom: '20px' }}>
            <List>
              {Object.keys(carData).map((vehicle) => (
                <ListItem key={vehicle}>
                  <ListItemText
                    primary={`차량: ${vehicle}`}
                    secondary={`주소: ${carData[vehicle].address}, 속도: ${carData[vehicle].velocity} km / h`}
                  />
                </ListItem>
              ))}
            </List>
          </div>
        </div>
      </div>
    </div>
  );
};

export default BusMap;

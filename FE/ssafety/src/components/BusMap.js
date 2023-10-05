import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import mapogu from '../mapData/mapo.json';
import '../css/BusMap.css';
import Swal from 'sweetalert2';
import { initializeApp } from 'firebase/app';
import { getAnalytics } from 'firebase/analytics';
import {
  getFirestore,
  doc,
  setDoc,
  onSnapshot,
  collection,
} from 'firebase/firestore';
import {
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Button,
  List,
  ListItem,
  ListItemText,
  Typography,
  ListItemIcon,
} from '@mui/material';
import SpeedIcon from '@mui/icons-material/Speed';

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
  const pinImage = 'https://a102.s3.ap-northeast-2.amazonaws.com/symbole-de-bus-orange.png'

  const getAddress = async (gps_x, gps_y) => {
    try {
      const apiUrl = `https://j9a102.p.ssafy.io/api/getAddress?latitude=${gps_y}&longitude=${gps_x}`;
      const response = await fetch(apiUrl);

      if (!response.ok) {
        throw new Error('API 요청이 실패했습니다.');
      }

      const dataArray = await response.json();
      const data = dataArray[0] + ' ' + dataArray[1] + ' ' + dataArray[2] + ' ' + dataArray[3];
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
    const translate = [width / 2 - scale * x + 110, height / 2 - scale * y + 320];
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
      .style('transition', 'transform 0.2s')
      .style('fill', 'skyblue')

    svgRef.current = newSvg;
  }, [featureData]);

  useEffect(() => {
    const firestore = getFirestore(app);
    const carsCollection = collection(firestore, 'car');
  
    const unsubscribe = onSnapshot(carsCollection, async (querySnapshot) => {
      const newData = {};
  
      for (const doc of querySnapshot.docs) {
        const data = doc.data();
        const addressData = await getAddress(data.gps_x, data.gps_y);
        newData[doc.id] = {
          ...data,
          address: addressData || '주소를 찾을 수 없음',
          velocity: data.velocity || "0.0"
        };
      }
  
      setCarData(newData);
    });
  
    return () => unsubscribe();
  }, []);

  useEffect(() => {
    const updatePins = async () => {
      const mapLayer = svgRef.current.select('g');

      mapLayer.selectAll('.pin').remove();

      for (const vehicle of Object.keys(carData)) {
        mapLayer
        .selectAll('.pin')
        .data(Object.keys(carData))
        .enter()
        .append('image') // 이미지 요소 추가
        .attr('class', 'pin')
        .attr('x', (d) => {
          const { gps_x } = carData[d];
          return projection([gps_x, 0])[0]; // 경도만 사용하여 X 좌표 계산
        })
        .attr('y', (d) => {
          const { gps_y } = carData[d];
          return projection([0, gps_y])[1]; // 위도만 사용하여 Y 좌표 계산
        })
        .attr('xlink:href', pinImage) // 이미지 핀 URL 설정
        .attr('width', 30) // 핀 이미지의 가로 크기 조정
        .attr('height', 30) // 핀 이미지의 세로 크기 조정
      }
    };

    updatePins();

    const interval = setInterval(updatePins, 1000);

    return () => clearInterval(interval);
  }, [carData]);

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
                <ListItem
                  key={vehicle}
                  sx={{
                    border: '1px solid #ccc',
                    borderRadius: '8px',
                    marginBottom: '10px',
                    backgroundColor: '#f5f5f5',
                  }}
                >
                  <ListItemIcon>
                    <SpeedIcon />
                  </ListItemIcon>
                  <ListItemText
                    primary={<Typography variant="h6">{`차량: ${vehicle}`}</Typography>}
                    secondary={<Typography variant="subtitle1">{`주소: ${carData[vehicle].address}, 속도: ${carData[vehicle].velocity} km / h`}</Typography>}
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

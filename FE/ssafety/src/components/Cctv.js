import React, { useState, useEffect } from 'react';
import axios from 'axios';

const Cctv = () => {
  const [cctvData, setCCTVData] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    console.log('데이터 송신을 요청했습니다.');
    axios
      .get('https://j9a102.p.ssafy.io/api/cctv/data')
      .then((response) => {
        // response.data.data를 사용하여 cctvData 배열 설정
        setCCTVData(response.data);
        console.log(response.data);
        setLoading(false);
      })
      .catch((error) => {
        console.error('Failed to fetch CCTV data:', error);
        setLoading(false);
      });
  }, []);

  // 첫 번째 CCTV 데이터 가져오기
  const firstCctv = cctvData.length > 0 ? cctvData[0] : null;
  console.log(firstCctv)

  return (
    <div>
      <h1>CCTV Data</h1>
      {/* {loading ? (
        <p>Loading...</p>
      ) : firstCctv ? (
        <div>
          <video controls width="640" height="360">
            <source src={firstCctv.cctvurl} type="application/x-mpegURL" />
            Your browser does not support the video tag.
          </video>
        </div>
      ) : (
        <p>No CCTV data available.</p>
      )} */}
    </div>
  );
};

export default Cctv;

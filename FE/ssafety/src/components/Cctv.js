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
  const firstCctv = cctvData[0];
  console.log(firstCctv)
  console.log(firstCctv.cctvurl)

  return (
    <div>
      <h1>CCTV Data</h1>
      {loading ? (
        <p>Loading...</p>
      ) : (
        <div>
          <video controls width="640" height="360">
            <source src={firstCctv.cctvurl} type="application/x-mpegURL" />
            Your browser does not support the video tag.
          </video>
        </div>
      )}
    </div>
  );
};

export default Cctv;

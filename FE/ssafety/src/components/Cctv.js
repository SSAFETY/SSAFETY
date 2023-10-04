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
        // response.data.response.data를 사용하여 cctvData 배열 설정
        const cctvUrls = response.data.response.data.map((cctv) => cctv.cctvurl);
        setCCTVData(cctvUrls);
        console.log(cctvUrls);
        setLoading(false);
      })
      .catch((error) => {
        console.error('Failed to fetch CCTV data:', error);
        setLoading(false);
      });
  }, []);

  return (
    <div>
      <h1>CCTV Data</h1>
      {loading ? (
        <p>Loading...</p>
      ) : cctvData.length > 0 ? (
        <div>
          <video controls width="640" height="360">
            <source src={'http://cctvsec.ktict.co.kr/6/9HxtCvRB12u+NMBJD2dAshoqrFv73rhdeBUuGvlWul2eve26ZKz20Ht77ZlKx3yAjuSq8yZ+1jTICSSjbh2q1A=='} type="application/x-mpegURL" />
            Your browser does not support the video tag.
          </video>
        </div>
      ) : (
        <p>No CCTV data available.</p>
      )}
    </div>
  );
};

export default Cctv;

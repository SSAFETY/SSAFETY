import React, { useState, useEffect } from 'react';
import axios from 'axios';

const Cctv = () => {
  const [cctvData, setCCTVData] = useState('');
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    axios
      .get('/cctv/data') // 이 URL은 Spring Boot 애플리케이션의 엔드포인트와 맞춰야 합니다.
      .then((response) => {
        setCCTVData(response.data);
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
      ) : (
        <pre>{JSON.stringify(cctvData, null, 2)}</pre>
      )}
    </div>
  );
};

export default Cctv;

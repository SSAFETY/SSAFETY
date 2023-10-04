import React, { useState, useEffect } from 'react';
import axios from 'axios';

const Cctv = () => {
  const [cctvData, setCCTVData] = useState('');
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    axios
      .get('https://j9a102.p.ssafy.io:8080/api/cctv/data')
      .then((response) => {
        setCCTVData(response.data);
        console.log(response.data)
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

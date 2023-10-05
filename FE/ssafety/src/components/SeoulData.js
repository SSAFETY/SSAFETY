import React, { useEffect, useState } from 'react';
import axios from 'axios';
import AiResult from './AiResult';
import GuResult from './GuResult';
import TimeResult from './TimeResult';

const SeoulData = () => {
  const [data, setData] = useState([]);

  useEffect(() => {
    // 서버에서 데이터를 가져오는 함수
    const fetchData = async () => {
      try {
        const response = await axios.get('https://j9a102.p.ssafy.io/api/getData');
        const responseData = response.data;
        setData(responseData);
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };

    fetchData();
  }, []);

  return (
    <div className='seoul-data-container'>
      <div className='horizontal-graphs'>
        <div className='horizontal-graph'>
          <GuResult data={data} />
        </div>
        <div className='horizontal-graph'>
          <TimeResult data={data} />
        </div>
        <div className='horizontal-graph'>
          <AiResult data={data} />
        </div>
      </div>
    </div>
  );
};

export default SeoulData;
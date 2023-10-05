import React from 'react';
import SeoulData from './SeoulData'; // 서울 데이터를 표시하는 컴포넌트
import SeoulMap from './SeoulMap';   // 서울 지도를 표시하는 컴포넌트
import '../css/SeoulMap.css'

const Seoul = () => {
  return (
    <div className='seoul'>
      <div className='seoul-map-container'>
        <SeoulMap />
      </div>
      <div className='seoul-data-container'>
        <SeoulData />
      </div>
    </div>
  );
};

export default Seoul;

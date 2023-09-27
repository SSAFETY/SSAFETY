import React from 'react';
import SeoulData from './SeoulData'; // 서울 데이터를 표시하는 컴포넌트
import SeoulMap from './SeoulMap';   // 서울 지도를 표시하는 컴포넌트

const Seoul = () => {
  return (
    <div>
      <SeoulData />
      <SeoulMap />
    </div>
  );
};

export default Seoul;

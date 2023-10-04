import React from 'react';
import { ResponsiveBar } from '@nivo/bar';

const GuResult = ({ data }) => {
  // 데이터 가공: 3글자 이상인 데이터의 마지막 글자 제거
  const groupedData = data.reduce((acc, item) => {
    let depth3 = item.depth3;
    if (depth3.length >= 3) {
      depth3 = depth3.slice(0, -1); // 마지막 글자 제거
    }
    if (!acc[depth3]) {
      acc[depth3] = 0;
    }
    acc[depth3]++;
    return acc;
  }, {});

  // 데이터 포맷 변환
  const formattedData = Object.keys(groupedData).map(key => ({
    depth3: key,
    사건발생수 : groupedData[key],
  }));

  // y값 범위 설정
  const minY = Math.min(...formattedData.map(d => d.사건발생수));
  const maxY = Math.max(...formattedData.map(d => d.사건발생수));

  // y값에 따른 그라데이션 설정
  const colorScale = (value) => {
    const alpha = (value - minY) / (maxY - minY); // 그라데이션 계산
    const r = 0;
    const g = Math.round(255 * (1 - alpha)); // 하늘색에서 흰색으로 그라데이션
    const b = 255;
    const color = `rgba(${r}, ${g}, ${b}, 1)`;
    return color;
  };

  return (
    <div className='horizontal-graph' style={{ height: '250px' }}>
      <ResponsiveBar
        data={formattedData}
        keys={['사건발생수']}
        indexBy="depth3"
        margin={{ top: 50, right: 60, bottom: 50, left: 60 }}
        padding={0.3}
        colors={({ value }) => colorScale(value)}
        axisTop={null}
        axisRight={null}
        axisBottom={{
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: '', // x축 레전드를 비웁니다.
          legendPosition: 'middle',
          legendOffset: 32,
        }}
        axisLeft={{
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: '사건 발생 수',
          legendPosition: 'middle',
          legendOffset: -40,
        }}
        labelSkipWidth={12}
        labelSkipHeight={12}
        labelTextColor={{ from: 'color', modifiers: [['brighter', 1.6]] }}
        animate={true}
        motionStiffness={90}
        motionDamping={15}
      />
    </div>
  );
};

export default GuResult;

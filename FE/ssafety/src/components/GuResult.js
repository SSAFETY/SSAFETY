import React from 'react';
import { ResponsiveBar } from '@nivo/bar';

const GuResult = ({ data }) => {
  const groupedData = data.reduce((acc, item) => {
    const depth3 = item.depth3;
    if (!acc[depth3]) {
      acc[depth3] = 0;
    }
    acc[depth3]++;
    return acc;
  }, {});

  // 데이터 포맷 변환
  const formattedData = Object.keys(groupedData).map(key => ({
    depth3: key,
    count: groupedData[key],
  }));

  return (
    <div style={{ height: '400px' }}>
      <ResponsiveBar
        data={formattedData}
        keys={['count']}
        indexBy="depth3"
        margin={{ top: 50, right: 60, bottom: 50, left: 60 }}
        padding={0.3}
        colors={{ scheme: 'nivo' }}
        axisTop={null}
        axisRight={null}
        axisBottom={{
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: 'Depth3',
          legendPosition: 'middle',
          legendOffset: 32,
        }}
        axisLeft={{
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: 'Count',
          legendPosition: 'middle',
          legendOffset: -40,
        }}
        labelSkipWidth={12}
        labelSkipHeight={12}
        labelTextColor={{ from: 'color', modifiers: [['darker', 1.6]] }}
        animate={true}
        motionStiffness={90}
        motionDamping={15}
      />
    </div>
  );
};

export default GuResult;
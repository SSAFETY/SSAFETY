import React from 'react';
import { ResponsivePie } from '@nivo/pie';

const AiResult = ({ data }) => {
  // 데이터 가공 및 변환
  const aiResultCounts = [
    {
      id: '차선 침범',
      value: data.filter(item => item.aiResult === '차선 침범').length,
    },
    {
      id: '신호 위반',
      value: data.filter(item => item.aiResult === '신호 위반').length,
    },
    {
      id: '과속',
      value: data.filter(item => item.aiResult === '과속').length,
    },
  ];

  return (
    <div style={{ height: '400px' }}>
      <ResponsivePie
        data={aiResultCounts} // 변환된 데이터를 전달
        margin={{ top: 40, right: 80, bottom: 80, left: 80 }}
        innerRadius={0.5}
        padAngle={0.7}
        cornerRadius={3}
        colors={{ scheme: 'nivo' }}
        borderWidth={1}
        borderColor={{ from: 'color', modifiers: [['darker', 0.2]] }}
        radialLabelsSkipAngle={10}
        radialLabelsTextColor="#333333"
        radialLabelsLinkOffset={0}
        radialLabelsLinkDiagonalLength={16}
        radialLabelsLinkHorizontalLength={24}
        radialLabelsLinkStrokeWidth={1}
        radialLabelsLinkColor={{ from: 'color' }}
        slicesLabelsSkipAngle={10}
        slicesLabelsTextColor="#333333"
        animate={true}
        motionStiffness={90}
        motionDamping={15}
      />
    </div>
  );
};

export default AiResult;

import React from 'react';
import { ResponsiveLine } from '@nivo/line';

const TimeResult = ({ data }) => {
  // 시간대별 개수를 저장할 배열 초기화
  const timeCounts = new Array(24).fill(0);

  // 데이터에서 시간대별 개수 계산
  data.forEach(item => {
    const hour = item.creationTime[3];
    timeCounts[hour]++;
  });

  // 그래프 데이터 포맷 생성
  const formattedData = timeCounts.map((count, hour) => ({
    x: hour, // x축은 시간
    y: count, // y축은 개수
  }));

  // 커스텀 툴팁 함수 정의
  const customTooltip = ({ point }) => (
    <div style={{ background: 'white', padding: '10px' }}>
      시간: {point.data.x} 시
      <br />
      사건 발생 수: {point.data.y}
    </div>
  );

  return (
    <div className='horizontal-graph' style={{ height: '250px' }}>
      <ResponsiveLine
        data={[{ id: 'data', data: formattedData }]}
        margin={{ top: 50, right: 60, bottom: 50, left: 60 }}
        xScale={{
          type: 'point', // x축을 point로 설정
        }}
        yScale={{ type: 'linear', min: 0, max: 'auto', stacked: false, reverse: false }}
        axisTop={null}
        axisRight={null}
        axisBottom={{
          orient: 'bottom',
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: '시간', // x축 레전드
          legendOffset: 36,
          legendPosition: 'middle',
        }}
        axisLeft={{
          orient: 'left',
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: '사건 발생 수', // y축 레전드
          legendOffset: -40,
          legendPosition: 'middle',
        }}
        colors={{ scheme: 'nivo' }}
        pointSize={10}
        pointColor={{ theme: 'background' }}
        pointBorderWidth={2}
        pointBorderColor={{ from: 'serieColor' }}
        enableArea={false}
        enableCrosshair={false}
        useMesh={true}
        tooltip={customTooltip} // 커스텀 툴팁 함수 설정
      />
    </div>
  );
};

export default TimeResult;

import React from 'react';
import { ResponsiveLine } from '@nivo/line';

const TimeResult = ({ data }) => {
  // creation_time을 시간 기준으로 그룹화하여 데이터 가공
  const groupedData = data.reduce((acc, item) => {
    const createTime = new Date(item.creation_time);
    const hours = createTime.getHours();
    const minutes = createTime.getMinutes();
    const timeString = `${hours}:${minutes}`;

    if (!acc[timeString]) {
      acc[timeString] = 0;
    }
    acc[timeString]++;
    return acc;
  }, {});

  // 데이터 포맷 변환
  const formattedData = Object.keys(groupedData).map(timeString => ({
    time: timeString,
    count: groupedData[timeString],
  }));

  return (
    <div style={{ height: '400px' }}>
      <ResponsiveLine
        data={[{ id: 'data', data: formattedData }]}
        margin={{ top: 50, right: 60, bottom: 50, left: 60 }}
        xScale={{ type: 'point' }}
        yScale={{ type: 'linear', min: 0, max: 'auto', stacked: false, reverse: false }}
        axisTop={null}
        axisRight={null}
        axisBottom={{
          orient: 'bottom',
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: 'Time',
          legendOffset: 36,
          legendPosition: 'middle',
        }}
        axisLeft={{
          orient: 'left',
          tickSize: 5,
          tickPadding: 5,
          tickRotation: 0,
          legend: 'Count',
          legendOffset: -40,
          legendPosition: 'middle',
        }}
        colors={{ scheme: 'nivo' }}
        pointSize={10}
        pointColor={{ theme: 'background' }}
        pointBorderWidth={2}
        pointBorderColor={{ from: 'serieColor' }}
        pointLabel="y"
        pointLabelYOffset={-12}
        enableArea={false}
        enableCrosshair={false}
        useMesh={true}
      />
    </div>
  );
};

export default TimeResult;

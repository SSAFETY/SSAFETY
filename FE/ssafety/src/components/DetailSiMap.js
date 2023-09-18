import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import axios from 'axios';
import korea from '../mapData/skorea-municipalities-2018-topo.json';
import '../css/Home.css';
import { useNavigate } from 'react-router-dom';


const SeoulMap = () => {
  const chart = useRef(null);
  const featureData = feature(korea, korea.objects['SeoulMap']);
  const [data, setData] = useState({});
  const [selectedData, setSelectedData] = useState(null);
  const navigate = useNavigate();


  useEffect(() => {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const projection = d3.geoMercator().scale(1).translate([0, 0]);
    const path = d3.geoPath().projection(projection);
    const bounds = path.bounds(featureData);
    const dx = bounds[1][0] - bounds[0][0];
    const dy = bounds[1][1] - bounds[0][1];
    const x = (bounds[0][0] + bounds[1][0]) / 2;
    const y = (bounds[0][1] + bounds[1][1]) / 2;
    const scale = 6 / Math.max(dx / width, dy / height);
    const translate = [width / 2 - scale * x + 850, height / 2 - scale * y + 1950];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = svg.append('g');

    // 데이터를 받아오는 함수
    const fetchData = () => {
      axios
        .get('http://localhost:8080/getAll')
        .then((response) => {
          const reportData = response.data;
          const newData = {};

          reportData.forEach((report) => {
            const depth3 = report.depth3;
            const region = depth3; // depth3 값 그대로 사용
            if (newData[region]) {
              newData[region]++;
            } else {
              newData[region] = 1;
            }
          });

          setData(newData);

          const colorScale = d3.scaleLinear()
            .domain([0, d3.max(Object.values(newData))])
            .range(['skyblue', 'darkblue']);

          // 데이터를 받아온 후에 지도의 색상을 변경합니다.
          mapLayer
            .selectAll('path')
            .style('fill', (d) => {
              const region = d.properties.name;
              const value = newData[region] != null ? newData[region] : 0;
              const color = colorScale(value);
              return color;
            });
        })
        .catch((error) => {
          console.error('Error fetching data:', error);
        });
    };

    // fetchData 함수를 호출하여 데이터를 받아옵니다.
    fetchData();

    mapLayer
      .selectAll('path')
      .data(featureData.features)
      .enter()
      .append('path')
      .attr('d', path)
      .style('transition', 'transform 0.2s');

    const popupGroup = svg.append('g').style('display', 'none');

    mapLayer
      .selectAll('path')
      .on('mouseover', function (event, d) {
        const originalColor = d3.select(this).style('fill'); // 마우스를 올리기 전의 색상 기억
        d3.select(this).style('fill', 'red').attr('transform', 'translate(0, -5)');

        popupGroup.style('display', 'block');
        popupGroup.selectAll('*').remove();

        popupGroup
          .append('circle')
          .attr('cx', path.centroid(d)[0])
          .attr('cy', path.centroid(d)[1] - 30)
          .attr('r', 30)
          .style('fill', 'white')
          .style('stroke', 'blue')
          .style('stroke-width', 2);

        popupGroup
          .append('text')
          .attr('x', path.centroid(d)[0])
          .attr('y', path.centroid(d)[1] - 30)
          .attr('text-anchor', 'middle')
          .attr('alignment-baseline', 'middle')
          .text(d.properties.name)
          .style('fill', 'black')
          .style('font-size', '12px');

        d3.select(this).attr('data-original-color', originalColor);
      })
      .on('mouseout', function (event, d) {
        const originalColor = d3.select(this).attr('data-original-color'); // 기존 색상 가져오기
        d3.select(this).style('fill', originalColor).attr('transform', 'translate(0, 0)');
        popupGroup.style('display', 'none');
      })
      .on('click', function (event, d) {
        if (d.properties.name === '마포구') {
          navigate('detailgu');
        }
      });
  }, []); // 빈 배열로 전달하여 초기 렌더링 시에만 호출되도록 설정

  return (
    <div className='seoulmap-container'>
      <div className='seoulmap' ref={chart}></div>
      {selectedData && (
        <div className='selected-data-right'>
          <h2>Selected Data</h2>
          <p>Creation Time: {selectedData.creationTime}</p>
          <p>AI Result: {selectedData.aiResult}</p>
          <p>GPS Location: {selectedData.gpsLocation}</p>
          <p>Vehicle Number: {selectedData.vehicleNumber}</p>
          {selectedData.videoUrl && (
            <video controls width="100%">
            <source src={selectedData.videoUrl} type="video/mp4" />
            Your browser does not support the video tag.
            </video>
          )}
        </div>
      )}
    </div>
  );
};

export default SeoulMap;
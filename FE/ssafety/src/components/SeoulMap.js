import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import axios from 'axios';
import korea from '../mapData/skorea-municipalities-2018-topo.json';
import '../css/SeoulMap.css'

const SeoulMap = () => {
  const chart = useRef(null);
  const featureData = feature(korea, korea.objects['SeoulMap']);
  const [data, setData] = useState({});
  const [selectedData, setSelectedData] = useState(null);

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
    const scale = 7 / Math.max(dx / width, dy / height);
    const translate = [width / 1 - scale * x + 700, height / 1 - scale * y + 2850];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = svg.append('g');

    const fetchData = () => {
      axios
        .get('https://j9a102.p.ssafy.io/api/getData')
        .then((response) => {
          const reportData = response.data;
          const newData = {};
          reportData.forEach((report) => {
            const depth3 = report.depth3;
            const region = depth3;
            if (newData[region]) {
              newData[region]++;
            } else {
              newData[region] = 1;
            }
          });

          setData(newData);

          const colorScale = d3.scaleSequential(d3.interpolateBlues) // 색상 척도 설정
            .domain([0, d3.max(Object.values(newData))]);

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
        const originalColor = d3.select(this).style('fill');
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
        const originalColor = d3.select(this).attr('data-original-color');
        d3.select(this).style('fill', originalColor).attr('transform', 'translate(0, 0)');
        popupGroup.style('display', 'none');
      })
  }, []);

  return (
    <div className='seoulmap' ref={chart}></div>
  );
};

export default SeoulMap;

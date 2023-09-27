import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import axios from 'axios';
import korea from '../mapData/skorea-municipalities-2018-topo.json';
import { useNavigate } from 'react-router-dom';
import Swal from 'sweetalert2';
import '../css/SeoulMap.css'

const SeoulMap = () => {
  const chart = useRef(null);
  const featureData = feature(korea, korea.objects['SeoulMap']);
  const [data, setData] = useState({});
  const [selectedData, setSelectedData] = useState(null);
  const navigate = useNavigate();

  const handleButton = () => {
    if(selectedRegion === "서울시" && selectedDistrict === "마포구" && selectedDong === "상암동") {
      navigate('/detailsi/detailgu');
    } else {
      Swal.fire('아직 지원하지 않는 구역입니다!');
    }
  }

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
    const translate = [width / 1 - scale * x + 250, height / 1 - scale * y + 1900];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = svg.append('g');

    const fetchData = () => {
      axios
        // .get('http://localhost:8080/api/getData')
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

          const colorScale = d3.scaleLinear()
            .domain([0, d3.max(Object.values(newData))])
            .range(['skyblue', 'darkblue']);

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
      .on('click', function (event, d) {
        if (d.properties.name === '마포구') {
          navigate('detailgu');
        } else {
          Swal.fire('아직 지원하지 않는 구역입니다!');
        }
      });
  }, []);

  return (
    <div className='seoulmap' ref={chart}></div>
  );
};

export default SeoulMap;

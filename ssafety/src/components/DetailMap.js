import React, { useEffect, useRef } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import sung from '../mapData/sung.json'; // sung.json은 GeoJSON 데이터 파일입니다.

const DetailMap = () => {
  const chart = useRef(null);

  const printD3 = () => {
    const width = window.innerWidth; // 페이지 너비로 설정
    const height = window.innerHeight; // 페이지 높이로 설정

    const projection = d3.geoMercator().scale(1).translate([0, 0]);
    const path = d3.geoPath().projection(projection);
    const featureData = feature(sung, sung.objects['gu']); // 데이터 소스에 맞게 수정
    const bounds = path.bounds(featureData);

    const dx = bounds[1][0] - bounds[0][0];
    const dy = bounds[1][1] - bounds[0][1];
    const x = (bounds[0][0] + bounds[1][0]) / 2;
    const y = (bounds[0][1] + bounds[1][1]) / 2;
    const scale = 0.9 / Math.max(dx / width, dy / height);
    const translate = [width / 2 - scale * x, height / 2 - scale * y];

    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);

    const mapLayer = svg.append('g');

    // SIG_KOR_NM이 성남시로 시작하는 지역만 필터링합니다.
    const filteredFeatures = featureData.features.filter((feature) =>
      feature.properties.SIG_KOR_NM.startsWith('성남시')
    );

    // 필터링된 지역만을 지도에 표시합니다.
    mapLayer
      .selectAll('path')
      .data(filteredFeatures)
      .enter()
      .append('path')
      .attr('d', path)
      .style('fill', 'skyblue') // 기본 색상 설정
      .style('transition', 'transform 0.2s'); // 트랜지션 효과 추가
  };

  useEffect(() => {
    printD3();
  }, []);

  return <div ref={chart} className="detail-map"></div>;
};

export default DetailMap;

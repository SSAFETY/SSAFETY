import React, { useEffect, useRef } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import sung from '../mapData/skorea-municipalities-2018-topo.json';
import '../css/Home.css';

const SungMap = () => {
  // svg를 그릴 엘리먼트 설정을 위한 ref
  const chart = useRef(null);

  const printD3 = () => {
    // 부모 요소의 크기를 가져오도록 수정
    const parentWidth = chart.current.clientWidth;
    const parentHeight = chart.current.clientHeight;

    // 메르카토르 투영법 설정
    const projection = d3.geoMercator().scale(1).translate([0, 0]);
    const path = d3.geoPath().projection(projection);

    const featureData = feature(sung, sung.objects['skorea_municipalities_2018_geo']);

    const filteredFeatures = featureData.features.filter((feature) =>
      feature.properties.name.startsWith('서울시')
    );

    // 필터링된 데이터의 경계를 계산합니다.
    const bounds = path.bounds({ type: 'FeatureCollection', features: filteredFeatures });

    // svg의 크기에 따른 지도의 크기와 위치값을 설정합니다.
    const dx = bounds[1][0] - bounds[0][0];
    const dy = bounds[1][1] - bounds[0][1];
    const x = (bounds[0][0] + bounds[1][0]) / 2;
    const y = (bounds[0][1] + bounds[1][1]) / 2;
    const scale = 0.9 / Math.max(dx / parentWidth, dy / parentHeight);
    const translate = [parentWidth / 2 - scale * x, parentHeight / 2 - scale * y];

    projection.scale(scale).translate(translate);

    // svg를 만들고
    const svg = d3.select(chart.current).append('svg').attr('width', parentWidth).attr('height', parentHeight);

    const mapLayer = svg.append('g');

    // 필터링된 지역만을 지도에 표시합니다.
    mapLayer
      .selectAll('path')
      .data(filteredFeatures)
      .enter()
      .append('path')
      .attr('d', path)
      .style('fill', '#666');
  };

  useEffect(() => {
    printD3();
  }, []);

  return <div ref={chart} style={{ width: '100%', height: '100%' }}></div>;
};

export default SungMap;

import * as React from 'react';
import { useEffect, useRef } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import korea from '../mapData/korea-topo.json';

const featureData = feature(korea, korea.objects['korea-topo']);

const KoreaMap = () => {
  const chart = useRef(null);

  const printD3 = () => {
    const width = window.innerWidth; // 페이지 너비로 설정
    const height = window.innerHeight; // 페이지 높이로 설정

    const projection = d3.geoMercator().scale(1).translate([0, 0]);
    const path = d3.geoPath().projection(projection);
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

    // 기본 지도 스타일 (하늘색)
    mapLayer
      .selectAll('path')
      .data(featureData.features)
      .enter()
      .append('path')
      .attr('d', path)
      .style('fill', 'skyblue') // 기본 색상 설정
      .style('transition', 'transform 0.2s'); // 트랜지션 효과 추가

    // 팝업을 위한 그룹 엘리먼트 추가
    const popupGroup = svg.append('g').style('display', 'none');

    // 호버 이벤트 처리
    mapLayer
      .selectAll('path')
      .on('mouseover', function (event, d) {
        d3.select(this).style('fill', 'blue').attr('transform', 'translate(0, -5)'); // 위로 약간 이동

        // 팝업 표시
        popupGroup.style('display', 'block');
        popupGroup.selectAll('*').remove(); // 팝업 내용 초기화

        // 팝업 배경
        popupGroup
          .append('rect')
          .attr('x', path.centroid(d)[0] - 50) // 팝업 가로 위치 조정
          .attr('y', path.centroid(d)[1] - 30) // 팝업 세로 위치 조정
          .attr('width', 100) // 팝업 가로 크기
          .attr('height', 60) // 팝업 세로 크기
          .attr('rx', 5) // 팝업 둥근 모서리 반지름
          .style('fill', 'white') // 팝업 배경색
          .style('stroke', 'blue') // 팝업 테두리 색
          .style('stroke-width', 2); // 팝업 테두리 두께

        // 팝업 텍스트
        popupGroup
          .append('text')
          .attr('x', path.centroid(d)[0]) // 텍스트 가로 위치 조정
          .attr('y', path.centroid(d)[1]) // 텍스트 세로 위치 조정
          .attr('text-anchor', 'middle') // 가운데 정렬
          .attr('alignment-baseline', 'middle') // 가운데 정렬
          .text(d.properties.CTP_KOR_NM) // 지역 이름 데이터 필드 이름으로 변경
          .style('fill', 'black') // 텍스트 색상 설정
          .style('font-size', '12px'); // 텍스트 크기 설정
      })
      .on('mouseout', function (event, d) {
        d3.select(this).style('fill', 'skyblue').attr('transform', 'translate(0, 0)'); // 원래 위치로 복원

        // 팝업 숨기기
        popupGroup.style('display', 'none');
      });
  };

  useEffect(() => {
    printD3();
  }, []);

  return <div ref={chart} style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}></div>;
};

export default KoreaMap;

import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import mapogu from '../mapData/mapo.json';
import '../css/Home.css';
import Swal from "sweetalert2";
import { initializeApp } from "firebase/app";
import { getAnalytics } from "firebase/analytics";

const firebaseConfig = {
  apiKey: "AIzaSyCe8s_k1g8-g2qRvgv3i0lJwFuVLRAMJtU",
  authDomain: "ssafy9th-19ed3.firebaseapp.com",
  databaseURL: "https://ssafy9th-19ed3-default-rtdb.firebaseio.com",
  projectId: "ssafy9th-19ed3",
  storageBucket: "ssafy9th-19ed3.appspot.com",
  messagingSenderId: "219156140787",
  appId: "1:219156140787:web:007d653d7bae027a1f9d86",
  measurementId: "G-M7W3LJDE33"
}

const app = initializeApp(firebaseConfig);

const analytics = getAnalytics(app);

const GuMap = () => {
  const chart = useRef(null);
  const featureData = feature(mapogu, mapogu.objects['mapo']);
  console.log(analytics)

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
    const scale = 1.5 / Math.max(dx / width, dy / height);
    const translate = [width / 2 - scale * x + 80, height / 2 - scale * y + 235];
    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);
    const mapLayer = svg.append('g');

    mapLayer
    .selectAll('path')
    .data(featureData.features)
    .enter()
    .append('path')
    .filter((d) => d.properties.temp === '마포구 상암동')
    .attr('d', path)
    .style('transition', 'transform 0.2s');
  }, []);

  return (
    <div className='gumap' ref={chart}></div>
  );
};

export default GuMap;

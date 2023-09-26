import React from 'react';
import KoreaMap from './KoreaMap';
import SeoulMap from './SeoulMap';
import MainList from './MainList';
import '../css/Home.css'

const Home = () => {
  return (
    <div className="combined-maps-container">
      <div className="map-container">
        <SeoulMap className="seoul-map" />
        <KoreaMap className="korea-map" />
        <MainList className="main-list" />
      </div>
    </div>
  );
};

export default Home;

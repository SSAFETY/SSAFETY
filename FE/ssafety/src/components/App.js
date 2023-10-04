import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './NavBar';
import Violation from './Violation';
import MainPage from './MainPage';
import Seoul from './Seoul';
import BusMap from './BusMap';
import Live from './Cctv';

function App() {
  return (
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/" element={<MainPage />} />
          <Route path="/violation" element={<Violation />} />
          <Route path="/seoul" element={<Seoul />} />
          <Route path="/bus" element={<BusMap />} />
          <Route path="/cctv" element={<Live/>} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;

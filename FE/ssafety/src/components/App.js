import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './NavBar';
import Statistics from './Statistics';
import Violation from './Violation';
import DetailGuMap from './DetailGuMap'
import MainPage from './MainPage';
import SeoulMap from './SeoulMap';
import Seoul from './Seoul';

function App() {
  return (
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/" element={<MainPage />} />
          <Route path="/violation" element={<Violation />} />
          <Route path="/statistics" element={<Statistics />} />
          <Route path="/Seoul" element={<Seoul />} />
          <Route path="/detailsi/detailgu" element={<DetailGuMap />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;

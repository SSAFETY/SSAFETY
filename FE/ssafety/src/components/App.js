import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './NavBar';
import Home from './Home';
import Road from './Road';
import Statistics from './Statistics';
import LiveVideo from './LiveVideo';
import Violation from './Violation';
import DetailSiMap from './DetailSiMap';
import DetailGuMap from './DetailGuMap'

function App() {
  return (
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/violation" element={<Violation />} />
          <Route path="/statistics" element={<Statistics />} />
          <Route path="/livevideo" element={<LiveVideo />} />
          <Route path="/detailsi" element={<DetailSiMap />} />
          <Route path="/detailsi/detailgu" element={<DetailGuMap />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './NavBar';
import Home from './Home';
import Road from './Road';
import Statistics from './Statistics';
import LiveVideo from './LiveVideo';
import Violation from './Violation';

function App() {
  return (
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/violation" element={<Violation />} />
          <Route path="/statistics" element={<Statistics />} />
          <Route path="/road" element={<Road />} />
          <Route path="/livevideo" element={<LiveVideo />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;

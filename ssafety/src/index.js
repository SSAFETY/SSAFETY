// App.js
import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './components/NavBar';
import Home from './components/Home';
import Road from './components/Road';
import Statistics from './components/Statistics';
import LiveVideo from './components/LiveVideo';
import Violation from './components/Violation';

function App() {
  return (
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/" exact component={Home} />
          <Route path="/violation" component={Violation} />
          <Route path="/statistics" component={Statistics} />
          <Route path="/road" component={Road} />
          <Route path="/livevideo" component={LiveVideo} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;

import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavBar from './NavBar';
import Violation from './Violation';
import Seoul from './Seoul';
import BusMap from './BusMap';
import { createTheme, ThemeProvider } from '@mui/material';

const theme = createTheme({
  typography: {
    fontFamily: "Main"
  }
})

function App() {
  return (
    <ThemeProvider theme={theme}>
    <Router>
      <div>
        <NavBar />
        <Routes>
          <Route path="/violation" element={<Violation />} />
          <Route path="/" element={<Seoul />} />
          <Route path="/bus" element={<BusMap />} />
        </Routes>
      </div>
    </Router>
    </ThemeProvider>
  );
}

export default App;

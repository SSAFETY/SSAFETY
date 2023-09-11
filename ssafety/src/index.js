import React from 'react';
import ReactDOM from 'react-dom';
import Maps from './Maps';

import './index.css';
import 'leaflet/dist/leaflet.css'

ReactDOM.render(
  <React.StrictMode>
    <Maps />
  </React.StrictMode>,
  document.getElementById('root')
);
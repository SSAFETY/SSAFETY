// NavBar.js
import React from 'react';
import { Link } from 'react-router-dom';
import '../css/Nav.css';

const NavBar = () => {
  return (
    <div>
      <div className='navbar'>
          <Link className='navbarMenu' to={'/'}>홈</Link>
          <Link className='navbarMenu' to={'/violation'}>위반 목록</Link>
          <Link className='navbarMenu' to={'/bus'}>버스</Link>
          <Link className='navbarMenu' to={'/live'}>live</Link>
        </div>
    </div>
  );
};

export default NavBar;

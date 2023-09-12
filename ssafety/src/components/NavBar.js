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
          <Link className='navbarMenu' to={'/statistics'}>통계</Link>
          <Link className='navbarMenu' to={'road'}>경로 설정</Link>
          <Link className='navbarMenu' to={'livevideo'}>실시간 영상</Link>
        </div>
    </div>
  );
};

export default NavBar;

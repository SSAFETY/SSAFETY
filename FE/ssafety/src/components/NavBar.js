import React from 'react';
import { Link } from 'react-router-dom';
import { AppBar, Toolbar, Typography, Button } from '@mui/material';
import '../css/Nav.css';

const NavBar = () => {
  return (
    <AppBar className="navbar">
      <Toolbar>
        <Typography variant="h6" component="div" className="logo">
          <img src="https://a102.s3.ap-northeast-2.amazonaws.com/%EB%A1%9C%EA%B3%A0%2B%EB%84%A4%EC%9E%84.png" alt="로고" className="logo-image" />
        </Typography>
        <div className="nav-links">
          <Button color="inherit" component={Link} to="/">
            홈
          </Button>
          <Button color="inherit" component={Link} to="/violation">
            위반 목록
          </Button>
          <Button color="inherit" component={Link} to="/bus">
            버스
          </Button>
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default NavBar;

import React from 'react';
import { Modal, Box, Button, Typography } from '@mui/material';

const CustomModal = ({ isOpen, closeModal, data }) => {
  if (!isOpen) return null;
  const formatCreationTime = (creationTimeArray) => {
    const [year, month, day, hour, minute] = creationTimeArray;
    const formattedTime = `${year}년 ${month}월 ${day}일 ${hour}시 ${minute}분`;
    return formattedTime;
  };

  return (
    <Modal
      open={isOpen}
      onClose={closeModal}
      aria-labelledby="modal-modal-title"
      aria-describedby="modal-modal-description"
    >
      <div
        style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          background: 'rgba(0, 0, 0, 0.5)', // 블러 처리를 위한 반투명 배경
          backdropFilter: 'blur(5px)', // 배경을 블러 처리
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
        }}
      >
        <Box
          sx={{
            width: 1000,
            height: 600,
            bgcolor: 'white',
            borderRadius: '16px',
            boxShadow: '0px 0px 10px rgba(0, 0, 0, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            padding: '16px',
          }}
        >
          <Button
            sx={{
              position: 'absolute',
              top: 0,
              right: 0,
              borderRadius: '16px 0 0 0',
            }}
            onClick={closeModal}
          >
            Close
          </Button>
          <Typography variant="h6" id="modal-modal-title" sx={{ marginBottom: '24px', color: 'skyblue', fontSize: '24px' }}>
            상세 정보
          </Typography>
          <div sx={{ marginBottom: '24px', marginTop: '16px' }}>
            <Typography variant="body2" id="modal-modal-description" sx={{ fontWeight: 'bold', fontSize: '18px' }}>
              시 (도): {data.city}
            </Typography>
          </div>
          <div sx={{ marginBottom: '24px' }}>
            <Typography variant="body2" id="modal-modal-description" sx={{ fontWeight: 'bold', fontSize: '18px' }}>
              구: {data.depth3}
            </Typography>
          </div>
          <div sx={{ marginBottom: '24px' }}>
            <Typography variant="body2" id="modal-modal-description" sx={{ fontWeight: 'bold', fontSize: '18px' }}>
              위반 종류: {data.aiResult}
            </Typography>
          </div>
          <div sx={{ marginBottom: '24px' }}>
            <Typography variant="body2" id="modal-modal-description" sx={{ fontWeight: 'bold', fontSize: '18px' }}>
              위반 일시: {formatCreationTime(data.creationTime)}
            </Typography>
          </div>
          <div className="video-container">
            <video controls>
              <source src={data.videoUrl} type="video/mp4" />
              Your browser does not support the video tag.
            </video>
          </div>
        </Box>
      </div>
    </Modal>
  );
};

export default CustomModal;

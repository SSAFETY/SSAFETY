import React from 'react'

const TfVideoPlayer = ({videoUrl}) => {
return (
    <div>
        <video controls width="640" height="360">
            <source src={videoUrl} type="video/mp4">
            </source>
        </video>
    </div>
);
};

export default TfVideoPlayer;

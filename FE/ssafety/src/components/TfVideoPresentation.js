import React, { useEffect, useState } from 'react';
import TfVideoPlayer from './TfVideoPlayer';
import axios from 'axios'

const TfVideoPresentation = () => {
    const [videoUrl, setVideoUrl] = useState('');

    useEffect(() => {
        axios.get('https://openapi.its.go.kr:9443/cctvInfo?apiKey=0db19b82b4d84ef3846c147df0b0028d&type=ex&cctvType=1&minX=127.100000&maxX=128.890000&minY=34.100000&maxY=39.100000&getType=json')
        .then((data) =>{
            console.log(data);
            setVideoUrl(data[0].cctvurl)
        })
        .catch((error) =>
        {
            console.error('Video Fetch Error', error)
        });
    }, []);
        {videoUrl && <TfVideoPlayer videoUrl={videoUrl}/>}
    return (
        <div>
            <h1>Video Show</h1>
        </div>
    )
}

export default TfVideoPresentation;
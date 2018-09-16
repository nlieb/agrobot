import React, {Component} from 'react';

import {db} from '../firebase';

class StreamingBox extends Component {

  constructor(props) {
    super(props);
    
    this.state = {
      rawImageData: '',
    }

    this.frameUpdateCallback = this.frameUpdateCallback.bind(this);

    db.onVideoFrame(0, this.frameUpdateCallback);
  }

  frameUpdateCallback(snap){
    var data = snap.val();
    this.setState({
        rawImageData: data.image,
    });
  }

  render() {
    return <img height={'100%'} width={'100%'} src={this.state.rawImageData} alt="Video Stream" />
  }
}

export default StreamingBox
import React, {Component} from 'react';

import ReactMapGL from 'react-map-gl';
import {SVGOverlay} from 'react-map-gl';
import {Button, Header, Icon} from 'semantic-ui-react'

import {db} from '../firebase';

class ArgoBotMap extends Component {

  constructor(props) {
    super(props);
    
    this.state = {
      viewport:{
        width: this.props.width || window.innerWidth,
        height: this.props.height || window.innerHeight,
        latitude: 43.4727,
        longitude: -80.5398,
        zoom: 18,
        mapStyle: 'mapbox://styles/nathandor/cjm3o5nus1bda2slei1miqxva',
      },
      robotlat: 43.4727,
      robotlng: -80.5398,
    }

    this.locationUpdateCallback = this.locationUpdateCallback.bind(this);
    this.redraw = this.redraw.bind(this);

    db.onLocationUpdate(0, this.locationUpdateCallback);
  }

  locationUpdateCallback(snap){
    var data = snap.val();
    this.setState({
      robotlat: data.lat,
      robotlng: data.lng,
    });
  }

  redraw({project}) {
    const [cx, cy] = project([this.state.robotlng, this.state.robotlat]);
    return <circle cx={cx} cy={cy} r={5} fill="blue" />;
  }

  render() {

    return <ReactMapGL
      {...this.state.viewport}
      onViewportChange={(viewport) => this.setState({viewport})}
      mapboxApiAccessToken={process.env.REACT_APP_MAPBOX}>
        <SVGOverlay redraw={this.redraw} />
      </ReactMapGL>
  }
}

export default ArgoBotMap
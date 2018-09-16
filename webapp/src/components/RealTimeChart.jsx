import React, {Component} from 'react';

import { VictoryChart, VictoryArea } from 'victory';

import {db} from '../firebase';

class RealTimeChart extends Component {

  constructor() {
    super();
    this.state = {
      data: [ {x: new Date() - 0.00001, y: 0}, {x: new Date(), y: 0} ],
    };

    this.onROIUpdateCallback = this.onROIUpdateCallback.bind(this);

    db.onROIUpdate(0, this.onROIUpdateCallback);
  }

  onROIUpdateCallback(snap){
    var localdata = this.state.data;
    const data = snap.val();
    const key = Object.keys(data)[0];
    localdata.push({x: new Date(data[key].time * 1000), y: data[key].num});
    if (localdata.length > 50){
      localdata.shift();
    }
    this.setState({
      data: localdata,
    });
  }

  render() {
    return <div style={{marginTop:-50}}>
      <VictoryChart scale={{ x: "time" }}>
        <VictoryArea style={{ data: { stroke: "2176AE", fill:"57B8FF", fillOpacity:0.3 }}} data={this.state.data}/>
      </VictoryChart>
    </div>
  }
}

export default RealTimeChart
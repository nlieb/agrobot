import React, {Component} from 'react';

import { VictoryChart, VictoryLine, VictoryZoomContainer } from 'victory';

import last from 'lodash/last'

const y = x => Math.sin(x / 10);

class RealTimeChart extends Component {

  constructor() {
    super();
    this.state = {
      data: [ {x: 0, y: 0.3} ]
    };

    setInterval(() => {
      const {data} = this.state;
      const x = last(data).x + 1;
      data.push({x, y: y(x)});
      this.setState({data});
    }, 20);
  }

  render() {
    return <div style={{
      marginTop: -40
    }}>
      <VictoryChart containerComponent={< VictoryZoomContainer dimension = "x" />}>
        <VictoryLine data={this.state.data}/>
      </VictoryChart>
    </div>
  }
}

export default RealTimeChart
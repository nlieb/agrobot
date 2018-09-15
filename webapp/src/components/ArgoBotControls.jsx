import React, { Component } from 'react';

import {Button, Header, Icon} from 'semantic-ui-react'

import { db } from '../firebase';

class ArgoBotControls extends Component {

  constructor(props) {
    super(props);

    this.state = { 
      running: false
    };

    this.runningStatusCallback = this.runningStatusCallback.bind(this);
    this.editRunningStatus = this.editRunningStatus.bind(this);

    db.onRunningStatusUpdate(0, this.runningStatusCallback);

  }

  runningStatusCallback(snap){
    var data = snap.val();
      this.setState( {running: data.running} );
  }

  editRunningStatus(value){
    db.updateRunningStatus(0, value)
  }
  
    render() {


      let runningStatus = 
      <Header as='h2' color='red'>
        <Icon name='stop' color='red' />
          <Header.Content>
            ArgoBot is not running
          <Header.Subheader>Press Start to startup ArgoBot</Header.Subheader>
        </Header.Content>
      </Header>

      if (this.state.running){
        runningStatus = 
          <Header as='h2' color='green'>
            <Icon name='play' color='green'/>
              <Header.Content>
                ArgoBot is Running!
              <Header.Subheader>Use the stop button to pause ArgoBot</Header.Subheader>
            </Header.Content>
          </Header>
      }


      return <div style={{padding:10}}>
        {runningStatus}
        <Button color='green' disabled={this.state.running} onClick={() => this.editRunningStatus(true)}>Start</Button>
        <Button color='red' disabled={this.state.running === false} onClick={() => this.editRunningStatus(false)}>
          Stop
        </Button>
      </div>
    }
}

export default ArgoBotControls
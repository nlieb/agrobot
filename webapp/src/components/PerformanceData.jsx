import React, {Component} from 'react';

import {db} from '../firebase';

import {Statistic, Grid} from 'semantic-ui-react'

class PerformanceData extends Component {

    constructor(props) {
        super(props);

        this.state = {
            weedsPerSqrMeter: 0,
            totalWeedsPicked: 0,
            distanceTraveled: 0,
            pesticidesSaved: 0,
            areaMonitored: 0,
        }

        this.statsUpdateCallback = this
            .statsUpdateCallback
            .bind(this);

        db.onStatsUpdate(0, this.statsUpdateCallback)
    }

    statsUpdateCallback(snap) {
        var data = snap.val();
        var distance = data.distance;
        var totalWeeds = data.weedsPicked;
        const widthOfRobot = 0.27;
        var areaMonitored = widthOfRobot * distance;
        var totalWeedsPerSqrMeter = totalWeeds / areaMonitored;

        // Based on 1 qt / A or 0.946353/4046.86
        // 0.946353/4046.86 = 0.00023384871

        var pesticidesSaved = 0.00023384871 * areaMonitored;
        
        this.setState({
            weedsPerSqrMeter: totalWeedsPerSqrMeter.toFixed(2),
            totalWeedsPicked: totalWeeds,
            distanceTraveled: distance.toFixed(2),
            pesticidesSaved: pesticidesSaved.toFixed(2),
            areaMonitored: areaMonitored.toFixed(2),
        });
    }

    render() {
        return <div style={{marginLeft:10}}>
            <Grid divided='vertically'>
                <Grid.Row columns={2}>
                    <Grid.Column>
                        <Statistic.Group horizontal>
                            <Statistic>
                                <Statistic.Value>{this.state.weedsPerSqrMeter}</Statistic.Value>
                                <Statistic.Label>Weeds Per Square Meter</Statistic.Label>
                            </Statistic>
                            <Statistic>
                                <Statistic.Value>{this.state.pesticidesSaved}</Statistic.Value>
                                <Statistic.Label>Pesticides Saved (L)</Statistic.Label>
                            </Statistic>
                            <Statistic>
                                <Statistic.Value>{this.state.totalWeedsPicked}</Statistic.Value>
                                <Statistic.Label>Total Weeds Picked</Statistic.Label>
                            </Statistic>
                        </Statistic.Group>
                    </Grid.Column>
                    <Grid.Column>
                        <Statistic.Group horizontal>
                            <Statistic>
                                <Statistic.Value>{this.state.distanceTraveled}</Statistic.Value>
                                <Statistic.Label>Distance Traveled</Statistic.Label>
                            </Statistic>
                            <Statistic>
                                <Statistic.Value>{this.state.areaMonitored}</Statistic.Value>
                                <Statistic.Label>Area Monitored</Statistic.Label>
                            </Statistic>
                        </Statistic.Group>
                    </Grid.Column>
                </Grid.Row>
            </Grid>
        </div>
    }
}

export default PerformanceData
import React from 'react';

import { Mosaic, MosaicWindow} from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css'

import '@blueprintjs/core/dist/blueprint.css';
import withAuthorization from './withAuthorization';

import DashboardNav from './DashboardNav';
import ArgoBotControls from './ArgoBotControls'
import RealTimeChart from './RealTimeChart'
import ArgoBotMap from './ArgoBotMap'

const WINDOW_MAP = {
  a: {
    'name': 'Left Window',
    'content': <p>YAS</p>
  },
  b: {
    'name': 'User Controls',
    'content': <DashboardNav />
  },
  c: {
    'name': 'AgroBot Map',
    'content': <ArgoBotMap />
  },
  d: {
    'name': 'AgroBot Controls',
    'content': <ArgoBotControls />
  },
  e: {
    'name': 'Live Stream',
    'content': <p>Stream here</p>
  },
  f: {
    'name': 'Graph 1',
    'content': <RealTimeChart />
  },
};

const ViewIdMosaic = Mosaic;
const ViewIdMosaicWindow = MosaicWindow;

const DashboardPage = () =>
  <ViewIdMosaic
    renderTile={(id, path) => (
      <ViewIdMosaicWindow path={path} title={WINDOW_MAP[id].name}>
        {WINDOW_MAP[id].content}
      </ViewIdMosaicWindow>
    )}
    initialValue={{
      direction: 'row',
      first: {
        direction: 'column',
        first: 'e',
        second: 'f'
      },
      second: {
        direction: 'column',
        first: {
          direction: 'column',
          first: {
            direction: 'row',
            first: 'b',
            second: 'd',
          },
          second: 'a',
          splitPercentage: 40,
        },
        second: 'c',
      },
    }}
  />

const authCondition = (authUser) => !!authUser;

export default withAuthorization(authCondition)(DashboardPage);
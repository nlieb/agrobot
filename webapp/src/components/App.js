import React from 'react';
import {
  BrowserRouter as Router,
  Route,
} from 'react-router-dom';

import withAuthentication from './withAuthentication';

import Navigation from './Navigation';
import SignUpPage from './SignUp';
import SignInPage from './SignIn';
import PasswordForgetPage from './PasswordForget';
import DashboardPage from './Dashboard';
import AccountPage from './Account';
import './App.css'

import 'mapbox-gl/dist/mapbox-gl.css';

import * as routes from '../constants/routes';

const App = () =>
  <Router>
    <div id='app'>
      <Navigation />

      <Route exact path={routes.DASHBOARD} component={DashboardPage} />
      <Route exact path={routes.SIGN_UP} component={SignUpPage} />
      <Route exact path={routes.SIGN_IN} component={SignInPage} />
      <Route exact path={routes.PASSWORD_FORGET} component={PasswordForgetPage} />
      <Route exact path={routes.ACCOUNT} component={AccountPage} />
    </div>
  </Router>

export default withAuthentication(App);
import React from 'react';

import { Container, Button } from 'semantic-ui-react'

import AuthUserContext from './AuthUserContext';
import { PasswordForgetForm } from './PasswordForget';
import PasswordChangeForm from './PasswordChange';

import withAuthorization from './withAuthorization';

import { Link } from 'react-router-dom';

import * as routes from '../constants/routes';

const AccountPage = () =>
  <AuthUserContext.Consumer>
    {authUser =>
      <div>
        <Container>
          <h1>Account: {authUser.email}</h1>
          <PasswordForgetForm />
          <br></br>
          <PasswordChangeForm />
          <br></br>
          <Button as={ Link } to={routes.DASHBOARD}>
            Back to Dashboard
          </Button>
        </Container>
      </div>
    }
  </AuthUserContext.Consumer>


const authCondition = (authUser) => !!authUser;

export default withAuthorization(authCondition)(AccountPage);
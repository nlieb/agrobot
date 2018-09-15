import React, { Component } from 'react';
import { withRouter } from 'react-router-dom';
import { Button, Form, Message, Container } from 'semantic-ui-react'

import { SignUpLink } from './SignUp';
import { PasswordForgetLink } from './PasswordForget';
import { auth } from '../firebase';
import * as routes from '../constants/routes';

const SignInPage = ({ history }) =>
  <div>
    <Container>
        <h1>Sign In</h1>
        <SignInForm history={history} />
        <PasswordForgetLink />
        <SignUpLink />
    </Container>
  </div>

const byPropKey = (propertyName, value) => () => ({
  [propertyName]: value,
});

const INITIAL_STATE = {
  email: '',
  password: '',
  error: null,
};

class SignInForm extends Component {
  constructor(props) {
    super(props);

    this.state = { ...INITIAL_STATE };
  }

  onSubmit = (event) => {
    const {
      email,
      password,
    } = this.state;

    const {
      history,
    } = this.props;

    auth.doSignInWithEmailAndPassword(email, password)
      .then(() => {
        this.setState({ ...INITIAL_STATE });
        history.push(routes.DASHBOARD);
      })
      .catch(error => {
        this.setState(byPropKey('error', error));
      });

    event.preventDefault();
  }

  render() {
    const {
      email,
      password,
      error,
    } = this.state;

    const isInvalid =
      password === '' ||
      email === '';

    return (
      <Form onSubmit={this.onSubmit}>
        <Form.Field>
            <input
                value={email}
                onChange={event => this.setState(byPropKey('email', event.target.value))}
                type="text"
                placeholder="Email Address"
            />
        </Form.Field>
        <Form.Field>
            <input
            value={password}
            onChange={event => this.setState(byPropKey('password', event.target.value))}
            type="password"
            placeholder="Password"
            />
        </Form.Field>
        <Button disabled={isInvalid} type='submit'>Sign In</Button>

        { error && <Message
                        error
                        header='Error'
                        content={error.message}
                    /> }
      </Form>
    );
  }
}

export default withRouter(SignInPage);

export {
  SignInForm,
};
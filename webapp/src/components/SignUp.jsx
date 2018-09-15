import React, { Component } from 'react';
import { Link, withRouter } from 'react-router-dom';
import { Button, Form, Message, Container } from 'semantic-ui-react'

import * as routes from '../constants/routes';
import { auth, db } from '../firebase';

const SignUpPage = ({ history }) =>
  <div>
    <Container>
        <h1>Sign Up</h1>
        <SignUpForm history={history} />
    </Container>
  </div>

const byPropKey = (propertyName, value) => () => ({
    [propertyName]: value,
  });

const INITIAL_STATE = {
    username: '',
    email: '',
    passwordOne: '',
    passwordTwo: '',
    error: null,
  };

class SignUpForm extends Component {
  constructor(props) {
    super(props);
    this.state = { ...INITIAL_STATE };
  }

  onSubmit = (event) => {
    const {
        username,
        email,
        passwordOne,
      } = this.state;

      const {
        history,
      } = this.props;
  
      auth.doCreateUserWithEmailAndPassword(email, passwordOne)
        .then(authUser => {
            db.doCreateUser(authUser.user.uid, username, email)
            .then(() => {
              this.setState({ ...INITIAL_STATE });
              history.push(routes.DASHBOARD);
            })
            .catch(error => {
              this.setState(byPropKey('error', error));
            });
        })
        .catch(error => {
          this.setState(byPropKey('error', error));
        });
  
      event.preventDefault();
  }

  render() {
    const {
        username,
        email,
        passwordOne,
        passwordTwo,
        error,
      } = this.state;

    const isInvalid =
      passwordOne !== passwordTwo ||
      passwordOne === '' ||
      email === '' ||
      username === '';
    return (
      <Form onSubmit={this.onSubmit}>
        <Form.Field>
        <input
          value={username}
          onChange={event => this.setState(byPropKey('username', event.target.value))}
          type="text"
          placeholder="Full Name"
        /> 
        </Form.Field>
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
            value={passwordOne}
            onChange={event => this.setState(byPropKey('passwordOne', event.target.value))}
            type="password"
            placeholder="Password"
            />
        </Form.Field>
        <Form.Field>
        <input
          value={passwordTwo}
          onChange={event => this.setState(byPropKey('passwordTwo', event.target.value))}
          type="password"
          placeholder="Confirm Password"
        />
        </Form.Field>
        <Button disabled={isInvalid} type="submit">
          Sign Up
        </Button>

        { error && <Message
                        error
                        header='Error'
                        content={error.message}
                    /> }
      </Form>
    );
  }
}

const SignUpLink = () =>
  <p>
    Don't have an account?
    {' '}
    <Link to={routes.SIGN_UP}>Sign Up</Link>
  </p>

export default withRouter(SignUpPage);

export {
  SignUpForm,
  SignUpLink,
};
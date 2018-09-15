import React from 'react';
import {Button} from 'semantic-ui-react'

import {auth} from '../firebase';

const SignOutButton = () => <Button type="button" color='red' onClick={auth.doSignOut}>
  Sign Out
</Button>

export default SignOutButton;
import React from 'react';

import * as routes from '../constants/routes';

import SignOutButton from './SignOut';

import { Link } from 'react-router-dom';

import { List, Icon } from 'semantic-ui-react'

import './DashboardNav.css'

const DashboardNav = () => 
    <div className="dashboardNav">
        <List>
            <List.Item as={Link} to={routes.ACCOUNT}>
                <Icon name='user'/>
                <List.Content>
                    <List.Header>Account</List.Header>
                    <List.Description>
                        View your account details
                    </List.Description>
                </List.Content>
            </List.Item>
            <List.Item>
                <SignOutButton />
            </List.Item>
        </List>
    </div>

export default DashboardNav;
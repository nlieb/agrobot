import { db } from './firebase';

// User API

export const doCreateUser = (id, username, email) =>
  db.ref(`users/${id}`).set({
    username,
    email,
  });

export const onceGetUsers = () =>
  db.ref('users').once('value');

export const onLocationUpdate = (id, callback) =>
  db.ref(`locations/${id}`).on('value', callback);

export const onRunningStatusUpdate = (id, callback) =>
  db.ref(`robot/${id}`).on('value', callback);

export const updateRunningStatus = (id, value) =>
  db.ref(`robot/${id}`).update({
    running: value
  });


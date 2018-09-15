import firebase from 'firebase/app';
import 'firebase/auth';
import 'firebase/database';

const config = {
    apiKey: "AIzaSyASodGF-KTGvhM5OaLHAr6boxzKGVGFEM4",
    authDomain: "agribot-2019.firebaseapp.com",
    databaseURL: "https://agribot-2019.firebaseio.com",
    projectId: "agribot-2019",
    storageBucket: "agribot-2019.appspot.com",
    messagingSenderId: "418078781881"
};

if (!firebase.apps.length) {
  firebase.initializeApp(config);
}

const auth = firebase.auth();
const db = firebase.database();

export {
    db,
    auth,
};
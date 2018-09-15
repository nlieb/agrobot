import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import App from './components/App';
import 'semantic-ui-css/semantic.min.css';
import registerServiceWorker from './registerServiceWorker';

require('dotenv').config()

ReactDOM.render(<App />, document.getElementById('root'));
registerServiceWorker();
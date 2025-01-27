from flask import Flask, request
from flask_cors import CORS

from flask_socketio import SocketIO, emit

import os

# isolated to prevent circular dependencies in the codebase
CLIENT_URL = os.environ.get("CLIENT_URL")

app = Flask(__name__)
cors = CORS(app)
io = SocketIO(app, cors_allowed_origins="http://localhost:3000")
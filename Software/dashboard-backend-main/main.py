from flask import request

from flask_socketio import emit

from api.researchApi import app as researchApi
from api.clickApi import app as clickApi
from api.clickerApi import app as clickerApi
from api.beaconApi import app as beaconApi
from api.locationApi import app as locationApi
from api.mapApi import app as mapApi

from util.setup import setup

from util.flask import app, io

app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024

app.register_blueprint(researchApi, url_prefix="/api/research")
app.register_blueprint(clickApi, url_prefix="/api/click")
app.register_blueprint(clickerApi, url_prefix="/api/clicker")
app.register_blueprint(beaconApi, url_prefix="/api/beacon")
app.register_blueprint(locationApi, url_prefix="/api/location")
app.register_blueprint(mapApi, url_prefix="/api/map")



@io.on("connect")
def connect():
    print(f"Client connected: {request.sid}")


@io.on("disconnect")
def disconnect():
    print(f"Client disconnected: {request.sid}")

# example of how to handle data coming from websockets
# @io.on("data")
# def handle_data(data):
#     io.emit("data", {'socketId': request.sid, 'data': data}, broadcast=True)

if (__name__) == "__main__":
    setup() # initializes the database if it doesnt exist yet
    app.run()

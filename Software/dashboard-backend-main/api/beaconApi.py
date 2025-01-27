from flask import Blueprint, request
from util.session import Session
from models.db.beacon import Beacon as DbBeacon
from models.api.beacon import Beacon as ApiBeacon

from functions.json import json

from sqlalchemy import func, select, or_, and_

from functions.beaconFunctions import getBeaconsForLocation


app = Blueprint("beacon", __name__)

@app.route("")
def getAllBeacons():
    session = Session()
    request = select(DbBeacon)
    arr = []
    print(request)
    for beacon in session.scalars(request):
        arr.append(beacon.value())
    return json(arr), 200

@app.route("/location/<locationId>")
def getAtLocation(locationId):
    request = getBeaconsForLocation(locationId)
    if (request == None):
        return json([]), 200
    return json(request), 200

@app.route("/<beacon_id>")
def getBeaconById(beacon_id):
    session = Session()
    request = session.query(DbBeacon).filter(DbBeacon.id == beacon_id).first()

    if (request == None):
        return json(DbBeacon().value()), 200
    return json(request.value()), 200

@app.route("/register/<beacon_uid>", methods=["POST"])
def registerBeacon(beacon_uid):
    jsonData = request.get_json()
    print(jsonData)
    
    location_id = jsonData["location_id"]
    x = jsonData["x"]
    y = jsonData["y"]
    session = Session()
    newBeacon = DbBeacon.new(beacon_uid, x, y, location_id)
    print(newBeacon.value())
    session.add(newBeacon)
    session.commit()

    return json(ApiBeacon.fromDb(newBeacon)), 200


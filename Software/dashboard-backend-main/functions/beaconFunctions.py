from util.session import Session
from models.db.beacon import Beacon
from models.api.beacon import Beacon as ApiBeacon
from models.db.clickBeaconConnections import ClickBeaconConnection as CBC
from sqlalchemy import select

from models.db.location import Location

def getAll():
    session = Session()
    request = select(Beacon)
    arr = []
    print(request)
    for beacon in session.scalars(request):
        arr.append(beacon.value())
    return arr

def getBeaconByUid(uid):
    session = Session()
    return session.query(Beacon).filter(Beacon.uid == uid).first()

def getBeacon(id):
    session = Session()
    return session.query(Beacon).filter(Beacon.id == id).first()

def getBeaconIdsByClickId(click_id):
    session = Session()
    beaconClicks = session.query(CBC).filter(CBC.click_id == click_id)
    beaconIds = []
    for beaconClick in beaconClicks:
        beaconIds.append(beaconClick.beacon_id)

    return beaconIds

def getBeaconsByClickId(click_id):
    session = Session()
    beaconClicks = session.query(CBC).filter(CBC.click_id == click_id).all()
    beaconIds = []
    for beaconClick in beaconClicks:
        beaconIds.append({
            "beacon": getBeacon(beaconClick.beacon_id),
            "distance": beaconClick.distance})

    return beaconIds

def getBeaconsForLocation(location_id):
    session = Session()
    req = session.query(Beacon).select_from(Beacon).join(Location, Beacon.location_id == Location.id).filter(Location.id == location_id).all()
    session.close()

    jsonArray = []
    for element in req:
        apiElement: ApiBeacon = ApiBeacon.fromDb(element)
        jsonArray.append(apiElement.toJSON())
    return jsonArray
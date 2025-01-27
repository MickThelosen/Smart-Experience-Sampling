from models.db.click import Click as DbClick
from sqlalchemy import select
from util.session import Session
from models.db.clicker import Clicker as DbClicker

from models.api.click import Click as ApiClick
from models.api.beacon import Beacon as ApiBeacon
from models.api.clicker import Clicker as ApiClicker

from functions.beaconFunctions import getBeaconsByClickId


def getAllClicks():
    session = Session()
    res = select(DbClick).order_by(DbClick.click_time)
    arr = []
    for click in session.scalars(res):
        tempClick: ApiClick = ApiClick.fromDb(click)
        tempClick.clicker = ApiClicker.fromDb(session.query(DbClicker).filter(DbClicker.id == click.clicker_id).first())
        beaconIds = getBeaconsByClickId(click.id)
        for dbBeacon in beaconIds:
            beaconArray = []
            beaconArray.append({
                "beacon": ApiBeacon.fromDb(dbBeacon["beacon"]),
                "distance": dbBeacon["distance"]})
            tempClick.click_beacons = beaconArray
            
        arr.append(tempClick.toJSON())
    session.close()
    return arr

def getAllClickValues():
    session = Session()
    res = select(DbClick).order_by(DbClick.click_time)
    arr = []
    for click in session.scalars(res):
        tempClick: ApiClick = ApiClick.fromDb(click)
        tempClick.clicker = ApiClicker.fromDb(session.query(DbClicker).filter(DbClicker.id == click.clicker_id).first())
        beaconIds = getBeaconsByClickId(click.id)
        beaconArray = []
        for dbBeacon in beaconIds:
            beaconArray.append({
                "beacon": ApiBeacon.fromDb(dbBeacon["beacon"]),
                "distance": dbBeacon["distance"]})
            tempClick.click_beacons = beaconArray
            
        arr.append(tempClick)
    session.close()
    return arr
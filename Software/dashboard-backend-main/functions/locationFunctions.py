from util.session import Session
from models.db.location import Location

def getDbLocation(id):
    session = Session()
    location = session.query(Location).filter(Location.id == id).first()
    session.close()
    return location

def getLocationsFromParent(parent_id):
    session = Session()
    locations = session.query(Location).filter(Location.parent_id == parent_id).all()
    session.close()
    return locations

def getLocation(id):
    print('test')
    session = Session()
    location = session.query(Location).filter(Location.id == id).first()
    session.close()
    return location
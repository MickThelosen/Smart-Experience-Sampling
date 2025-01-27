from util.session import Session
from models.db.research import Research
from models.db.location import Location
from models.api.research import Research as ApiResearch
from sqlalchemy import select


def getActiveResearchAtTime(time):
    session = Session()
    req = session.query(Research).filter(Research.start_time < time).filter(Research.end_time > time).first()
    session.close()
    return req

def getResearchesForLocation(locationId):
    session = Session()
    req = session.query(Research).select_from(Research).join(Location, Research.location_id == Location.id).filter(Location.id == locationId).all()
    session.close()

    jsonArray = []
    for element in req:
        apiElement: ApiResearch = ApiResearch.fromDb(element)
        jsonArray.append(apiElement.toJSON())
    return jsonArray
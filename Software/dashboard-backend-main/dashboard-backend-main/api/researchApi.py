from datetime import datetime
from flask import Blueprint, request
from sqlalchemy import func, select, or_, and_
from functions.json import json

from util.session import Session
from models.db.research import Research
from models.api.research import Research as ApiResearch

from functions.researchFunctions import getActiveResearchAtTime, getResearchesForLocation

app = Blueprint("research", __name__)

@app.route("/active")
def getActiveResearch():
    return getActiveResearchAtTime(datetime.isoformat(datetime.now()))

@app.route("/active/<time>")
def getActiveResearchAtTime(time):
    req = getActiveResearchAtTime(time)
    if (req == None):
        return json(Research()), 200
    return req, 200

@app.route("")
def getAll():
    session = Session()
    request = select(Research)
    arr = []
    print(request)
    for research in session.scalars(request):
        arr.append(research.value())
    session.close()

    return json(arr), 200

@app.route("/<locationId>")
def getAtLocation(locationId):
    request = getResearchesForLocation(locationId)
    if (request == None):
        return json([]), 200
    return json(request), 200


@app.route("", methods=["POST"])
def create():
    session = Session()
    jsonData = request.json
    print(jsonData)
    force = request.args.get('force')
    print(force)
    try:
        question = jsonData["question"]
        start_time_string: datetime = jsonData["startTime"]
        start_time = datetime.fromisoformat(start_time_string)
        end_time_string = jsonData["endTime"]
        location_id = jsonData["locationId"]
        end_time = datetime.fromisoformat(end_time_string)
    except KeyError:
        session.close()
        return "Model is invalid", 400
    
    except ValueError:
        session.close()
        return "Time is not in valid ISO format", 400
    
    numActiveResearches = session.query(
        func.count(Research.id)
        ).filter(and_(Research.end_time > start_time, Research.start_time < end_time))
    
    print(numActiveResearches.scalar())
    # TODO:
    # if another research is already happening at that time, cancel and tell user to use force flag
    # if force flag is used, allow
    
    research = Research.new(question, start_time, end_time, location_id)
    session.add(research)
    session.commit()

    apiResearch: ApiResearch = ApiResearch.fromDb(research)
    
    return apiResearch.toJSON(), 200
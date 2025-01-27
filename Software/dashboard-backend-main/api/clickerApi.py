from flask import Blueprint, request
from util.session import Session
from models.db.clicker import Clicker

from functions.json import json

from sqlalchemy import func, select, or_, and_


app = Blueprint("clicker", __name__)

@app.route("")
def getAllClickers():
    session = Session()
    request = select(Clicker)
    arr = []
    print(request)
    for clicker in session.scalars(request):
        arr.append(clicker)
    return json(arr), 200

@app.route("/location/<location_id>")

@app.route("/<clicker_id>")
def getClickerById(clicker_id):
    session = Session()
    request = session.query(Clicker).filter(Clicker.id == clicker_id).first()

    if (request == None):
        return json(Clicker()), 200
    return json(request), 200

@app.route("/register/<clicker_uid>", methods=["POST"])
def registerClicker(clicker_uid):
    session = Session()
    newClicker = Clicker.new(clicker_uid)
    session.add(newClicker)
    session.commit()

    return json(newClicker), 200
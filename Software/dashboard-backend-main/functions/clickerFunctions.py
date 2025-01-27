from util.session import Session
from models.db.clicker import Clicker


def getClickerByUid(uid):
    session = Session()
    return session.query(Clicker).filter(Clicker.uid == uid).first()
from models.db.beacon import Beacon as DbBeacon

import json

class Beacon():
    id: str
    uid: str
    x: int
    y: int

    def fromDb(dbBeacon: DbBeacon):
        beacon = Beacon()
        beacon.id = dbBeacon.id
        beacon.uid = dbBeacon.uid
        beacon.x = dbBeacon.x
        beacon.y = dbBeacon.y

        return beacon
    
    def toJSON(self):
            
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
from datetime import datetime
import json
from typing import Optional
from functions.locationFunctions import getDbLocation, getLocationsFromParent
from models.db.location import Location as DbLocation

from enum import Enum

class Relationships(Enum):
    NONE = 1
    DIRECT = 2
    ALL = 3

class Location:
    id: str
    name: str
    parent_id: str | None
    parent_location: Optional['Location']
    child_locations: Optional[list['Location']]

    def fromDb(dbLocation: DbLocation, parents: Relationships, children: Relationships):
        location = Location()
        location.id = dbLocation.id
        location.name = dbLocation.name
        location.parent_id = dbLocation.parent_id

        location.parent_location = None
        location.child_locations = []
        if (location.parent_id != None):
            match parents:
                case Relationships.DIRECT:
                    location.parent_location = Location.fromDb(getDbLocation(dbLocation.parent_id), Relationships.NONE, Relationships.NONE)
                case Relationships.ALL:
                    location.parent_location = Location.fromDb(getDbLocation(dbLocation.parent_id), Relationships.ALL, Relationships.NONE)

        match children:
            case Relationships.DIRECT:
                location.child_locations = Location.fromDbs(getLocationsFromParent(dbLocation.id), Relationships.NONE, Relationships.NONE)
            case Relationships.ALL:
                location.child_locations = Location.fromDbs(getLocationsFromParent(dbLocation.id), Relationships.NONE, Relationships.ALL)
        
        print(location.child_locations)
        return location

    def fromDbs(dbLocations: list[DbLocation], parents: Relationships, children: Relationships):
        arr = []
        if (dbLocations == None):
            return arr
        for dbLocation in dbLocations:
            arr.append(Location.fromDb(dbLocation, parents, children))
        return arr
    
    def toJSON(self: 'Location'):
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
    
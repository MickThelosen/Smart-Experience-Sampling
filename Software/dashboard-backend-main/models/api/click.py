from datetime import datetime
from models.db.click import Click as DbClick

from models.api.clickBeacon import ClickBeacon
from models.api.clicker import Clicker

import json

class Click:
    id: str
    click_time: datetime | str
    clicker_id: str
    click_beacons: list[ClickBeacon] | None
    clicker: Clicker

    def toDb(self):
        return DbClick.create(self.id, self.click_time, self.clicker_id)
    
    def fromDb(dbClick: DbClick):
        click = Click()
        click.id = dbClick.id
        click.click_time = dbClick.click_time
        click.clicker_id = dbClick.clicker_id

        return click
    
    def toJSON(self):
        if type(self.click_time) == datetime:
            self.click_time = datetime.isoformat(self.click_time)
            
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
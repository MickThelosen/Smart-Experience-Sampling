from datetime import datetime
from models.db.research import Research as DbResearch

import json

class Research:
    id: str
    question: str
    startTime: datetime | str
    endTime: datetime | str

    def fromDb(dbResearch: DbResearch):
        research = Research()
        research.id = dbResearch.id
        research.question = dbResearch.question
        research.startTime = dbResearch.start_time
        research.endTime = dbResearch.end_time

        return research
    
    def toJSON(self):
        if type(self.startTime) == datetime:
            self.startTime = datetime.isoformat(self.startTime)

        if type(self.endTime) == datetime:
            self.endTime = datetime.isoformat(self.endTime)
            
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
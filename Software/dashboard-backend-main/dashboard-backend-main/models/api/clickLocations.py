from models.api.click import Click as ApiClick


import json

class ClickWithLocation:
    click: ApiClick
    x: int
    y: int
    
    def toJSON(self):
            
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
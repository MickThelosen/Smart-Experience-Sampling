import json as Json

def json(data):
    return Json.dumps(data,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)
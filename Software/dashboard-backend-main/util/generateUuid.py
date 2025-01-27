import uuid

# uses uuidv4 (= guid), which is pretty common among loads of programming languages
def generateUuid():
    return str(uuid.uuid4())
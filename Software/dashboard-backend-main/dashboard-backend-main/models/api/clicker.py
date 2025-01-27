from models.db.clicker import Clicker as DbClicker

class Clicker:
    id: str
    uid: str

    def fromDb(dbClicker: DbClicker):
        clicker = Clicker()
        clicker.id = dbClicker.id
        clicker.uid = dbClicker.uid

        return clicker
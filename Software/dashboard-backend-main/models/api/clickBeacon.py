from models.api.beacon import Beacon

class ClickBeacon:
    click_id: str | None
    beacon_id: str | None
    beacon: Beacon | None
    distance: int
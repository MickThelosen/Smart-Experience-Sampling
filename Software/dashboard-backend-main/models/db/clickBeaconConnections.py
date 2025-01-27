from sqlalchemy import Column, String, Float

from util.setup import Base
from util.generateUuid import generateUuid

from models.db.click import Click


class ClickBeaconConnection(Base):
    __tablename__ = "click_beacon_connection"
    id = Column("id", String(36), primary_key=True)
    click_id = Column("click_id", String(36), nullable=False)
    beacon_id = Column("beacon_id", String(36), nullable=False)
    distance = Column("distance", Float, nullable=False)

    def create(id: String, click_id: String, beacon_id: String, distance: Float):
        self = ClickBeaconConnection()
        self.id = id
        self.click_id = click_id
        self.beacon_id = beacon_id
        self.distance = distance
        return self
    
    def new(click_id: String, beacon_id: String, distance: Float):
        self = ClickBeaconConnection.create(generateUuid(), click_id, beacon_id, distance)
        
        return self
    
    def value(self):
        
        return {
            "id": self.id,
            "click_id": self.click_id,
            "beacon_id": self.beacon_id,
            "distance": self.distance
        }
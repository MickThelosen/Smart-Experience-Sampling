from util.setup import Base
from sqlalchemy import Column, String, Float, ForeignKey, DateTime, func, Integer
from sqlalchemy.orm import relationship 
from util.generateUuid import generateUuid

class Beacon(Base):
    __tablename__ = "beacon"
    id = Column("id", String(36), primary_key=True)
    uid = Column("uid", String(255))
    x = Column("x", Float, nullable=False)
    y = Column("y", Float, nullable=False)

    location_id = Column("location_id", ForeignKey('location.id'))
    location = relationship("Location", back_populates='beacon')

    def create(id: String, uid: String, x: Float, y: Float, location_id: String):
        self = Beacon()
        self.id = id
        self.uid = uid
        self.x = x
        self.y = y
        self.location_id = location_id
        return self

    def new(uid: String, x: Float, y: Float, location_id: String):
        beacon = Beacon.create(generateUuid(), uid, x, y, location_id)
        return beacon

    def value(self):
        return {
            "id": self.id,
            "uid": self.uid
        }
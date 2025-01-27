from util.setup import Base
from sqlalchemy import Column, Integer, String, Float, ForeignKey, DateTime, func
from datetime import datetime
from util.generateUuid import generateUuid

class Clicker(Base):
    __tablename__ = "clicker"
    id = Column("id", String(36), primary_key=True)
    uid = Column("uid", String(255), nullable=False) # This is the ID known by the clicker

    def create(id: String, uid: String):
        self = Clicker()
        self.id = id
        self.uid = uid
        return self

    def new(uid: String):
        clicker = Clicker.create(generateUuid(), uid)
        return clicker
from util.setup import Base
from sqlalchemy import Column, String, Float, ForeignKey, DateTime, func
from datetime import datetime
from util.generateUuid import generateUuid


class Click(Base):
    __tablename__ = "click"
    id = Column("id", String(36), primary_key=True)
    click_time = Column("click_time", DateTime(timezone=True), default=func.now())
    clicker_id = Column("clicker_id", String(36), nullable=False)

    def create(id: String, click_time: datetime, clicker_id: String):
        self = Click()
        self.id = id
        self.click_time = click_time
        self.clicker_id = clicker_id
        return self
    
    def new(click_time: datetime, clicker_id: String):
        self = Click.create(generateUuid(), click_time, clicker_id)
        
        return self
    
    def value(self):
        print(self.click_time)
        
        return {
            "id": self.id,
            "click_time": datetime.isoformat(self.click_time),
            "clicker_id": self.clicker_id
        }
    
    # def getBeacons(self):

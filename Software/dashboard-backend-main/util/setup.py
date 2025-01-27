from util.database import engine
from sqlalchemy_utils import database_exists, create_database
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

def setup():
    print(engine)
    if not database_exists(engine.url):
        create_database(engine.url)
        print("Created database")

    Base.metadata.create_all(bind = engine)

    

from util.database import engine
from sqlalchemy.orm import sessionmaker

# isolated to prevent circular dependencies
Session = sessionmaker(bind=engine)
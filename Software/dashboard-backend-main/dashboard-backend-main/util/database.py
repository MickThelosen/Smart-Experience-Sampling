from sqlalchemy import create_engine, URL
from dotenv import load_dotenv
from flask_sqlalchemy import SQLAlchemy

load_dotenv()

import os

db = SQLAlchemy()

# uses either .env file or defaults
url_object = URL.create(
    os.environ.get("DB_TYPE", "mysql"),
    username=os.environ.get("DB_USERNAME", "root"),
    password=os.environ.get("DB_PASSWORD", "passwd"),
    host=os.environ.get("DB_URL", "localhost"),
    database=os.environ.get("DB_NAME", "database")
)
engine = create_engine(url_object, pool_size=100, max_overflow=100)

# for dependency injection
def Alchemy(app):
    db = SQLAlchemy(app)

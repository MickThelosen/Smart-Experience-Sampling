import unittest
from util.setup import Base
from models.db.clicker import Clicker
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from util.generateUuid import generateUuid

class TestClickerModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Set up the engine and create the tables once for all tests."""
        cls.engine = create_engine("sqlite:///:memory:")
        Base.metadata.create_all(cls.engine)
        cls.Session = sessionmaker(bind=cls.engine)

    @classmethod
    def tearDownClass(cls):
        """Drop the tables after all tests are done."""
        Base.metadata.drop_all(cls.engine)

    def setUp(self):
        """Create a new session for each test."""
        self.session = self.Session()

    def tearDown(self):
        """Close the session after each test."""
        self.session.close()

    def test_create_clicker(self):
        """Test creating a Clicker object with specific id and uid"""
        clicker_id = generateUuid()
        clicker_uid = 1234
        clicker = Clicker.create(clicker_id, clicker_uid)

        # Add the clicker to the session and commit
        self.session.add(clicker)
        self.session.commit()

        # Query the database to ensure the clicker was created
        created_clicker = self.session.query(Clicker).filter_by(id=clicker_id).first()
        self.assertIsNotNone(created_clicker)
        self.assertEqual(created_clicker.id, clicker_id)
        self.assertEqual(created_clicker.uid, clicker_uid)

    def test_new_clicker(self):
        """Test creating a new Clicker using the new() method"""
        clicker_uid = 5678
        clicker = Clicker.new(clicker_uid)

        # Add the clicker to the session and commit
        self.session.add(clicker)
        self.session.commit()

        # Query the database to ensure the new clicker was created
        created_clicker = self.session.query(Clicker).filter_by(uid=clicker_uid).first()
        self.assertIsNotNone(created_clicker)
        self.assertEqual(created_clicker.uid, clicker_uid)
        self.assertEqual(created_clicker.id, clicker.id)  # The ID should be automatically generated

    def test_clicker_duplicate(self):
        """Test that duplicate ids are not allowed"""
        clicker_id = generateUuid()
        clicker_uid = 1234
        clicker1 = Clicker.create(clicker_id, clicker_uid)
        self.session.add(clicker1)
        self.session.commit()

        # Attempt to insert a clicker with the same ID
        clicker2 = Clicker.create(clicker_id, clicker_uid)
        self.session.add(clicker2)

        # Check if an exception is raised due to primary key constraint violation
        with self.assertRaises(Exception):  # This will raise an exception due to primary key constraint violation
            self.session.commit()

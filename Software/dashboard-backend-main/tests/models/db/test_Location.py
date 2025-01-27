import unittest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from models.db.location import Location
from util.generateUuid import generateUuid
from util.setup import Base

class TestLocationModel(unittest.TestCase):

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

    def test_create_location(self):
        """Test creating a Location object with specific id, name, and parent_id"""
        location_id = generateUuid()
        location_name = "Test Location"
        location_parent_id = generateUuid()
        
        # Create the Location object using the create method
        location = Location.create(location_id, location_name, location_parent_id)

        # Add the location to the session and commit
        self.session.add(location)
        self.session.commit()

        # Query the database to ensure the location was created
        created_location = self.session.query(Location).filter_by(id=location_id).first()
        self.assertIsNotNone(created_location)
        self.assertEqual(created_location.id, location_id)
        self.assertEqual(created_location.name, location_name)
        self.assertEqual(created_location.parent_id, location_parent_id)

    def test_new_location(self):
        """Test creating a new Location using the new() method"""
        location_name = "New Location"
        location_parent_id = generateUuid()

        # Create the Location object using the new method
        location = Location.new(location_name, location_parent_id)

        # Add the location to the session and commit
        self.session.add(location)
        self.session.commit()

        # Query the database to ensure the new location was created
        created_location = self.session.query(Location).filter_by(name=location_name).first()
        self.assertIsNotNone(created_location)
        self.assertEqual(created_location.name, location_name)
        self.assertEqual(created_location.parent_id, location_parent_id)
        self.assertEqual(len(created_location.id), 36)  # Ensure the id is a valid UUID

    def test_new_location_no_parent(self):
        """Test creating a new Location without a parent_id"""
        location_name = "Location without Parent"
        
        # Create the Location object using the new method without a parent_id
        location = Location.new(location_name, None)

        # Add the location to the session and commit
        self.session.add(location)
        self.session.commit()

        # Query the database to ensure the new location was created
        created_location = self.session.query(Location).filter_by(name=location_name).first()
        self.assertIsNotNone(created_location)
        self.assertEqual(created_location.name, location_name)
        self.assertIsNone(created_location.parent_id)  # Ensure parent_id is None

    def test_create_location_empty_name(self):
        """Test creating a Location with an empty name"""
        location_id = generateUuid()
        location_name = ""  # Empty name
        location_parent_id = generateUuid()

        # Create the Location object using the create method
        location = Location.create(location_id, location_name, location_parent_id)

        # Add the location to the session and commit
        self.session.add(location)
        self.session.commit()

        # Query the database to ensure the location was created
        created_location = self.session.query(Location).filter_by(id=location_id).first()
        self.assertIsNotNone(created_location)
        self.assertEqual(created_location.id, location_id)
        self.assertEqual(created_location.name, location_name)  # Empty name is allowed
        self.assertEqual(created_location.parent_id, location_parent_id)

    def test_generate_uuid_function(self):
        """Test that generateUuid function generates valid UUIDs"""
        generated_uuid = generateUuid()
        self.assertEqual(len(generated_uuid), 36)  # UUIDs should be 36 characters long
        self.assertEqual(generated_uuid.count('-'), 4)  # UUID format has 4 hyphens

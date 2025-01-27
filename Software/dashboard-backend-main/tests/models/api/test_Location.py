import unittest
from unittest.mock import MagicMock
from models.api.location import Location, Relationships
from models.db.location import Location as DbLocation
from functions.locationFunctions import getDbLocation, getLocationsFromParent

class TestApiLocationModel(unittest.TestCase):

    def test_fromDb_basic(self):
        """Test the fromDb method with basic conversion using MagicMock for functions"""
        # Create a mock DbLocation object (direct object instantiation)
        db_location = DbLocation.create(id="1234-uuid", name="Test Location", parent_id="5678-uuid")
        
        # Create a MagicMock for the getDbLocation function (mocking the external function call)
        mock_get_db_location = MagicMock(return_value=DbLocation.create(id="5678-uuid", name="Parent Location", parent_id=None))
        
        # Create a MagicMock for the getLocationsFromParent function (no children)
        mock_get_locations_from_parent = MagicMock(return_value=[])

        # Mocking the function calls inside the Location.fromDb method
        with unittest.mock.patch('models.api.location.getDbLocation', mock_get_db_location), \
             unittest.mock.patch('models.api.location.getLocationsFromParent', mock_get_locations_from_parent):
            api_location = Location.fromDb(db_location, Relationships.DIRECT, Relationships.NONE)

        # Assert the values are correctly mapped
        self.assertEqual(api_location.id, db_location.id)
        self.assertEqual(api_location.name, db_location.name)
        self.assertEqual(api_location.parent_id, db_location.parent_id)
        self.assertIsNotNone(api_location.parent_location)
        self.assertEqual(api_location.parent_location.id, "5678-uuid")
        self.assertEqual(len(api_location.child_locations), 0)

    def test_fromDb_with_all_relationships(self):
        """Test the fromDb method with ALL relationships for both parent and children using MagicMock for functions"""
        # Create a mock DbLocation object
        db_location = DbLocation.create(id="1234-uuid", name="Test Location", parent_id="5678-uuid")
        
        # Mock the parent location
        mock_parent_location = DbLocation(id="5678-uuid", name="Parent Location", parent_id=None)
        
        # Mock the child location
        mock_child_location = DbLocation(id="91011-uuid", name="Child Location", parent_id="1234-uuid")

        # Create MagicMock for the getDbLocation function
        mock_get_db_location = MagicMock(return_value=mock_parent_location)
        
        # Create MagicMock for the getLocationsFromParent function (returning one child location)
        mock_get_locations_from_parent = MagicMock(side_effect=[[mock_child_location], []])

        # Mocking the function calls inside the Location.fromDb method
        with unittest.mock.patch('models.api.location.getDbLocation', mock_get_db_location), \
             unittest.mock.patch('models.api.location.getLocationsFromParent', mock_get_locations_from_parent):
            api_location = Location.fromDb(db_location, Relationships.ALL, Relationships.ALL)

        # Assert the values are correctly mapped
        self.assertEqual(api_location.id, db_location.id)
        self.assertEqual(api_location.name, db_location.name)
        self.assertEqual(api_location.parent_id, db_location.parent_id)
        self.assertIsNotNone(api_location.parent_location)
        self.assertEqual(api_location.parent_location.id, "5678-uuid")
        self.assertEqual(len(api_location.child_locations), 1)
        self.assertEqual(api_location.child_locations[0].id, "91011-uuid")

    def test_fromDbs(self):
        """Test converting a list of DbLocation objects using fromDbs with MagicMock for functions"""
        db_location_1 = DbLocation(id="1234-uuid", name="Location 1", parent_id="5678-uuid")
        db_location_2 = DbLocation(id="5678-uuid", name="Location 2", parent_id=None)
        
        db_locations = [db_location_1, db_location_2]

        # Create MagicMock for the functions (no external calls needed for the mock here)
        mock_get_db_location = MagicMock(return_value=None)
        mock_get_locations_from_parent = MagicMock(return_value=[])

        # Mocking the function calls inside the Location.fromDbs method
        with unittest.mock.patch('functions.locationFunctions.getDbLocation', mock_get_db_location), \
             unittest.mock.patch('functions.locationFunctions.getLocationsFromParent', mock_get_locations_from_parent):
            api_locations = Location.fromDbs(db_locations, Relationships.NONE, Relationships.NONE)

        # Assert that the list has been converted properly
        self.assertEqual(len(api_locations), 2)
        self.assertEqual(api_locations[0].id, db_location_1.id)
        self.assertEqual(api_locations[1].id, db_location_2.id)

    def test_toJSON(self):
        """Test converting a Location object to JSON using MagicMock for functions"""
        db_location = DbLocation(id="1234-uuid", name="Test Location", parent_id=None)

        # Convert it to an api Location object using fromDb
        api_location = Location.fromDb(db_location, Relationships.NONE, Relationships.NONE)

        # Convert the api Location object to JSON
        location_json = api_location.toJSON()

        # Assert that the JSON output contains the correct fields
        self.assertIn("id", location_json)
        self.assertIn("name", location_json)
        self.assertIn("parent_id", location_json)

        # Assert that the id is correct in the JSON
        self.assertTrue('"id": "1234-uuid"' in location_json)
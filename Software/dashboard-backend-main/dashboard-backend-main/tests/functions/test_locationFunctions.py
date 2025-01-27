import unittest
from unittest.mock import patch, MagicMock
from functions.locationFunctions import getDbLocation, getLocationsFromParent, getLocation
from models.db.location import Location


class TestLocationFunctions(unittest.TestCase):
    @patch('functions.locationFunctions.Session')
    def test_getDbLocation_found(self, MockSession):
        """Test getDbLocation when a location is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_location = MagicMock(spec=Location)
        mock_session.query.return_value.filter.return_value.first.return_value = mock_location

        location_id = "test-id"
        result = getDbLocation(location_id)

        # Assertions
        self.assertEqual(result, mock_location)
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()
        mock_session.close.assert_called_once()

    @patch('functions.locationFunctions.Session')
    def test_getDbLocation_not_found(self, MockSession):
        """Test getDbLocation when no location is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_session.query.return_value.filter.return_value.first.return_value = None

        location_id = "nonexistent-id"
        result = getDbLocation(location_id)

        # Assertions
        self.assertIsNone(result)
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()
        mock_session.close.assert_called_once()

    @patch('functions.locationFunctions.Session')
    def test_getLocationsFromParent_found(self, MockSession):
        """Test getLocationsFromParent when locations are found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_locations = [MagicMock(spec=Location), MagicMock(spec=Location)]
        mock_session.query.return_value.filter.return_value.all.return_value = mock_locations

        parent_id = "parent-id"
        result = getLocationsFromParent(parent_id)

        # Assertions
        self.assertEqual(result, mock_locations)
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.all.assert_called_once()
        mock_session.close.assert_called_once()

    @patch('functions.locationFunctions.Session')
    def test_getLocationsFromParent_not_found(self, MockSession):
        """Test getLocationsFromParent when no locations are found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_session.query.return_value.filter.return_value.all.return_value = []

        parent_id = "nonexistent-parent-id"
        result = getLocationsFromParent(parent_id)

        # Assertions
        self.assertEqual(result, [])
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.all.assert_called_once()
        mock_session.close.assert_called_once()

    @patch('functions.locationFunctions.Session')
    def test_getLocation_found(self, MockSession):
        """Test getLocation when a location is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_location = MagicMock(spec=Location)
        mock_session.query.return_value.filter.return_value.first.return_value = mock_location

        location_id = "test-id"
        result = getLocation(location_id)

        # Assertions
        self.assertEqual(result, mock_location)
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()
        mock_session.close.assert_called_once()

    @patch('functions.locationFunctions.Session')
    def test_getLocation_not_found(self, MockSession):
        """Test getLocation when no location is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_session.query.return_value.filter.return_value.first.return_value = None

        location_id = "nonexistent-id"
        result = getLocation(location_id)

        # Assertions
        self.assertIsNone(result)
        mock_session.query.assert_called_once_with(Location)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()
        mock_session.close.assert_called_once()
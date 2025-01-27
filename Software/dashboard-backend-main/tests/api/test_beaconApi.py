import unittest
from unittest.mock import MagicMock, patch
from models.db.beacon import Beacon

from api.beaconApi import getAllBeacons, getBeaconById, registerBeacon

class TestBeaconApi(unittest.TestCase):
    @patch('api.beaconApi.Session')
    @patch('api.beaconApi.select')
    def test_getAllBeacons(self, select_mock, mock_session):
        """Test getClickerByUid when a clicker is found."""
        mock_query = MagicMock()
        select_mock.return_value = mock_query

        mock_session_instance = MagicMock()
        mock_session.return_value = mock_session_instance

        mock_beacon = MagicMock()
        mock_beacon.value.return_value = Beacon.new(123)

        mock_session_instance.scalars.return_value = [mock_beacon]

        result, status_code = getAllBeacons()

        # Assertions
        select_mock.assert_called_once_with(Beacon)
        mock_session_instance.scalars.assert_called_once_with(mock_query)
        self.assertEqual(status_code, 200)

    @patch('api.beaconApi.Session')
    def test_getBeaconById_found(self, mock_session):
        mock_session_instance = MagicMock()
        mock_session.return_value = mock_session_instance

        mock_beacon = MagicMock()
        mock_beacon.value.return_value = Beacon.create('1', 123)

        mock_session_instance.query.return_value.filter.return_value.first.return_value = mock_beacon

        response, status_code = getBeaconById('1')

        mock_session_instance.query.assert_called_once_with(Beacon)
        self.assertEqual(status_code, 200)

    @patch('api.beaconApi.Session')  # Mock Session
    def test_registerBeacon(self, mock_Session):
        # Mock the session behavior
        mock_session_instance = MagicMock()
        mock_Session.return_value = mock_session_instance

        # Mock a DbBeacon instance and its behavior
        mock_beacon = Beacon.new("test_beacon_uid")

        # Call the function under test
        response, status_code = registerBeacon("test_beacon_uid")

        # Assert that the session methods are called as expected
        mock_session_instance.add.assert_called_once()
        mock_session_instance.commit.assert_called_once()

        # Assert the function's return value
        self.assertEqual(status_code, 200)
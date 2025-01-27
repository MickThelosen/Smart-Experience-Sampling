import unittest
from unittest.mock import patch, MagicMock
from functions.beaconFunctions import getBeaconByUid, getBeacon, getBeaconIdsByClickId, getBeaconsByClickId
from models.db.beacon import Beacon
from models.db.clickBeaconConnections import ClickBeaconConnection as CBC


class TestBeaconFunctions(unittest.TestCase):

    @patch('functions.beaconFunctions.Session')
    def test_getBeaconByUid(self, MockSession):
        """Test the getBeaconByUid function"""
        # Mock session and query
        mock_session = MagicMock()
        MockSession.return_value = mock_session
        mock_beacon = MagicMock(spec=Beacon)
        mock_session.query.return_value.filter.return_value.first.return_value = mock_beacon

        # Call the function
        uid = 1234
        result = getBeaconByUid(uid)

        # Assert that the session query was called correctly
        mock_session.query.assert_called_once_with(Beacon)
        
        # Ensure that filter was called with the correct arguments (i.e., Beacon.uid == 1234)
        mock_session.query.return_value.filter.assert_called_once()

        # Assert that the result is the mock beacon
        self.assertEqual(result, mock_beacon)

    @patch('functions.beaconFunctions.Session')
    def test_getBeacon(self, MockSession):
        """Test the getBeacon function"""
        # Mock session and query
        mock_session = MagicMock()
        MockSession.return_value = mock_session
        mock_beacon = MagicMock(spec=Beacon)
        mock_session.query.return_value.filter.return_value.first.return_value = mock_beacon

        # Call the function
        beacon_id = 'beacon-id-1234'
        result = getBeacon(beacon_id)

        # Assert that the session query was called correctly
        mock_session.query.assert_called_once_with(Beacon)

        # Assert that the result is the mock beacon
        self.assertEqual(result, mock_beacon)

    @patch('functions.beaconFunctions.Session')
    def test_getBeaconIdsByClickId(self, MockSession):
        """Test the getBeaconIdsByClickId function"""
        # Mock session and query
        mock_session = MagicMock()
        MockSession.return_value = mock_session
        mock_cbc = MagicMock(spec=CBC)
        mock_cbc.beacon_id = 'beacon-id-1234'
        mock_cbc.distance = 10.0
        mock_session.query.return_value.filter.return_value = [mock_cbc]

        # Call the function
        click_id = 'click-id-5678'
        result = getBeaconIdsByClickId(click_id)

        # Assert that the session query was called correctly
        mock_session.query.assert_called_once_with(CBC)
        mock_session.query.return_value.filter.assert_called_once()

        # Assert that the result contains the correct beacon ID
        self.assertEqual(result, ['beacon-id-1234'])

    @patch('functions.beaconFunctions.Session')
    def test_getBeaconsByClickId(self, MockSession):
        """Test the getBeaconsByClickId function"""
        # Mock session and query
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        cbcs = [CBC.create("cbc-id-1", 'click-id-1', 'beacon-id-1234', 10)]

        mock_beacon = Beacon.create('beacon-id-1234', 'beacon-uid')

        mock_get_beacon = MagicMock(return_value=mock_beacon)

    
        # Setup the query mocks
        mock_session.query.return_value.filter.return_value = cbcs
        
        # Call the function
        click_id = 'click-id-5678'

        with unittest.mock.patch('functions.beaconFunctions.getBeacon', mock_get_beacon):        
            result = getBeaconsByClickId(click_id)

        # Assert that the session query was called correctly
        mock_session.query.assert_called_once_with(CBC)

        # Assert that the result is a list of dictionaries with beacon data
        expected_result = [{'beacon': mock_beacon, 'distance': 10.0}]
        self.assertEqual(result, expected_result)

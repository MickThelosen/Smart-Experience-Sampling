import unittest
from unittest.mock import patch, MagicMock
from functions.clickerFunctions import getClickerByUid
from models.db.clicker import Clicker


class TestClickerFunctions(unittest.TestCase):
    @patch('functions.clickerFunctions.Session')
    def test_getClickerByUid_found(self, MockSession):
        """Test getClickerByUid when a clicker is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_clicker = MagicMock(spec=Clicker)
        mock_session.query.return_value.filter.return_value.first.return_value = mock_clicker

        uid = 12345
        result = getClickerByUid(uid)

        # Assertions
        self.assertEqual(result, mock_clicker)
        mock_session.query.assert_called_once_with(Clicker)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()

    @patch('functions.clickerFunctions.Session')
    def test_getClickerByUid_not_found(self, MockSession):
        """Test getClickerByUid when no clicker is found."""
        mock_session = MagicMock()
        MockSession.return_value = mock_session

        mock_session.query.return_value.filter.return_value.first.return_value = None

        uid = 67890
        result = getClickerByUid(uid)

        # Assertions
        self.assertIsNone(result)
        mock_session.query.assert_called_once_with(Clicker)
        mock_session.query.return_value.filter.assert_called_once()
        mock_session.query.return_value.filter.return_value.first.assert_called_once()
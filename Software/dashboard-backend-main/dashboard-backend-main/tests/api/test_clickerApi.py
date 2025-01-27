import unittest
from unittest.mock import MagicMock, patch
from models.db.clicker import Clicker

from api.clickerApi import getAllClickers, getClickerById, registerClicker

class TestClickerApi(unittest.TestCase):
    @patch('api.clickerApi.Session')
    @patch('api.clickerApi.select')
    def test_getAllClickers(self, select_mock, mock_session):
        """Test getClickerByUid when a clicker is found."""
        mock_query = MagicMock()
        select_mock.return_value = mock_query

        mock_session_instance = MagicMock()
        mock_session.return_value = mock_session_instance

        mock_clicker = MagicMock()
        mock_clicker.value.return_value = Clicker.new(123)

        mock_session_instance.scalars.return_value = [mock_clicker]

        result, status_code = getAllClickers()

        # Assertions
        select_mock.assert_called_once_with(Clicker)
        mock_session_instance.scalars.assert_called_once_with(mock_query)
        self.assertEqual(status_code, 200)

    @patch('api.clickerApi.Session')
    def test_getClickerById_found(self, mock_session):
        mock_session_instance = MagicMock()
        mock_session.return_value = mock_session_instance

        mock_clicker = MagicMock()
        mock_clicker.value.return_value = Clicker.create('1', 123)

        mock_session_instance.query.return_value.filter.return_value.first.return_value = mock_clicker

        response, status_code = getClickerById('1')

        mock_session_instance.query.assert_called_once()
        self.assertEqual(status_code, 200)

    @patch('api.clickerApi.Session')  # Mock Session
    def test_registerClicker(self, mock_Session):
        # Mock the session behavior
        mock_session_instance = MagicMock()
        mock_Session.return_value = mock_session_instance

        # Mock a Clicker instance and its behavior
        mock_clicker = Clicker.new("test_clicker_uid")

        # Call the function under test
        response, status_code = registerClicker("test_clicker_uid")

        # Assert that the session methods are called as expected
        mock_session_instance.add.assert_called_once()
        mock_session_instance.commit.assert_called_once()

        # Assert the function's return value
        self.assertEqual(status_code, 200)
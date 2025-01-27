import unittest
from unittest.mock import patch, MagicMock
from functions.researchFunctions import getActiveResearchAtTime
from models.db.research import Research

class TestResearchFunctions(unittest.TestCase):
    @patch('functions.researchFunctions.Session')  # Mock Session
    def test_getActiveResearchAtTime_found(self, mock_Session):
        # Mock the session and query behavior
        mock_session_instance = MagicMock()
        mock_Session.return_value = mock_session_instance

        # Mock a Research instance to simulate the query result
        mock_research = MagicMock()
        mock_session_instance.query.return_value.filter.return_value.filter.return_value.first.return_value = mock_research

        # Call the function with a test time
        time = 1000  # Example timestamp
        result = getActiveResearchAtTime(time)

        # Assert that the query was constructed correctly
        mock_session_instance.query.assert_called_once()

        # Assert that the session was closed
        mock_session_instance.close.assert_called_once()

        # Assert that the function returns the mock research object
        self.assertEqual(result, mock_research)
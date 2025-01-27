import unittest
from unittest.mock import patch
from datetime import datetime, timedelta
from models.db.research import Research
from util.generateUuid import generateUuid

class TestDbResearchModel(unittest.TestCase):

    def test_create_research(self):
        """Test creating a Research object with specific attributes"""
        # Define sample data
        question = "Click if you get distracted"
        start_time = datetime(2025, 1, 1, 10, 0, 0)
        end_time = datetime(2025, 1, 1, 12, 0, 0)
        created_time = datetime.now()

        # Create the research object using the create method
        research = Research.create(generateUuid(), question, created_time, start_time, end_time)

        # Assert that the research object has the correct attributes
        self.assertEqual(research.question, question)
        self.assertEqual(research.start_time, start_time)
        self.assertEqual(research.end_time, end_time)
        self.assertEqual(research.created_time, created_time)

    def test_new_research(self):
        """Test creating a new Research object using the new method"""
        question = "Click if you think it's too cold at your working spot"
        start_time = datetime(2025, 1, 2, 10, 0, 0)
        end_time = datetime(2025, 1, 2, 12, 0, 0)

        # Create the research object using the new method
        research = Research.new(question, start_time, end_time)

        # Assert that the research object is created with the provided question, start_time, and end_time
        self.assertEqual(research.question, question)
        self.assertEqual(research.start_time, start_time)
        self.assertEqual(research.end_time, end_time)

    def test_value_method(self):
        """Test the value method that returns a dictionary representation of the Research object"""
        question = "Click when you think it's too loud at your working location"
        start_time = datetime(2025, 1, 3, 10, 0, 0)
        end_time = datetime(2025, 1, 3, 12, 0, 0)
        created_time = datetime.now()

        # Create the research object
        research = Research.create(generateUuid(), question, created_time, start_time, end_time)

        # Get the dictionary representation of the research object
        value = research.value()

        # Assert that the dictionary has the correct values
        self.assertEqual(value["question"], question)
        self.assertEqual(value["start_time"], start_time.isoformat())
        self.assertEqual(value["end_time"], end_time.isoformat())
        self.assertEqual(value["created_time"], created_time.isoformat())
        self.assertNotEqual(value["id"], "00000000-0000-0000-0000-000000000000")

    def test_value_method_empty_fields(self):
        """Test the value method with empty fields"""
        # Create the research object with empty fields
        with self.assertRaises(TypeError):
            research = Research.create(None, None, None, None, None)

    def test_invalid_date_values(self):
        """Test handling invalid date values"""
        # Use an invalid date for start_time and end_time
        start_time = "invalid_date"
        end_time = "invalid_date"
        
        # Create the research object with invalid date values
        with self.assertRaises(TypeError):
            Research.create(generateUuid(), "Invalid Question", datetime.now(), start_time, end_time)

    def test_default_created_time(self):
        """Test that the created_time defaults to the current time when not provided"""
        question = "Click every time someone comes to ask you a question"
        start_time = datetime(2025, 1, 4, 10, 0, 0)
        end_time = datetime(2025, 1, 4, 12, 0, 0)

        # Create the research object using the new method
        research = Research.new(question, start_time, end_time)

        # Assert that the created_time is not None and is close to the current time
        self.assertIsNotNone(research.created_time)
        self.assertTrue(abs(research.created_time - datetime.now()) < timedelta(seconds=1))

import unittest
from unittest.mock import MagicMock
from datetime import datetime
from models.db.click import Click as DbClick
from models.api.click import Click
import json

class TestApiClick(unittest.TestCase):
    def setUp(self):
        # Set up a default DbClick instance
        self.db_click = DbClick(id="123", click_time=datetime(2025, 1, 1, 12, 0, 0), clicker_id="456")

        # Set up a default Click instance
        self.api_click = Click()
        self.api_click.id = "123"
        self.api_click.click_time = datetime(2025, 1, 1, 12, 0, 0)
        self.api_click.clicker_id = "456"

    def test_toDb(self):
        # Test the toDb method
        DbClick.create = MagicMock(return_value=self.db_click)

        db_click = self.api_click.toDb()

        # Assertions
        self.assertIsInstance(db_click, DbClick)
        self.assertEqual(db_click.id, self.api_click.id)
        self.assertEqual(db_click.click_time, self.api_click.click_time)
        self.assertEqual(db_click.clicker_id, self.api_click.clicker_id)

    def test_fromDb(self):
        # Test the fromDb method
        api_click = Click.fromDb(self.db_click)

        # Assertions
        self.assertIsInstance(api_click, Click)
        self.assertEqual(api_click.id, self.db_click.id)
        self.assertEqual(api_click.click_time, self.db_click.click_time)
        self.assertEqual(api_click.clicker_id, self.db_click.clicker_id)

    def test_toJSON(self):
        # Test the toJSON method
        expected_json = (
            '{\n'
            '    "clicker_id": "456",\n'
            '    "id": "123",\n'
            '    "click_time": "2025-01-01T12:00:00"\n'
            '}'
        )

        json_output = self.api_click.toJSON()

        # Assertions
        self.assertEqual(json.loads(json_output), json.loads(expected_json))
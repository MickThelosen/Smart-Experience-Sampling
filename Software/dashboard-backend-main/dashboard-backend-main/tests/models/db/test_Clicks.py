import unittest
from unittest.mock import MagicMock
from models.db.click import Click
from util.generateUuid import generateUuid
from datetime import datetime

class TestClick(unittest.TestCase):
    def setUp(self):
        # Set up a default Click instance
        self.click = Click.create(id="123", click_time=datetime(2025, 1, 1, 12, 0, 0), clicker_id="456")

    def tearDown(self):
        del self.click

    def test_create(self):
        # Test the create method
        self.click = Click.create(id="123", click_time=datetime(2025, 1, 1, 12, 0, 0), clicker_id="456")

        # Assertions
        self.assertIsInstance(self.click, Click)
        self.assertEqual(self.click.id, "123")
        self.assertEqual(self.click.click_time, datetime(2025, 1, 1, 12, 0, 0))
        self.assertEqual(self.click.clicker_id, "456")

    def test_new(self):
        # Test the new method
        generated_uuid = generateUuid()  # Simulate UUID generation
        old_create = Click.create
        Click.create = MagicMock(return_value=Click(id=generated_uuid, click_time=datetime(2025, 1, 3, 10, 0, 0), clicker_id="clicker-123"))
        new_click = Click.new(datetime(2025, 1, 3, 10, 0, 0), "clicker-123")

        # Assertions
        self.assertIsInstance(new_click, Click)
        self.assertEqual(new_click.clicker_id, "clicker-123")
        self.assertEqual(new_click.click_time, datetime(2025, 1, 3, 10, 0, 0))
        self.assertEqual(new_click.id, generated_uuid)
        Click.create = old_create

    def test_value(self):
        print(self.click.id)
        # Test the value method
        expected_value = {
            "id": "123",
            "click_time": "2025-01-01T12:00:00",
            "clicker_id": "456"
        }

        self.assertEqual(self.click.value(), expected_value)

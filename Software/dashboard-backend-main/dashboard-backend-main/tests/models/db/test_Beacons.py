import unittest
from unittest.mock import MagicMock
from models.db.beacon import Beacon as DbBeacon
from util.generateUuid import generateUuid

class TestDbBeacon(unittest.TestCase):
    def setUp(self):
        # Set up a default DbBeacon instance
        self.db_beacon = DbBeacon(id="123", uid=456)

    def test_create(self):
        # Test the create method
        new_beacon = DbBeacon.create("test-id", 789)

        # Assertions
        self.assertIsInstance(new_beacon, DbBeacon)
        self.assertEqual(new_beacon.id, "test-id")
        self.assertEqual(new_beacon.uid, 789)

    def test_new(self):
        # Test the new method
        generated_uuid = generateUuid()  # Simulate UUID generation
        DbBeacon.create = MagicMock(return_value=DbBeacon(id=generated_uuid, uid=101))

        new_beacon = DbBeacon.new(101)

        # Assertions
        self.assertIsInstance(new_beacon, DbBeacon)
        self.assertEqual(new_beacon.uid, 101)
        self.assertEqual(new_beacon.id, generated_uuid)

    def test_value(self):
        # Test the value method
        expected_value = {"id": "123", "uid": 456}

        self.assertEqual(self.db_beacon.value(), expected_value)
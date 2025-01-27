import unittest
from models.db.clickBeaconConnections import ClickBeaconConnection
from util.generateUuid import generateUuid
from unittest.mock import MagicMock

class TestClickBeaconConnection(unittest.TestCase):
    def setUp(self):
        # Set up a default ClickBeaconConnection instance
        self.connection = ClickBeaconConnection(id="conn-123", click_id="click-456", beacon_id="beacon-789", distance=15.5)

    def test_create(self):
        # Test the create method
        new_connection = ClickBeaconConnection.create("conn-999", "click-123", "beacon-456", 20.0)

        # Assertions
        self.assertIsInstance(new_connection, ClickBeaconConnection)
        self.assertEqual(new_connection.id, "conn-999")
        self.assertEqual(new_connection.click_id, "click-123")
        self.assertEqual(new_connection.beacon_id, "beacon-456")
        self.assertEqual(new_connection.distance, 20.0)

    def test_new(self):
        # Test the new method
        generated_uuid = generateUuid()  # Simulate UUID generation
        ClickBeaconConnection.create = MagicMock(return_value=ClickBeaconConnection(id=generated_uuid, click_id="click-123", beacon_id="beacon-456", distance=25.0))

        new_connection = ClickBeaconConnection.new("click-123", "beacon-456", 25.0)

        # Assertions
        self.assertIsInstance(new_connection, ClickBeaconConnection)
        self.assertEqual(new_connection.click_id, "click-123")
        self.assertEqual(new_connection.beacon_id, "beacon-456")
        self.assertEqual(new_connection.distance, 25.0)
        self.assertEqual(new_connection.id, generated_uuid)

    def test_value(self):
        # Test the value method
        expected_value = {
            "id": "conn-123",
            "click_id": "click-456",
            "beacon_id": "beacon-789",
            "distance": 15.5
        }

        self.assertEqual(self.connection.value(), expected_value)

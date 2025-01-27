import unittest

from models.db.beacon import Beacon as DbBeacon
from models.api.beacon import Beacon

class test_Beacons(unittest.TestCase):
    def setUp(self):
        # Create a mock DbBeacon instance
        self.mock_db_beacon = DbBeacon(id="123", uid=456)

    def test_fromDb(self):
        # Test the fromDb method
        beacon = Beacon.fromDb(self.mock_db_beacon)

        # Assertions
        self.assertIsInstance(beacon, Beacon)
        self.assertEqual(beacon.id, self.mock_db_beacon.id)
        self.assertEqual(beacon.uid, self.mock_db_beacon.uid)

    def test_fromDb_with_empty_values(self):
        # Test with a DbBeacon that has empty values
        self.mock_db_beacon.id = None
        self.mock_db_beacon.uid = None

        beacon = Beacon.fromDb(self.mock_db_beacon)

        # Assertions
        self.assertIsInstance(beacon, Beacon)
        self.assertIsNone(beacon.id)
        self.assertIsNone(beacon.uid)
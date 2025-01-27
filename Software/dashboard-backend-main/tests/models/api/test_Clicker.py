import unittest
from models.api.clicker import Clicker
from models.db.clicker import Clicker as DbClicker

class TestApiClickerModel(unittest.TestCase):

    def test_fromDb(self):
        """Test the fromDb method of the Clicker class"""
        # Create a mock DbClicker object
        db_clicker = DbClicker(id="1234-uuid", uid="5678")

        # Convert it to an api Clicker object using fromDb
        api_clicker = Clicker.fromDb(db_clicker)

        # Assert that the values match between the DbClicker and the api Clicker
        self.assertEqual(api_clicker.id, db_clicker.id)
        self.assertEqual(api_clicker.uid, db_clicker.uid)

    def test_fromDb_empty(self):
        """Test the fromDb method with empty values"""
        # Create a mock DbClicker object with empty fields
        db_clicker = DbClicker(id="", uid="")

        # Convert it to an api Clicker object using fromDb
        api_clicker = Clicker.fromDb(db_clicker)

        # Assert that the values match between the DbClicker and the api Clicker (empty values)
        self.assertEqual(api_clicker.id, db_clicker.id)
        self.assertEqual(api_clicker.uid, db_clicker.uid)

    def test_fromDb_none(self):
        """Test the fromDb method with a None DbClicker"""
        # Test that fromDb handles None gracefully (it should raise an error)
        with self.assertRaises(AttributeError):
            Clicker.fromDb(None)
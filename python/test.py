"""
Test case for the gjk wrapper
"""
import unittest
import gjk

class TestGjk(unittest.TestCase):

    def test_main(self):
        vertices1 = ((4,11), (4,5), (9,9))
        vertices2 = ((5,7), (7,3), (10,2), (12, 7))
        self.assertTrue(gjk.gjk(vertices1, vertices2))

    def test_empty(self):
        vertices1 = ()
        vertices2 = ((5,7), (7,3), (10,2), (12, 7))
        self.assertFalse(gjk.gjk(vertices1, vertices2))

    def test_mixed(self):
        vertices1 = ([4.0,11], (4,5), (9,9))
        vertices2 = [(5,7), (7.1,3), (10,2), (12, 7)]
        self.assertTrue(gjk.gjk(vertices1, vertices2))

    def test_bad_argument_type(self):
        with self.assertRaises(TypeError):
            gjk.gjk(None, None)

    def test_bad_argument_item(self):
        vertices1 = ((None,11),)
        vertices2 = ()
        vertices3 = ((11,),)

        with self.assertRaises(TypeError):
            gjk.gjk(vertices1, vertices2)

        with self.assertRaises(TypeError):
            gjk.gjk(vertices1, vertices3)


if __name__ == '__main__':
    unittest.main()
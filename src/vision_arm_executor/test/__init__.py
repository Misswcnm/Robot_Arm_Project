"""Unittest discovery hook used by ament_python/colcon test."""

import os


def load_tests(loader, tests, pattern):
    return loader.discover(
        os.path.dirname(__file__), pattern=pattern or 'test_*.py')

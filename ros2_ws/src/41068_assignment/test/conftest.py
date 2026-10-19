"""Make the modules in scripts/ importable from the tests.

The nodes import their library modules as siblings (scripts/ is not a Python
package), so the tests do the same.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

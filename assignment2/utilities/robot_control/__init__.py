#!/usr/bin/env python

"""
The map module for topological_map.
It includes three submodules for commands:
1. Manipulation - commands that can modify information stored in an ontology
2. Query - commands used to retrieve information from the ontology
3. Sysutil - utility commands such as load/save ontology and toggle ARMOR logging
Also includes:
1. Exceptions - All exceptions that armor_api commands can raise
"""

from . import map

__author__ = "Bauyrzhan Zhakanov"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "bauyrzhan.zhakanov@gmail.com"
__status__ = "Development"

__all__ = ['map.py']

.. robotPatrol documentation master file, created by
   sphinx-quickstart on Thu Feb 16 15:05:56 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to robotPatrol's documentation!
=======================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


Fsm Module
=====================
.. automodule:: src.fsm
   :members:

Robot_State Module
=====================
.. automodule:: src.robot_states
   :members:

Map Module
===========
.. automodule:: utilities.robot_control.map
   :members:

Marker Service
=======================
.. doxygenfile:: marker_server.cpp
   :project: robotPatrol

Controller Service
====================
.. doxygenfile:: robotik.cpp
   :project: robotPatrol

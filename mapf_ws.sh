#!/bin/bash

# Upgrade the matplotlib and pyqt5 to prevent from the error of 
# "ImportError: Matplotlib qt-based backends require an external PyQt4, PyQt5, or PySide package to be installed, but it was not found."
pip install --upgrade matplotlib
pip install --upgrade pyqt5

# Install Type Stubs for PyYAML:
pip install types-PyYAML
# Use mypy --install-types: This command will install all missing stub packages1. 
mypy --install-types
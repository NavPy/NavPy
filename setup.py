"""
NavPy: GNC Tools written in Python
"""
from setuptools import setup, find_packages

VERSION = "1.0"
 
setup (
    name = "NavPy",
    version = VERSION,
    description="NavPy is GNC Tools written in Python",
    long_description="",
    author="Adhika Lie, Hamid Mokhtarzadeh, Demoz Gebre-Egziabher",
    author_email="", # Removed to limit spam harvesting.
    url="https://github.com/NavPy/NavPy/",
    packages = find_packages(),
    install_requires=[
        'numpy>1.7.1'
    ],
    provides="navpy",
    license="BSD",
    maintainer="NavPy Developers",
    zip_safe=False
)

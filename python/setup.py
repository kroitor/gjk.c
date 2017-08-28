"""
While inside this file folder, run `python setup.py install` or `python setup.py build` to compile this library

NOTE: Visual Studio is required to build C extensions on Windows.
"""
from distutils.core import setup, Extension

gjk = Extension(
    name = "gjk",  
    sources = ["gjk_wrapper.c"],
    include_dirs = [".."],
)

setup(
    name = "gjk",
    description = "Python wrapper of Igor Kroitor implementation Gilbert-Johnson-Keerthi (GJK) collision detection algorithm",
    ext_modules = [gjk],
)

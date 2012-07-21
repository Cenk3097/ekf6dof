from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

ext_modules = [
    Extension("dynamics",
              ["dynamics.pyx"],
              include_dirs=['/usr/lib/python2.7/site-packages/numpy/core/include/'],
              extra_objects=[],
              libraries=['m',])]

setup(
  name = 'corisco aux functions',
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)

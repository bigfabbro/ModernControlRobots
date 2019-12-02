def __bootstrap__():
   global __bootstrap__, __loader__, __file__
   import sys, imp
   __loader__ = None; del __bootstrap__, __loader__
   imp.load_dynamic(__name__, 'python/intersect.so')
__bootstrap__()

import os

from panda3d.core import Vec3

class Asset():
  """
  This class is a singleton, it will always return the same model that can
  be placed many times in the scene while limiting memory use.

  Assets should inherit this class and add any modification they need (shaders)
  """
  singletonNodePath = None

  @classmethod
  def Get(self, path, params={}):
    """
    This is the actual loading, done once
    """
    if self.singletonNodePath is not None:
      return self.singletonNodePath

    self.singletonNodePath = loader.loadModel(path + "/model.dae")\
                                   .find("**/object")

    if self.singletonNodePath.isEmpty():
      print(__file__ + ":")
      print("Object in model file must be named 'object'.")
      exit()

    self.customize(self.singletonNodePath)

    return self.singletonNodePath

  def customize(self):
    # nothing to do here, can be implemented in asset
    pass

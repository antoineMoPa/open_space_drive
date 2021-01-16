import os

from panda3d.core import Vec3

class Model():
  """
  This class is a singleton, it will always return the same model that can
  be placed many times in the scene while limiting memory use.
  """
  singletonNodePath = None

  @classmethod
  def Get(self, params={}):
    """
    This is the actual loading, done once
    """
    if self.singletonNodePath is not None:
      return self.singletonNodePath

    self.singletonNodePath = loader.loadModel(os.path.dirname(__file__) + "/model.dae")\
                                   .find("**/object")

    if self.singletonNodePath.isEmpty():
      print("Object in model file must be named 'object'.")
      exit()

    self.singletonNodePath.setRenderModeWireframe()
    self.singletonNodePath.setColor(Vec3(0.0,0.6,0.0))

    return self.singletonNodePath

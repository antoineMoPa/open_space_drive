from panda3d.core import Vec3

from Asset import Asset

class Model(Asset):
  def customize(nodePath):
    nodePath.setRenderModeWireframe()
    nodePath.setColor(Vec3(0.0,0.6,0.0))

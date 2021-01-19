
from Asset import Asset

from panda3d.core import CullFaceAttrib
from panda3d.core import ColorBlendAttrib
from panda3d.core import Shader
from panda3d.core import Filename


class Model(Asset):
  def customize(nodePath):
    volume = nodePath.find("light_volume")
    volume.setTransparency(True)
    volume.setAttrib(CullFaceAttrib.make(CullFaceAttrib.MCullNone))
    volume.setDepthWrite(False)
    volume.setAttrib(ColorBlendAttrib.make(ColorBlendAttrib.M_add, ColorBlendAttrib.OIncomingAlpha, ColorBlendAttrib.O_one))
    volume.setShader(Shader.load(Shader.SL_GLSL,
                                 vertex=Filename("shaders/streetlight.vert"),
                                 fragment=Filename("shaders/streetlight.frag")))

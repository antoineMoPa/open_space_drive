from panda3d.core import Vec3
from AssetsManager import AssetsManager
from pandac.PandaModules import WindowProperties
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename
from main import OpenSpaceDriveApp
from panda3d.core import GraphicsWindow

from panda3d.core import loadPrcFileData
loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "win-size 512 512")

class PreviewApp(OpenSpaceDriveApp):
    def __init__(self):
        ShowBase.__init__(self)

        render.setShaderAuto()
        base.setBackgroundColor(0.3,0,0.4,1.0)

        self.initPostProcessing()
        self.addLights()

        assetManager = AssetsManager(useUI=False)
        asset = assetManager.generateAsset("palm_tree", Vec3(0,0,0), Vec3(0,0,0))
        nodePath = assetManager.instanciateAsset(asset)

        base.disableMouse()

        pt1, pt2 = nodePath.getTightBounds()
        height = pt2.getY() - pt1.getY()

        d = height * 4.5
        self.camera.setPos(Vec3(d,-d, d))
        self.camera.lookAt(Vec3(0,0,height*1.5))
        base.graphicsEngine.render_frame()
        base.screenshot('preview.png', False)


app = PreviewApp()

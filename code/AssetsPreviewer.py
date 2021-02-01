from panda3d.core import Vec3
from AssetsManager import AssetsManager
from pandac.PandaModules import WindowProperties
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename
from main import OpenSpaceDriveApp
from panda3d.core import GraphicsWindow
import os

from panda3d.core import loadPrcFileData
loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "win-size 115 115")
# disable cache
loadPrcFileData("", "model-cache-dir")

PREVIEW_FRAMES=30

class PreviewApp(OpenSpaceDriveApp):
    def __init__(self):
        ShowBase.__init__(self)

        render.setShaderAuto()
        base.setBackgroundColor(0.3,0,0.4,1.0)

        self.initPostProcessing()
        self.addLights()

        assetManager = AssetsManager(useUI=False)

        base.disableMouse()

        for asset_name in os.listdir("assets"):
            print("Generating %s" % asset_name)
            asset = assetManager.generateAsset(asset_name, Vec3(0,0,0), Vec3(0,0,0))
            nodePath = assetManager.instanciateAsset(asset)

            pt1, pt2 = nodePath.getTightBounds()
            height = pt2.getY() - pt1.getY()

            d = height * 4.5
            self.camera.setPos(Vec3(d,-d, d))
            self.camera.lookAt(Vec3(0,0,height))

            preview_folder = "assets/"+asset_name+'/preview/'

            if not os.path.exists(preview_folder):
                os.makedirs(preview_folder)

            for i in range(0,PREVIEW_FRAMES):
                nodePath.setH(i/PREVIEW_FRAMES*360)
                base.graphicsEngine.render_frame()
                base.screenshot(preview_folder+'/'+("%04d" % i)+'.png', False)

            os.system("avconv -i " + preview_folder + "/%04d.png -y " + preview_folder + "/preview.gif")
            os.system("rm " + preview_folder + "/*.png")
            base.graphicsEngine.render_frame()
            base.screenshot(preview_folder+'/preview.png', False)

            nodePath.remove()

if __name__ == "__main__":
    app = PreviewApp()

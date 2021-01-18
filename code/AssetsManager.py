import uuid
import json
import importlib
import cefpanda
from Refresh import Refresh

from panda3d.core import Vec3
from direct.task import Task

OBJECT_SELECTION_DISTANCE=30

class AssetsManagerUI():
    def __init__(self, assetsManager, cursorObject):
        self.assetsManager = assetsManager
        self.ui = cefpanda.CEFPanda(
            transparent=True,
            size=[-1.0, 1.0, -1.0, 1.0]
        )
        self.ui.node().setScale(1.0)
        self.ui.node().setPos((0, 0, 0))
        self.cursorObject = cursorObject
        self.load()
        self.selection = None
        self.nearest = None

        Refresh.addListener(self.load)

    def load(self):
        self.ui.load_file('assets_manager/main.html')
        self.ui.set_js_function('addAsset', self.addAsset)
        self.ui.set_js_function('clearSelection', self.clearSelection)
        self.ui.set_js_function('selectAsset', self.selectAsset)
        self.ui.set_js_function('deleteAsset', self.deleteAsset)

    def addAsset(self, asset_name):
        position = self.cursorObject.getPos()
        hpr = self.cursorObject.getHpr()
        self.assetsManager.registerAsset("palm_tree", position, hpr, {})

    def update(self):
        if self.selection is None:
            currentPosition = self.cursorObject.getPos()
            self.nearest = self.assetsManager.getClosestsAsset(currentPosition)

            if self.nearest is None:
                return

            distance = (Vec3(*self.nearest["position"]) - currentPosition).length()

            if distance > OBJECT_SELECTION_DISTANCE:
                self.nearest = None

            self.ui.exec_js_func('onNearestAsset', self.nearest)

    def clearSelection(self):
        self.selection = None

    def selectAsset(self, _uuid):
        if _uuid not in self.assetsManager.assets:
            return

        self.selection = self.assetsManager.assets[_uuid]
        self.ui.exec_js_func('onAssetSelected', self.selection)

    def deleteAsset(self, _uuid):
        self.selection = None
        self.assetsManager.deleteAsset(_uuid)


class AssetsManager():
    def __init__(self, cursorObject=None, useUI=True):
        self.assets = {}
        self.visibleAssets = {}
        self.readFile()
        self.cursorObject = cursorObject
        self.ui = AssetsManagerUI(self, cursorObject) if useUI else None
        self.assetsNodePaths = {}

    def readFile(self, file_path="./assets_store.json"):
        try:
            f = open(file_path,"r")
        except FileNotFoundError:
            print("New assets store will be created.")
            return
        self.assets = json.loads(f.read())
        f.close()

    def update(self, task):
        self.ui.update()
        return Task.cont

    def saveFile(self, file_path="./assets_store.json"):
        f = open(file_path,"w")
        f.write(json.dumps(self.assets))
        f.close()

    def generateAsset(self, path, position, hpr, parameters={}, _uuid=None):
        if _uuid is None:
            _uuid = uuid.uuid4()

        return {
            "path": path,
            "position": [position.x, position.y, position.z],
            "hpr": [hpr.x, hpr.y, hpr.z],
            "parameters": parameters,
            "uuid": str(_uuid)
        }

    def registerAsset(self, path, position, hpr, parameters={}):
        """
        path is the folder name of the asset
        position is a Vec3
        hpr is a Vec3
        parameters is a dictionnary
        """
        _uuid = uuid.uuid4()
        asset = self.generateAsset(path, position, hpr, parameters, _uuid)
        self.assets[str(_uuid)] = asset
        self.instanciateAsset(asset)
        self.saveFile()

    def deleteAsset(self, uuid):
        if uuid not in self.assets:
            return
        self.assetsNodePaths[uuid].removeNode()
        del self.assets[uuid]
        if uuid in self.visibleAssets:
            del self.visibleAssets[uuid]
        self.saveFile()

    def getClosestsAsset(self, Point):
        if len(self.assets) == 0:
            return None

        minIndex = min(self.assets, key=lambda i: (Point - Vec3(*self.assets[i]['position'])).length())
        return self.assets[minIndex]

    def getCloseAssets(self, Point):
        return self.assets

    def instanciateCloseAssets(self, Point):
        for i in self.assets:
            self.instanciateAsset(self.assets[i])

    def instanciateAsset(self, asset):
        assetModule = importlib.import_module("assets."+asset['path']+".asset")
        model = assetModule.Model.Get(asset['parameters'])
        placeholder = render.attachNewNode(asset['uuid'])
        model.instanceTo(placeholder)

        placeholder.setPos(asset['position'][0],asset['position'][1],asset['position'][2])
        placeholder.setHpr(asset['hpr'][0],asset['hpr'][1],asset['hpr'][2])
        placeholder.reparentTo(render)
        self.visibleAssets[asset['uuid']] = asset
        self.assetsNodePaths[asset['uuid']] = placeholder

        return placeholder

    def garbageCollectFarAssets(self, Point):
        pass

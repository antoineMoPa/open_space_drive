import uuid
import json
import importlib
import cefpanda
from WebUI import WebUI
from Refresh import Refresh

from panda3d.core import Vec3
from direct.task import Task

OBJECT_SELECTION_DISTANCE=30
OBJECT_INSTANCIATION_MAX=10
OBJECT_GC_DIST=3000

class AssetsManagerUI():
    def __init__(self, assetsManager, cursorObject):
        self.assetsManager = assetsManager
        self.ui = WebUI.get()
        self.ui.node().setScale(1.0)
        self.ui.node().setPos((0, 0, 0))
        self.cursorObject = cursorObject
        self.load()
        self.selection = None
        self.nearest = None

        self.shadowUUID = None

        Refresh.addListener(self.load)

    def load(self):
        self.ui.set_js_function('addAsset', self.beginAddAsset)
        self.ui.set_js_function('clearSelection', self.clearSelection)
        self.ui.set_js_function('selectAsset', self.selectAsset)
        self.ui.set_js_function('deleteAsset', self.deleteAsset)

    def beginAddAsset(self, asset_name):
        position = self.cursorObject.getPos()
        hpr = self.cursorObject.getHpr()
        _uuid = self.assetsManager.registerAsset(asset_name, position, hpr, {})
        nodePath = self.assetsManager.assetsNodePaths[_uuid]
        nodePath.setAlphaScale(0.5)
        self.shadowUUID = _uuid

    def finalizePlaceAsset(self):
        if self.shadowUUID is not None:
            nodePath = self.assetsManager.assetsNodePaths[self.shadowUUID]
            nodePath.setAlphaScale(1.0)
            self.shadowUUID = None

    def cancelPlaceAsset(self):
        if self.shadowUUID is not None:
            self.deleteAsset(self.shadowUUID)

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

        if self.shadowUUID is not None:
            newPosition = self.cursorObject.getPos()
            newHpr = self.cursorObject.getHpr()
            self.assetsManager.changeAssetPosition(self.shadowUUID, newPosition, newHpr)

    def clearSelection(self):
        self.selection = None

    def selectAsset(self, _uuid):
        if _uuid not in self.assetsManager.assets:
            return

        self.selection = self.assetsManager.assets[_uuid]
        self.ui.exec_js_func('onAssetSelected', self.selection)

    def deleteAsset(self, _uuid):
        if self.shadowUUID == _uuid:
            self.shadowUUID = None

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
        Refresh.addListener(self.refresh)

    def refresh(self):
        """
        F5 handler
        """

        for uuid in self.visibleAssets:
            self.assetsNodePaths[uuid].removeNode()
        self.visibleAssets = {}
        self.assetsNodePaths = {}
        self.readFile()

    def update(self, Point):
        self.instanciateCloseAssets(Point)
        self.garbageCollectFarAssets(Point)
        self.ui.update()

    def changeAssetPosition(self, _uuid, position, hpr):
        """
        Given position and hpr, update nodepath, visible assets dict and asset dict.
        """
        self.assetsNodePaths[_uuid].setPos(position)
        self.assetsNodePaths[_uuid].setHpr(hpr)
        self.visibleAssets[_uuid]["position"] = [position.x, position.y, position.z]
        self.visibleAssets[_uuid]["hpr"] = [hpr.x, hpr.y, hpr.z]
        self.assets[_uuid]["position"] = [position.x, position.y, position.z]
        self.assets[_uuid]["hpr"] = [hpr.x, hpr.y, hpr.z]

    def readFile(self, file_path="./assets_store.json"):
        try:
            f = open(file_path,"r")
        except FileNotFoundError:
            print("New assets store will be created.")
            return
        self.assets = json.loads(f.read())
        f.close()

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

        returns newly created uuid
        """
        _uuid = uuid.uuid4()
        asset = self.generateAsset(path, position, hpr, parameters, _uuid)
        self.assets[str(_uuid)] = asset
        self.instanciateAsset(asset)
        self.saveFile()

        return str(_uuid)

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

        def distFunc(i):
            return (Point - Vec3(*self.assets[i]['position'])).length()

        minIndex = min(self.assets, key=distFunc)
        return self.assets[minIndex]

    def getCloseAssets(self, Point, N=None):
        if N is None:
            N = len(self.assets)

        def distFunc(i):
            return (Point - Vec3(*self.assets[i]['position'])).length()

        return sorted(self.assets, key=distFunc)[0:N]

    def instanciateCloseAssets(self, Point):
        count = 0
        for i in self.getCloseAssets(Point):
            if i not in self.visibleAssets:
                self.instanciateAsset(self.assets[i])
                count = count + 1
                if count > OBJECT_INSTANCIATION_MAX:
                    break


    def instanciateAsset(self, asset):
        assetModule = importlib.import_module("assets."+asset['path']+".asset")
        model = assetModule.Model.Get("assets/" + asset['path'], asset['parameters'])
        placeholder = render.attachNewNode(asset['uuid'])
        model.instanceTo(placeholder)

        placeholder.setPos(asset['position'][0],asset['position'][1],asset['position'][2])
        placeholder.setHpr(asset['hpr'][0],asset['hpr'][1],asset['hpr'][2])
        placeholder.reparentTo(render)
        self.visibleAssets[asset['uuid']] = asset
        self.assetsNodePaths[asset['uuid']] = placeholder

        return placeholder

    def garbageCollectFarAssets(self, Point):
        uuidsToDelete = []
        for uuid in self.visibleAssets:
            if (Point - Vec3(*self.visibleAssets[uuid]['position'])).length() > OBJECT_GC_DIST:
                uuidsToDelete.append(uuid)

        for uuid in uuidsToDelete:
            self.assetsNodePaths[uuid].removeNode()
            del self.visibleAssets[uuid]

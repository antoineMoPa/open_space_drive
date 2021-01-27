
from math import pi, sin, cos, inf
import random
from io import StringIO
import sys

from direct.showbase.ShowBase import ShowBase
from direct.filter.FilterManager import FilterManager
from direct.task import Task
from direct.actor.Actor import Actor

from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import GeomVertexReader
from panda3d.core import GeomVertexWriter
from panda3d.core import Geom
from panda3d.core import GeomTriangles
from panda3d.core import GeomNode
from panda3d.core import GeomVertexData
from panda3d.core import GeomVertexFormat
from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import lookAt
from panda3d.core import Quat
from panda3d.core import Plane
from panda3d.core import Shader
from panda3d.core import CullFaceAttrib
from panda3d.core import ColorBlendAttrib
from panda3d.core import TexGenAttrib
from panda3d.core import Texture
from panda3d.core import TextureStage

from pandac.PandaModules import WindowProperties

from AssetsManager import AssetsManager
from Refresh import Refresh

from panda3d.core import loadPrcFileData

# disable cache
loadPrcFileData("", "model-cache-dir")

HIDE_DEBUG_VECTOR=True
ROAD_POS_STRENGTH=30
BLEND_VELOCITY_FAC_TOWARDS_ROAD=0.3
ALIGN_STRENGTH=10
HALF_ROAD_WIDTH=3

ALIGN_VELOCITY_WITH_DIRECTION_FACTOR=2

AI_CAR_QUANTITY=0
AI_MAX_DISTANCE=400

def pleaseMakeTransparent(nodePath):
    nodePath.setTransparency(True)
    nodePath.setAttrib(CullFaceAttrib.make(CullFaceAttrib.MCullNone))
    nodePath.setDepthWrite(False)
    nodePath.setAttrib(ColorBlendAttrib.make(ColorBlendAttrib.MAdd))


def move_debug_vector_to(position, look_at):
    vector = render.find("scene.dae").find("Scene").find("debug_vector")
    vector.setPos(position)
    vector.lookAt(look_at)

class RoadBuilder():
    def __init__(self):
        self.debugSphere = render.find("scene.dae").find("Scene").find("debug_sphere")
        self.debugVector = render.find("scene.dae").find("Scene").find("debug_vector")

        self.vdata = GeomVertexData('road', GeomVertexFormat.getV3t2(), Geom.UHStatic)
        # TODO: call this with right number of rows when it is known
        # self.vdata.setNumRows()
        self.vertex = GeomVertexWriter(self.vdata, 'vertex')
        self.texcoord = GeomVertexWriter(self.vdata, 'texcoord')
        self.prim = None
        self.prims = []
        self.lastv = 0
        self.vertices = []

        self.lru_vertices = []

    def addPrim(self):
        if self.prim is not None:
            for vertex in self.vertices:
                self.prim.addVertex(vertex)

            self.prims.append(self.prim)

        self.prim = GeomTriangles(Geom.UHStatic)

    def snapToExisting(self, point, tolerance=0.1):
        CACHE_SIZE=1024

        for i in self.lru_vertices:
            if (point - i).length() < tolerance:
                return i

        self.lru_vertices.append(point)

        if len(self.lru_vertices) > CACHE_SIZE:
            self.lru_vertices = self.lru_vertices[0:CACHE_SIZE]

        return point


    def updateTask(self, dt):
        self.nodepath.setShaderInput("time", (globalClock.getFrameTime()))
        return Task.cont

    def addSegment(self, p0, p1):

        width = HALF_ROAD_WIDTH

        side = -(p1 - p0).cross(Vec3().up()).normalized() * width
        length = (p1 - p0)

        h = (p1 - p0).length()

        v1 = side
        v2 = side + length
        v3 = -side + length
        v4 = -side

        v1 += p0
        v2 += p0
        v3 += p0
        v4 += p0

        v1 = self.snapToExisting(v1)
        v2 = self.snapToExisting(v2)
        v3 = self.snapToExisting(v3)
        v4 = self.snapToExisting(v4)

        self.vertex.addData3(v1)
        self.texcoord.addData2(1, 0)

        self.vertex.addData3(v2)
        self.texcoord.addData2(1, 1)

        self.vertex.addData3(v3)
        self.texcoord.addData2(0, 1)

        self.vertex.addData3(v4)
        self.texcoord.addData2(0, 0)

        lastv = self.lastv
        self.vertices.append(0 + lastv)
        self.vertices.append(1 + lastv)
        self.vertices.append(2 + lastv)
        self.vertices.append(3 + lastv)
        self.vertices.append(2 + lastv)
        self.vertices.append(0 + lastv)

        self.lastv += 4

    def finish(self):
        self.geom = Geom(self.vdata)

        for prim in self.prims:
            self.geom.addPrimitive(prim)

        self.road_visual_node = GeomNode('road_node')
        visnode = self.road_visual_node
        visnode.addGeom(self.geom)

        self.road_visual_nodePath = render.attachNewNode(visnode)
        nodepath = self.road_visual_nodePath
        nodepath.setTwoSided(True)
        nodepath.setShader(Shader.load(Shader.SL_GLSL,
                                       vertex="shaders/road.vert",
                                       fragment="shaders/road.frag"))
        pleaseMakeTransparent(nodepath)
        self.nodepath = nodepath
        taskMgr.add(self.updateTask, "updateTask")

        self.lru_vertices = []

def vectorRatio(v1, v2):
    def zerodiv(x,y):
        return 0 if y == 0 else x/y

    a = zerodiv(v1.x,v2.x)
    b = zerodiv(v1.y,v2.y)
    c = zerodiv(v1.z,v2.z)

    return sorted([a,b,c], key=lambda x: abs(x))[-1]

class RoadInfoSingleton():
    roadInfo = None

    @classmethod
    def set(self, roadInfo):
        RoadInfoSingleton.roadInfo = roadInfo

    @classmethod
    def get(self):
        return RoadInfoSingleton.roadInfo

class RoadInfo():
    def __init__(self):
        roadBuilder = RoadBuilder()
        model = render.find("scene.dae").find("Scene").find("road_path")
        model.hide()
        self.setPathModel(model.get_node(0).getGeom(0), roadBuilder)
        roadBuilder.finish()

    def setPathModel(self, geom, roadBuilder):
        # Credits: Most code in this function comes from panda3d's docs
        self.road_segments = []

        def processGeom(geom):
            vdata = geom.getVertexData()
            for i in range(geom.getNumPrimitives()):
                prim = geom.getPrimitive(i)
                processPrimitive(prim, vdata)

        def processPrimitive(prim, vdata):
            vertex = GeomVertexReader(vdata, 'vertex')
            prim = prim.decompose()


            for p in range(prim.getNumPrimitives()):
                s = prim.getPrimitiveStart(p)
                e = prim.getPrimitiveEnd(p)
                last  = None

                roadBuilder.addPrim()

                for i in range(s, e):
                    vi = prim.getVertex(i)
                    vertex.setRow(vi)
                    v = vertex.getData3()
                    if last is not None:
                        self.road_segments.append([last, v])
                        roadBuilder.addSegment(last, v)
                    last = v

        processGeom(geom)

class CarController():
    def __init__(self, model):
        self.car_shadow = NodePath('car_shadow') # Nodepath used for calculations only
                                                 # Maybe this could just be a vector (position)
                                                 # and a quaternion? (orientation)
        self.model = model
        self.acceleration = Vec3(0,0,0)
        self.velocity = Vec3(0,0,0)
        self.direction = Vec3(0,-1,0)
        self.angular_velocity = Vec3(0,0,0) # Heading Pitch Roll relative to car
        self.absolute_angular_velocity = Vec3(0,0,0) # Same, relative to world

        self.angular_velocity_damping = 0.97
        self.velocity_damping = 0.99
        self.acceleration_damping = 0.1
        self.disable_road_force_field = False
        self.roadInfo = RoadInfoSingleton.get()
        self.fires = []
        self.initShaders()
        self.reactors = [self.model.find("**/reactor_left"),\
                         self.model.find("**/reactor_right")]

    def initShaders(self):
        fire_shader = Shader.load(Shader.SL_GLSL,
                                  vertex="shaders/reactor_fire.vert",
                                  fragment="shaders/reactor_fire.frag")


        for fire in ["**/right_fire", "**/left_fire"]:
            nodePath = self.model.find(fire)
            nodePath.setShader(fire_shader)
            pleaseMakeTransparent(nodePath)
            self.fires.append(nodePath)


    def upVector(self):
        return self.model.getNetTransform().get_mat().xformVec(Vec3(0,0,1))

    def rightVector(self):
        car_right_vec = self.model.getNetTransform().get_mat().xformVec(Vec3(1,0,0))

    def pointSegmentShortestPath(self, Point, segmentP0, segmentP1, bounds=True, boundsNone=False):
        """
        Point-Segment Shortest Path

        Finds the shortest path between a point and road segment delimited by
        2 points (segmentP0, segmentP1).

        Line is considered infinite if bounds=False

        """

        P = Point - segmentP0

        SegmentVector = segmentP1 - segmentP0
        Len = SegmentVector.length()

        Projection = P.project(SegmentVector)

        ratio = vectorRatio(Projection, SegmentVector)

        # At bounds, closest point is the segment ending point
        if bounds == True and ratio < 0.0:
            if boundsNone:
                return None
            return Point - segmentP0

        if bounds == True and ratio > 1.0:
            if boundsNone:
                return None
            return Point - segmentP1

        return -(Projection - P)

    def getCloseRoadSegment(self, Point):
        shortest = (1e100, None, None)
        for segment in self.roadInfo.road_segments:
            Len0 = (segment[0] - Point).length()
            Len1 = (segment[1] - Point).length()
            if Len0 < shortest[0]:
                shortest = (Len0, segment[0], segment[1])
            if Len1 < shortest[0]:
                shortest = (Len1, segment[0], segment[1])

        return (shortest[1],shortest[2])

    def alignCarTowardsForceField(self, dt, p0, p1):
        modelPos = self.model.getPos()
        path = self.pointSegmentShortestPath(modelPos, p1, p0, bounds=True, boundsNone=True)

        if path is None:
            return None

        Len = path.length()

        if Len < 0.2:
            Len = 0.2

        delta = 100.0
        vec = (p1 - p0).normalized()
        vec *= delta

        originalQuat = self.model.getQuat()
        quat = Quat(originalQuat)
        other_quat = Quat(originalQuat)
        lookAt(quat, vec, Vec3().up())
        lookAt(other_quat, -vec, Vec3().up())

        if not quat.almostSameDirection(originalQuat, 0.5):
            [p0, p1] = [p1, p0]
            vec = (p0 - p1).normalized() * delta
            quat = other_quat

        if not quat.almostSameDirection(originalQuat, 0.5):
            return None

        # Make car go more towards the selected path at intersection
        angleSimilarityFactor = abs(self.direction.normalized().dot(vec.normalized()) + path.length())

        # Go closer to road
        pos_strength = dt * angleSimilarityFactor
        pos_strength /= (Len * Len) if Len > 1.0 else 1.0

        pos_strength *= ROAD_POS_STRENGTH
        pos_strength = sorted([0,pos_strength,1])[1]

        position = modelPos - path * pos_strength

        strength = ALIGN_STRENGTH * dt / ((Len * Len) if Len > 1 else 1)
        strength = sorted([0, strength, 1])[1]
        self.car_shadow.setPos(self.model.getPos())
        self.car_shadow.setHpr(self.model.getHpr())
        self.model.setQuat(quat)
        targetHpr = self.model.getHpr(self.car_shadow)
        self.model.setQuat(originalQuat)
        currentHpr = self.model.getHpr(self.car_shadow)
        self.model.setHpr(self.car_shadow, targetHpr * strength + currentHpr * (1.0 - strength))
        quat = self.model.getQuat()

        # Put velocity in direction of road
        blend_velocity = BLEND_VELOCITY_FAC_TOWARDS_ROAD * dt
        velocity = self.velocity.project(vec) * (blend_velocity) + self.velocity * (1.0 - blend_velocity)

        self.model.setPos(position)
        self.model.setQuat(quat)
        self.velocity = velocity

    def alignCarTowardsForceFields(self, dt):
        forceFieldsPaths = []
        modelPos = self.model.getPos()
        modelQuat = self.model.getQuat()

        for segment in self.roadInfo.road_segments:
            path = self.pointSegmentShortestPath(modelPos, segment[0], segment[1], bounds=True)
            if path is not None:
                Len = path.length()
                if Len < HALF_ROAD_WIDTH * 1.5:
                    forceFieldsPaths.append((Len, path, segment[0], segment[1]))

        def sortRoadsWithDirection(road):
            """
            Find the road that is the most pointed by current direction
            """
            _path = self.pointSegmentShortestPath(modelPos + self.direction * 30.0,
                                                 road[2], road[3], bounds=False)

            return _path.length() - _path.project(self.direction).length()

        closestFields = sorted(forceFieldsPaths,key=lambda x: x[0])[0:2]

        if len(closestFields) >= 2 and\
           closestFields[0][0] < HALF_ROAD_WIDTH * 3 and \
           closestFields[1][0] < HALF_ROAD_WIDTH * 3:
            closestFields = sorted(closestFields,key=sortRoadsWithDirection)[0:2]

        if len(closestFields) == 0 or closestFields[0] is None:
            return

        if not self.disable_road_force_field:
            self.alignCarTowardsForceField(dt, closestFields[0][2], closestFields[0][3])


    def update(self, dt):
        self.velocity += self.acceleration * (dt * 60)
        self.model.setPos(self.model.getPos() + self.velocity * (dt * 60))

        self.model.getNetTransform().get_mat().getRow3(1)

        self.model.setHpr(self.model, self.model.getHpr(self.model) + self.angular_velocity)
        self.model.setHpr(self.model.getHpr() + self.absolute_angular_velocity)

        for fire in self.fires:
            value = self.acceleration.length() * 10
            fire.setShaderInput("acceleration", value)
            fire.setShaderInput("time", (globalClock.getFrameTime()))

        reactor_angle_offset = 180.0 if (self.acceleration.normalized() - self.direction).length() < 0.1 else 0

        if self.acceleration.length() < 0.01:
            reactor_angle_offset = 180.0

        for reactor in self.reactors:
            reactor.setH(-self.angular_velocity[0]*8.0 + reactor_angle_offset)

        self.velocity *= pow(self.velocity_damping, dt * 60)
        self.acceleration *= pow(self.acceleration_damping, dt * 60)
        self.angular_velocity *= pow(self.angular_velocity_damping, dt * 60)
        self.absolute_angular_velocity *= pow(self.angular_velocity_damping, dt * 60)

        self.alignCarTowardsForceFields(dt)

        self.direction = self.model.getNetTransform().get_mat().getRow3(1)

        blend_velocity = ALIGN_VELOCITY_WITH_DIRECTION_FACTOR * dt
        self.velocity = self.velocity.project(self.direction) * (blend_velocity) + self.velocity * (1.0 - blend_velocity)



class PlayerCarController(CarController):
    def __init__(self, model):
        CarController.__init__(self, model)
        Refresh.addListener(self.refreshBodyShader)
        self.initBodyShader()

    def initBodyShader(self):
        rig = NodePath('rig')
        buffer = base.win.makeCubeMap('env', 128, rig)
        rig.reparentTo(self.model)

        lens = rig.find('**/+Camera').node().getLens()
        lens.setNearFar(30, 1000)

        ts = TextureStage('env')
        self.model.setTexGen(ts, TexGenAttrib.MWorldCubeMap)
        self.model.setTexture(buffer.getTexture())

        self.refreshBodyShader()

    def refreshBodyShader(self):
        self.model.setShader(Shader.make(Shader.SL_GLSL,
                                         open("shaders/playerCar.vert").read(),
                                         open("shaders/playerCar.frag").read()))

    def updateCameraUniform(self, task):
        self.model.setShaderInput("cameraPosition", base.camera.getPos())
        return Task.cont

    def brake(self, dt):
        self.acceleration *= pow(0.94, dt * 60)
        self.velocity *= pow(0.94, dt * 60)
        self.angular_velocity *= pow(0.94, dt * 60)

class AICarController(CarController):
    def __init__(self, model):
        CarController.__init__(self,model)
        self.resetRandom()

    def resetRandom(self, cameraPosition=Vec3(0.0)):
        def r():
            return random.uniform(0,1)

        cameraPlusRandomPos = cameraPosition + ((Vec3(r(),r(),r())-0.5) * 40.0)
        (segment0, segment1) = self.getCloseRoadSegment(cameraPlusRandomPos)

        if segment0 is None:
            segment0 = cameraPlusRandomPos
            segment1 = cameraPlusRandomPos + Vec3(r())

        self.model.setPos(segment1)
        self.model.lookAt(segment0)
        self.acceleration = Vec3(Vec3(r(),r(),r())*0.3)
        self.angular_velocity = Vec3(0)

    def update(self, dt, cameraPosition):
        CarController.update(self, dt)
        self.velocity += self.direction * 0.02

        if (cameraPosition - self.model.getPos()).length() > AI_MAX_DISTANCE:
            self.resetRandom(cameraPosition)



class AIFleetController():
    def __init__(self):
        carModel = loader.loadModel("../models/scene.dae").find("Scene").find("player_car")

        self.ai_car_controllers = []

        for i in range(AI_CAR_QUANTITY):
            placeholder = render.attachNewNode("ai-car-"+str(i))
            carModel.instanceTo(placeholder)
            AICarController(placeholder)
            controller = AICarController(placeholder)
            self.ai_car_controllers.append(controller)

    def update(self, dt, cameraPosition):
        for i in self.ai_car_controllers:
            i.update(dt, cameraPosition)

class OpenSpaceDriveApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.initWindow()
        self.taskMgr.add(self.cameraFollowTask, "CameraFollowTask")
        self.taskMgr.add(self.updateCarPositionTask, "UpdateCarPositionTask")

        dae = loader.loadModel("../models/scene.dae")
        dae.reparentTo(render)
        dae.setHpr(0,90,0)
        self.scene = dae
        self.last_cam_pos = Vec3(0.0)

        self.addLights()

        render.setShaderAuto()
        base.setBackgroundColor(0.0,0.0,0.1)

        RoadInfoSingleton.set(RoadInfo())

        self.car = dae.find("Scene").find("player_car")

        self.playerCarController = PlayerCarController(self.car)
        self.taskMgr.add(self.playerCarController.updateCameraUniform,
                         "UpdateCarCameraUniform")



        self.last_update_car_time = None
        self.last_update_camera_time = None

        self.bindKeys()
        self.initBuildings()
        self.initDebugVector()

        self.aiFleetController = AIFleetController()

        self.initPostProcessing()
        self.initAssets()

        Refresh.addListener(self.refresh)

    def refresh(self):
        self.refreshBuildings()

    def initWindow(self):
        props = WindowProperties()
        props.setTitle('Open Space Drive')
        base.win.requestProperties(props)

        base.pipe.getDisplayWidth()

        props = WindowProperties()
        props.setSize(base.pipe.getDisplayWidth(), base.pipe.getDisplayHeight())
        base.win.requestProperties(props)

    def initAssets(self):
        self.assetsManager = AssetsManager(self.car)
        self.assetsManager.instanciateCloseAssets(self.camera.getPos())
        self.taskMgr.add(self.assetsManager.update, "UpdateAssets")

        def placePalmTree():
            position = self.car.getPos()
            hpr = self.car.getHpr()
            self.assetsManager.registerAsset("palm_tree", position, hpr, {})

        self.accept('p', placePalmTree, [])

    def initPostProcessing(self):
        manager = FilterManager(base.win, base.cam)
        tex = Texture()
        quad = manager.renderSceneInto(colortex=tex)
        quad.setShader(Shader.load("shaders/post_processing.sha"))
        quad.setShaderInput("tex", tex)


    def initDebugVector(self):
        if HIDE_DEBUG_VECTOR:
            vector = render.find("scene.dae").find("Scene").find("debug_vector")
        vector.hide()

    def refreshBuildings(self):
        nodePath = self.buildingsNodePath
        nodePath = render.find("scene.dae").find("Scene").find("Buildings")

        nodePath.setShader(Shader.make(Shader.SL_GLSL,
                                       open("shaders/buildings.vert").read(),
                                       open("shaders/buildings.frag").read()))


    def initBuildings(self):
        nodePath = render.find("scene.dae").find("Scene").find("Buildings")
        self.buildingsNodePath = nodePath
        self.refreshBuildings()
        self.taskMgr.add(self.updateBuildings, "Update Buildings")

    def updateBuildings(self, task):
        self.buildingsNodePath.setShaderInput("camera_position", self.camera.getPos())
        return Task.cont


    def addLights(self):
        dlight = DirectionalLight('global_dlight')
        dlnp = render.attachNewNode(dlight)
        dlnp.setPos(Vec3(0,100,0))
        render.setLight(dlnp)

        dlight = DirectionalLight('camera_dlight')
        dlnp = render.attachNewNode(dlight)
        render.setLight(dlnp)

        self.dlightnp = dlnp

        alight = AmbientLight('alight')
        alight.setColor((0.05, 0.0, 0.08, 0.3))
        alnp = render.attachNewNode(alight)
        render.setLight(alnp)

    def addNewListenedKey(self, key):
        """ Binds event listeners for keyboard events for one key"""
        self.keys[key] = False
        self.accept(key, self.keyDown, [key])
        self.accept(key + '-up', self.keyUp, [key])

    def keyDown(self, keyName):
        """ Key down listener """
        self.keys[keyName] = True

    def keyUp(self, keyName):
        """ Key up listener """
        self.keys[keyName] = False

    def cameraFollowTask(self, task):
        if self.last_update_camera_time is None:
            self.last_update_camera_time = task.time - 0.01

        dt = task.time - self.last_update_camera_time

        car_dir = self.playerCarController.direction

        current_cam_pos = self.last_cam_pos
        self.camera.setPos(self.car, Vec3(0.0,-25.0,4.0))


        # prevent smooth for now
        #self.camera.setHpr(self.car.getHpr())
        #self.dlightnp.setPos(self.camera.getPos())
        #self.dlightnp.headsUp(self.car)
        #self.last_update_camera_time = task.time
        #return Task.cont

        #self.camera.setPos(self.car, Vec3(0.0,-25.0,0.0))
        target_cam_pos = self.camera.getPos()
        convergeSpeed = 8.0 * dt

        self.last_cam_pos = (current_cam_pos * (1.0-convergeSpeed) + (target_cam_pos * convergeSpeed))
        self.camera.setPos(self.last_cam_pos)

        self.camera.setHpr(self.car.getHpr())
        self.dlightnp.setPos(self.camera.getPos())
        self.dlightnp.headsUp(self.car)

        self.last_update_camera_time = task.time

        return Task.cont

    def bindKeys(self):
        self.keys = dict()
        self.addNewListenedKey('w')            # Forward
        self.addNewListenedKey('s')            # Backward
        self.addNewListenedKey('shift')        # Boost
        self.addNewListenedKey('q')            # Up
        self.addNewListenedKey('e')            # Down
        self.addNewListenedKey('space')        # Slow down
        self.addNewListenedKey('a')            # Turn left
        self.addNewListenedKey('d')            # Turn right
        self.addNewListenedKey('arrow_up')     # Head down
        self.addNewListenedKey('arrow_down')   # Head up
        self.addNewListenedKey('arrow_left')   # Roll left
        self.addNewListenedKey('arrow_right')  # Roll right
        self.addNewListenedKey('f5')           # Refresh


    def updateCarPositionTask(self, task):
        ROLL_LEFT_KEY  = "arrow_left"
        ROLL_RIGHT_KEY = "arrow_right"
        TURN_LEFT_KEY  = "a"
        TURN_RIGHT_KEY = "d"

        if self.last_update_car_time is None:
            self.last_update_car_time = task.time - 0.01

        dt = task.time - self.last_update_car_time

        self.playerCarController.disable_road_force_field = True if self.keys['q'] or self.keys['e'] else False

        if self.keys['w']:
            gaz = 1.2
            if self.keys['shift']:
                gaz *= 1.0 + self.playerCarController.velocity.length()
            self.playerCarController.acceleration += self.playerCarController.direction.normalized() * gaz * dt
        elif self.keys['s']:
            self.playerCarController.acceleration -= self.playerCarController.direction.normalized() * 0.5 * dt
        elif self.keys['space']:
            self.playerCarController.brake(dt)

        if self.keys['q']:
            self.playerCarController.acceleration -= self.playerCarController.upVector() * 1.2 * dt
        if self.keys['e']:
            self.playerCarController.acceleration += self.playerCarController.upVector() * 1.2 * dt

        carRotateMatrix = self.car.get_mat().rotateMat(0,axis=1)

        if self.keys['arrow_up']:
            self.playerCarController.angular_velocity += Vec3(0.0,-1.50,0) * dt
        if self.keys['arrow_down']:
            self.playerCarController.angular_velocity += Vec3(0.0,1.50,0) * dt

        if self.keys[TURN_LEFT_KEY]:
            self.playerCarController.angular_velocity += Vec3(0.0,0.0,-2.0) * dt
        elif self.keys[TURN_RIGHT_KEY]:
            self.playerCarController.angular_velocity += Vec3(0.0,0.0,2.0) * dt

        if self.keys[ROLL_LEFT_KEY] or self.keys[ROLL_RIGHT_KEY]:
            factor = 4.0 * dt

            if self.keys[ROLL_RIGHT_KEY]:
                factor *= -1

            self.playerCarController.angular_velocity += Vec3(factor,0.0,0.0)


        self.playerCarController.update(dt)

        self.aiFleetController.update(dt, self.camera.getPos())

        self.last_update_car_time = task.time

        if self.keys['f5']:
            Refresh.sendRefresh()

        return Task.cont

if __name__ == "__main__":
    app = OpenSpaceDriveApp()
    app.run()

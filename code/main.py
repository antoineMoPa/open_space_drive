from math import pi, sin, cos, inf

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import GeomVertexReader
from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import Vec4

class CarController():
    def __init__(self, model):
        self.model = model
        self.acceleration = Vec3(0,0,0)
        self.velocity = Vec3(0,0,0)
        self.direction = Vec3(0,-1,0)
        self.angular_velocity = Vec3(0,0,0) # Heading Pitch Roll


        self.angular_velocity_damping = 0.99
        self.velocity_damping = 0.99
        self.acceleration_damping = 0.1
        self.last_is_going_forward = True

    def brake(self, dt):
        self.acceleration *= 0.94 * (dt * 60)
        self.velocity *= 0.94 * (dt * 60)
        self.angular_velocity *= 0.94 * (dt * 60)

    def upVector(self):
        return self.model.getNetTransform().get_mat().xformVec(Vec3(0,0,1))

    def rightVector(self):
        car_right_vec = self.model.getNetTransform().get_mat().xformVec(Vec3(1,0,0))

    def setPathModel(self, geom):
        # Credit: Most code in this function comes from panda3d's docs
        self.road_segments = []

        def processGeom(geom):
            vdata = geom.getVertexData()
            for i in range(geom.getNumPrimitives()):
                prim = geom.getPrimitive(i)
                processPrimitive(prim, vdata)

        def processPrimitive(prim, vdata):
            vertex = GeomVertexReader(vdata, 'vertex')
            prim = prim.decompose()
            last = None
            for p in range(prim.getNumPrimitives()):
                s = prim.getPrimitiveStart(p)
                e = prim.getPrimitiveEnd(p)
                for i in range(s, e):
                    vi = prim.getVertex(i)
                    vertex.setRow(vi)
                    v = vertex.getData3()
                    if last is not None:
                        self.road_segments.append([last, v])
                    last = v


        processGeom(geom)


    def pointSegmentShortestPath(self, Point, SegmentP1, SegmentP2):
        """
        Point-Segment Shortest Path

        Finds the shortest path between a point and road segment delimited by
        2 points (SegmentP1, SegmentP2).

        The value None is returned if the point cannot be found in a cylinder centered on
        the vector between P1 and P2 and delimited by these points. (whatever the radius)

        """
        SegmentVector = SegmentP2 - SegmentP1
        Len = SegmentVector.length()
        Projection = (Point - SegmentP1).project(SegmentVector)

        if Projection.x < 0 or Projection.y < 0 or Projection.z < 0:
            return None
        if Projection.x > Len or Projection.y > Len or Projection.z > Len:
            return None

        ShortestPath = Point - (Projection +  SegmentP1)

        return ShortestPath

    def alignCarWithForceFields(self, dt):
        for segment in self.road_segments:
            path = self.pointSegmentShortestPath(self.model.getPos(), segment[0], segment[1])
            if path is not None and path.length() < 10:
                v = Vec3(segment[0] - segment[1])
                angle = v.normalized().angleDeg(self.direction.normalized())
                if angle > 90 or angle < -90:
                    v = Vec3(segment[1] - segment[0])
                    angle = v.normalized().angleDeg(self.direction.normalized())

                initialAngle = self.model.getHpr()
                self.model.lookAt(self.model.getPos() + v * 1000.0)
                targetAngle = self.model.getHpr()


                strength = 4 * dt
                strength = sorted([0,strength,1])[1]
                self.model.setHpr(initialAngle * (1.0 - strength) + targetAngle * strength)

                strength = sorted([0,strength,1])[1]

                self.model.setPos(self.model.getPos() - path * strength)

                break


    def updatePos(self, dt):
        self.velocity += self.acceleration * (dt * 60)
        self.model.setPos(self.model.getPos() + self.velocity * (dt * 60))

        self.model.getNetTransform().get_mat().getRow3(1)

        self.model.setHpr(self.model, self.model.getHpr(self.model) + self.angular_velocity)

        self.velocity *= self.velocity_damping * (dt * 60)
        self.acceleration *= self.acceleration_damping * (dt * 60)
        self.angular_velocity *= self.angular_velocity_damping * (dt * 60)
        self.direction = self.model.getNetTransform().get_mat().getRow3(1)

        self.alignCarWithForceFields(dt)

        blend_velocity = 0.2 * (dt*30)
        self.velocity = self.velocity.project(self.direction) * (1.0 - blend_velocity) + self.velocity * blend_velocity


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.taskMgr.add(self.cameraFollowTask, "CameraFollowTask")
        self.taskMgr.add(self.updateCarPositionTask, "UpdateCarPositionTask")

        dae = loader.loadModel("../models/scene.dae")
        dae.reparentTo(render)
        dae.setHpr(0,90,0)
        self.scene = dae
        self.last_cam_pos = Vec3(0.0)

        self.addLights()

        #barrier = m.find('Scene').find('right_barrier')
        #barrier.set_pos(1,1,1)

        render.setShaderAuto()
        self.car = dae.find("Scene").find("player_car")

        self.car_controller = CarController(self.car)

        self.last_update_car_time = None
        self.last_update_camera_time = None
        self.is_backing_up = False

        self.car_controller.setPathModel(dae.find("Scene").find("road_path").get_node(0).getGeom(0))

        self.bindKeys()

    def addLights(self):
        dlight = DirectionalLight('global_dlight')
        dlnp = render.attachNewNode(dlight)
        render.setLight(dlnp)

        dlight = DirectionalLight('camera_dlight')
        dlnp = render.attachNewNode(dlight)
        render.setLight(dlnp)

        self.dlightnp = dlnp

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.0, 0.5, 0.3))
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

        car_dir = self.car_controller.direction

        current_cam_pos = self.last_cam_pos
        self.camera.setPos(self.car, Vec3(0.0,-25.0,4.0))
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
        self.addNewListenedKey('q')            # Up
        self.addNewListenedKey('e')            # Down
        self.addNewListenedKey('space')        # Slow down
        self.addNewListenedKey('a')            # Turn left
        self.addNewListenedKey('d')            # Turn right
        self.addNewListenedKey('arrow_up')     # Head down
        self.addNewListenedKey('arrow_down')   # Head up
        self.addNewListenedKey('arrow_left')   # Roll left
        self.addNewListenedKey('arrow_right')  # Roll right


    def updateCarPositionTask(self, task):
        ROLL_LEFT_KEY  = "arrow_left"
        ROLL_RIGHT_KEY = "arrow_right"
        TURN_LEFT_KEY  = "a"
        TURN_RIGHT_KEY = "d"

        if self.last_update_car_time is None:
            self.last_update_car_time = task.time - 0.01

        dt = task.time - self.last_update_car_time

        if self.keys['w']:
            self.car_controller.acceleration += self.car_controller.direction.normalized() * 1.2 * dt
            self.is_backing_up = False
        elif self.keys['s']:
            self.car_controller.acceleration -= self.car_controller.direction.normalized() * 0.5 * dt
            if self.car_controller.velocity.length() < 0.4:
                self.is_backing_up = True
        elif self.keys['space']:
            self.car_controller.brake(dt)

        if self.keys['q']:
            self.car_controller.acceleration += self.car_controller.upVector() * 3.2 * dt
        if self.keys['e']:
            self.car_controller.acceleration -= self.car_controller.upVector() * 3.2 * dt

        carRotateMatrix = self.car.get_mat().rotateMat(0,axis=1)

        if self.keys['arrow_up']:
            self.car_controller.angular_velocity += Vec3(0.0,-1.50,0) * dt
        if self.keys['arrow_down']:
            self.car_controller.angular_velocity += Vec3(0.0,1.50,0) * dt

        if self.keys[TURN_LEFT_KEY]:
            self.car_controller.angular_velocity += Vec3(0.0,0.0,-2.00) * dt
        if self.keys[TURN_RIGHT_KEY]:
            self.car_controller.angular_velocity += Vec3(0.0,0.0,2.00) * dt


        if self.keys[ROLL_LEFT_KEY] or self.keys[ROLL_RIGHT_KEY]:
            factor = 2.0 * dt

            if self.keys[ROLL_RIGHT_KEY]:
                factor *= -1

            if self.is_backing_up:
                factor *= -1
                factor *= 2.0

            # Main rotation axis
            self.car_controller.angular_velocity += Vec3(factor,0.0,0.0)

        self.last_update_car_time = task.time

        self.car_controller.updatePos(dt)
        return Task.cont


app = MyApp()
app.run()

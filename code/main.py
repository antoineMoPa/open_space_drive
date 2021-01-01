from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import AmbientLight, DirectionalLight

from panda3d.core import NodePath
from panda3d.core import Vec3

class CarController():
    def __init__(self, model):
        self.model = model
        self.acceleration = Vec3(0,0,0)
        self.velocity = Vec3(0,0,0)
        self.direction = Vec3(0,-1,0)
        self.angular_velocity = Vec3(0,0,0) # Heading Pitch Roll


        self.angular_velocity_damping = 0.94
        self.velocity_damping = 0.97
        self.acceleration_damping = 0.1
        self.last_is_going_forward = True

    def updatePos(self):
        # TODO: add delta time as argument and use it to adapt
        # this code for varying fps
        self.velocity += self.acceleration
        self.model.setPos(self.model.getPos() + self.velocity)
        self.model.setHpr(self.model.getHpr() + self.angular_velocity)

        self.velocity *= self.velocity_damping
        self.acceleration *= self.acceleration_damping
        self.angular_velocity *= self.angular_velocity_damping
        self.direction = self.model.getNetTransform().get_mat().getRow3(1)

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.taskMgr.add(self.cameraFollowTask, "CameraFollowTask")
        self.taskMgr.add(self.updateCarPositionTask, "UpdateCarPositionTask")

        dae = loader.loadModel("../models/scene.dae")
        dae.reparentTo(render)
        dae.setHpr(0,90,0)
        self.scene = dae


        self.addLights()

        #barrier = m.find('Scene').find('right_barrier')
        #barrier.set_pos(1,1,1)

        render.setShaderAuto()
        self.car = dae.find("Scene").find("player_car")

        self.car_controller = CarController(self.car)

        self.is_last_acceleration_forward = True

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
        alight.setColor((0.2, 0.2, 0.2, 0.3))
        alnp = render.attachNewNode(alight)
        render.setLight(alnp)



    def bindKeys(self):
        self.keys = dict()
        self.addNewListenedKey('w')      # Forward
        self.addNewListenedKey('s')      # Backward
        self.addNewListenedKey('space')  # Slow down
        self.addNewListenedKey('a')      # Turn left
        self.addNewListenedKey('d')      # Turn right

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
        currentCameraPosition = self.camera.getPos()
        cameraFinalPosition = (self.car.getPos() -
                               self.car_controller.direction * 20.0 +
                               Vec3(0,0,6))

        #convergeSpeed = 0.99
        #self.camera.setPos((cameraFinalPosition   * convergeSpeed) +
        #                   (currentCameraPosition * (1.0-convergeSpeed)))


        self.camera.setPos(cameraFinalPosition)


        self.camera.headsUp(self.car)
        self.dlightnp.setPos(self.camera.getPos())
        self.dlightnp.headsUp(self.car)
        return Task.cont

    def updateCarPositionTask(self, task):

        if self.keys['w']:
            self.car_controller.acceleration += self.car_controller.direction.normalized() * 0.05
            self.is_last_acceleration_forward = True
        elif self.keys['s']:
            self.car_controller.acceleration -= self.car_controller.direction.normalized() * 0.05
            self.is_last_acceleration_forward = False
        elif self.keys['space']:
            self.car_controller.acceleration *= 0.9
            self.car_controller.velocity *= 0.7

        if self.keys['a'] or self.keys['d']:
            factor = 0.2

            if self.keys['d']:
                factor *= -1

            if not self.is_last_acceleration_forward:
                factor *= -1

            self.car_controller.angular_velocity += Vec3(factor,0.0,0.0)

        self.car_controller.updatePos()
        return Task.cont


app = MyApp()
app.run()

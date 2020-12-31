from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import DirectionalLight

from panda3d.core import NodePath
from panda3d.core import Vec3

class CarController():
    def __init__(self, model):
        self.model = model
        self.acceleration = Vec3(0,0,0)
        self.velocity = Vec3(0,0,0)
        self.angular_velocity = Vec3(0,0,0) # Heading Pitch Roll


        self.angular_velocity_damping = 0.97
        self.velocity_damping = 0.97
        self.acceleration_damping = 0.1

    def updatePos(self):
        # TODO: add delta time as argument and use it to adapt
        # this code for varying fps
        self.velocity += self.acceleration
        self.model.setPos(self.model.getPos() + self.velocity)
        self.model.setHpr(self.model.getHpr() + self.angular_velocity)

        self.velocity *= self.velocity_damping
        self.acceleration *= self.acceleration_damping
        self.angular_velocity *= self.angular_velocity_damping



class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.taskMgr.add(self.cameraFollowTask, "CameraFollowTask")
        self.taskMgr.add(self.updateCarPositionTask, "UpdateCarPositionTask")

        dae = loader.loadModel("../models/scene.dae")
        dae.reparentTo(render)
        dae.setHpr(0,90,0)
        self.scene = dae

        dlight = DirectionalLight('my dlight')
        dlnp = render.attachNewNode(dlight)
        render.setLight(dlnp)

        #barrier = m.find('Scene').find('right_barrier')
        #barrier.set_pos(1,1,1)

        render.setShaderAuto()
        self.car = dae.find("Scene").find("player_car")

        self.car_controller = CarController(self.car)


        self.bindKeys()

    def bindKeys(self):
        self.accept('w-repeat', self.forward)    # Gas
        self.accept('s-repeat', self.backward)   # Gas (back)
        self.accept('a-repeat', self.left)       # Turn left
        self.accept('d-repeat', self.right)      # Turn right

    def forward(self):
        self.car_controller.acceleration += Vec3(0.0,0.3,0.0)

    def left(self):
        self.car_controller.angular_velocity += Vec3(0.1,0.0,0.0)

    def right(self):
        self.car_controller.angular_velocity -= Vec3(0.1,0.0,0.0)

    def backward(self):
        self.car_controller.acceleration -= Vec3(0.0,0.3,0.0)


    def cameraFollowTask(self, task):
        self.camera.setPos(self.car.getX(), self.car.getY()-20, 6)
        self.camera.headsUp(self.car)

        return Task.cont

    def updateCarPositionTask(self, task):
        self.car_controller.updatePos()
        return Task.cont


app = MyApp()
app.run()

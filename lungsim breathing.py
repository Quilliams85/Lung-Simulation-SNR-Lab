import Sofa # type: ignore
import Sofa.Gui # type: ignore
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


lungScale = 20

#matplotlib plotter(not used yet)
class RealTimeForcePlotController(Sofa.Core.Controller):
    def __init__(self, node, lung, interval=100):
        super().__init__()
        self.node = node
        self.lung = lung
        self.interval = interval
        self.forces = []

        # Set up the matplotlib figure and axis
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-10, 10)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=self.interval, blit=True)

        plt.ion()
        plt.show()

    def update_plot(self, frame):
        if len(self.forces) > 0:
            ydata = np.array(self.forces)
            xdata = np.arange(len(ydata))
            self.line.set_data(xdata, ydata)
            self.ax.set_xlim(0, len(ydata))
            self.ax.set_ylim(ydata.min() - 1, ydata.max() + 1)
        return self.line,

    def onAnimateBeginEvent(self, event):
        force = self.lung.dofs.force.value
        self.forces.append(force)
        if len(self.forces) > 1000:  # Keep the list size reasonable
            self.forces.pop(0)
        plt.pause(0.001)


#breathing force, spring force, damper force class
class BreathingForceField(Sofa.Core.ForceFieldVec3d):
    def __init__(self, p=1, a=10,ks=1.0, kd=1.0, force_sum = 0.0, *args, **kwargs):
        Sofa.Core.ForceFieldVec3d.__init__(self, *args, **kwargs)
        self.addData('p', type='float', value=p, help='Period of breathing', group='Lung Motion Params')
        self.addData('a', type='float', value=a, help='Amplitude of breathing', group='Lung Motion Params')
        self.addData("ks", type="float", value=ks, help="The stiffness spring", group="Spring's Properties")                  
        self.addData("kd", type="float", value=kd, help="The damping spring", group="Spring's Properties")
        self.addData('force_sum', type='float', value=force_sum)

    def init(self):
        mstate = self.getContext().dofs
        self.initpos = mstate.position.array().copy()
        self.initpos += [0, 0, 0]
        self.k = np.zeros((1,1))
        self.f = []
        self.d = 0.5
        #self.ks = (self.p.value / (2 * np.pi))**2

    def addForce(self, m, out_force, pos, vel):
        with out_force.writeableArray() as wa:
            current_time = self.context.time.value
            dia_force = (self.a.value * np.sin(((np.pi * 2) / self.p.value) * current_time))
            #diaphragm simulated force
            damping_force = self.getContext().dofs.velocity * -1 * self.kd.value
            #damping
            if dia_force <= 0:
                #exhale
                wa[:] +=  ( (self.initpos-pos.value) * self.ks.value) + damping_force
                self.force_sum = np.linalg.norm(( (self.initpos-pos.value) * self.ks.value) + damping_force)
            else:
                #inhale
                wa[:] +=  damping_force + [0,0, dia_force] + ( (self.initpos-pos.value) * self.ks.value )
                self.force_sum = np.linalg.norm(damping_force + [0,0, dia_force] + ( (self.initpos-pos.value) * self.ks.value ))
                
    def addDForce(self, df, dx, params):
        pass


def main():
    root = Sofa.Core.Node("root")

    # Call the createScene function, as runSofa does
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Launch the GUI (qt or qglviewer)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)

    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


def config(rootNode):
    confignode = rootNode.addChild("Config")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Solver", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.StateContainer", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.MechanicalLoad", printLog=False)

    rootNode.addObject('DefaultAnimationLoop')


def createScene(rootNode):
    #add all plugins/prerequisites
    config(rootNode)

    lungRoot = rootNode.addChild('lungRoot')

    #full res visual mesh
    lungRoot.addObject('MeshOBJLoader', name="LiverSurface", filename="mesh/manifold.obj", scale =20, rotation=[90, 0,0])

    lung = lungRoot.addChild('lung')
    lung.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    lung.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    lung.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/manifold.msh", scale = lungScale)
    lung.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    lung.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    lung.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo", showPointIndices="0")
    lung.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    lung.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="1000", computeGlobalMatrix="0")
    lung.addObject('FixedConstraint', name="FixedConstraint", indices="1052 1452 574")
    
    #lung.addObject('FixedConstraint', name="FixedConstraint", indices="1342 547 1052 581")
    #lung.addObject('ConstantForceField', name='Gravity', totalForce = [0,-981.0,0])
    #lung.addObject('SurfacePressureForceField', pressure=100, pulseMode=False, pressureSpeed=0.005)

    lung.addObject(BreathingForceField(p=2, a=1, ks=2, kd=0.7, force_sum=0))

    #visual model
    visu = lung.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")



    rootNode.addObject('MeshOBJLoader', name="LiverSurface", filename="mesh/lung.obj", scale =20, rotation=[-90, 0,0])
    
    lobes = rootNode.addChild('lobes')
    lobes.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    lobes.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    lobes.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/lung.msh", scale = lungScale, rotation=[-90, 0,0])
    lobes.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    lobes.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    lobes.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo", showPointIndices="0")
    lobes.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    lobes.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="1000", computeGlobalMatrix="0")

    lobes.addObject(BreathingForceField(p=2, a=2, ks=2, kd=0.7, force_sum=0))

    visu = lobes.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    #rootNode.addObject(RealTimeForcePlotController(rootNode, lung, interval=100))


if __name__ == '__main__':
    main()
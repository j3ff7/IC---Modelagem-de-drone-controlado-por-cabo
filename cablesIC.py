# ===========================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Modelos usando elementos de cabo ANCF
# ===========================================================================
import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Um único elemento de cabo ANCF, com uma extremidade fixa
# ----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh):
        self.system = system
        beam_L = 0.1
        beam_diameter = 0.015

        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(beam_diameter)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.000)

        hnodeancf1 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, 0, -0.2), chrono.ChVector3d(1, 0, 0))
        hnodeancf2 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_L, 0, -0.2), chrono.ChVector3d(1, 0, 0))

        mesh.AddNode(hnodeancf1)
        mesh.AddNode(hnodeancf2)

        belementancf1 = fea.ChElementCableANCF()
        belementancf1.SetNodes(hnodeancf1, hnodeancf2)
        belementancf1.SetSection(msection_cable)
        mesh.AddElement(belementancf1)

        hnodeancf2.SetForce(chrono.ChVector3d(0, 3, 0))
        hnodeancf1.SetFixed(True)

        self.body = chrono.ChBodyEasyBox(0.1, 0.02, 0.02, 1000)
        self.body.SetPos(hnodeancf2.GetPos() + chrono.ChVector3d(0.05, 0, 0))
        system.Add(self.body)

        constraint_pos = fea.ChLinkNodeFrame()
        constraint_pos.Initialize(hnodeancf2, self.body)
        system.Add(constraint_pos)

        constraint_dir = fea.ChLinkNodeSlopeFrame()
        constraint_dir.Initialize(hnodeancf2, self.body)
        constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
        system.Add(constraint_dir)

    def PrintBodyPosition(self):
        print("Time:", self.system.GetChTime())
        print("  ", self.body.GetPos())

# ----------------------------------------------------------------------------
# Model2: Um feixe de 10 elementos de cabo ANCF, preso em uma ponta
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# Model2: Um feixe de 10 elementos, preso em uma ponta (com visual de junta)
# ----------------------------------------------------------------------------
class Model2:
    def __init__(self, system, mesh):

        msection_cable2 = fea.ChBeamSectionCable()

        msection_cable2.SetDiameter(0.015)

        msection_cable2.SetYoungModulus(0.001e9)

        msection_cable2.SetRayleighDamping(0.000)


        builder = fea.ChBuilderCableANCF()

        builder.BuildBeam(mesh, msection_cable2, 10,

        chrono.ChVector3d(0, 0, -0.1),

        chrono.ChVector3d(1, 0.5, -0.1))


        mtruss = chrono.ChBody()

        mtruss.SetFixed(True)

        system.Add(mtruss)

        first_node = builder.GetLastBeamNodes().front()

        last_node = builder.GetLastBeamNodes().back()


        # --- Restrição de junta (articulada) ---

        # Usamos ChLinkNodeFrame para uma conexão de "junta esférica"

        constraint_hinge = fea.ChLinkNodeFrame()

        constraint_hinge.Initialize(first_node, mtruss) # Conecta o nó ao corpo fixo

        system.Add(constraint_hinge)

        constraint_pos = fea.ChLinkNodeFrame()

        constraint_pos.Initialize(last_node, mtruss)

        system.Add(constraint_pos)


        msphere_visual = chrono.ChVisualShapeSphere(0.02) # Esfera com 2cm de raio

        constraint_hinge.AddVisualShape(msphere_visual)

        constraint_pos.AddVisualShape(msphere_visual) 
    

# ----------------------------------------------------------------------------
# Model3: Vários feixes e corpos conectados
# ----------------------------------------------------------------------------
class Model3:
    def __init__(self, system, mesh, n_chains=6):
        self.bodies = []
        self.system = system

        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.000)

        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)

        for j in range(n_chains):
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection_cable2, 1 + j,
                              chrono.ChVector3d(0, 0, -0.1 * j),
                              chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j))

            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.2, 0))

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            msphere = chrono.ChVisualShapeSphere(0.02)
            constraint_hinge.AddVisualShape(msphere)

            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)
            mbox.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox)

            constraint_pos = fea.ChLinkNodeFrame()
            constraint_pos.Initialize(builder.GetLastBeamNodes().back(), mbox)
            system.Add(constraint_pos)

            constraint_dir = fea.ChLinkNodeSlopeFrame()
            constraint_dir.Initialize(builder.GetLastBeamNodes().back(), mbox)
            constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

            # Segunda parte do feixe
            builder.BuildBeam(mesh, msection_cable2, 1 + (n_chains - j),
                              chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),
                              chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j))

            constraint_pos2 = fea.ChLinkNodeFrame()
            constraint_pos2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            system.Add(constraint_pos2)

            constraint_dir2 = fea.ChLinkNodeSlopeFrame()
            constraint_dir2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir2)

            self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000))
            self.bodies[j].SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(self.bodies[j])

            constraint_pos3 = fea.ChLinkNodeFrame()
            constraint_pos3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            system.Add(constraint_pos3)

            constraint_dir3 = fea.ChLinkNodeSlopeFrame()
            constraint_dir3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir3)

    def PrintBodyPositions(self):
        print("Time:", self.system.GetChTime())
        for body in self.bodies:
            print("  ", body.GetPos())



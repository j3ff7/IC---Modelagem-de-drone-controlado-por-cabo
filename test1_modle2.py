import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model2:
    def __init__(self, system, mesh):
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.001e9)
        msection_cable2.SetRayleighDamping(0.000)
        
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(mesh, msection_cable2, 5,
                          chrono.ChVector3d(0, 0, -0.1),
                          chrono.ChVector3d(1, 0, -0.1))
        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)
        
        first_node = builder.GetLastBeamNodes().front()
        last_node = builder.GetLastBeamNodes().back()
        
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(first_node, mtruss)
        system.Add(constraint_hinge)
        
        last_node_pos = last_node.GetPos()
        driver_body = chrono.ChBody()
        driver_body.SetPos(last_node_pos)
        driver_body.SetFixed(False)
        driver_body.SetMass(0.1)
        driver_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
        system.Add(driver_body)
        
        driver_shape = chrono.ChVisualShapeSphere(0.03)
        driver_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
        driver_body.AddVisualShape(driver_shape)
        
        # CORREÇÃO: Frame com orientação padrão (eixo Z apontando para cima)
        # Sem rotação, o motor move ao longo do eixo Z por padrão
        frame_de_movimento = chrono.ChFramed(last_node_pos, chrono.QUNIT)
        
        # Movimento senoidal: amplitude maior e frequência visível
        movimento_seno = chrono.ChFunctionSine(0, 0.3, 1.0)  # offset=0, amplitude=0.3m, freq=1Hz
        
        link_motor = chrono.ChLinkMotorLinearPosition()
        link_motor.Initialize(driver_body, mtruss, frame_de_movimento)
        link_motor.SetMotorFunction(movimento_seno)
        system.Add(link_motor)
        
        constraint_movel = fea.ChLinkNodeFrame()
        constraint_movel.Initialize(last_node, driver_body)
        system.Add(constraint_movel)
        
        msphere_visual = chrono.ChVisualShapeSphere(0.02)
        constraint_hinge.AddVisualShape(msphere_visual)
        constraint_movel.AddVisualShape(msphere_visual)
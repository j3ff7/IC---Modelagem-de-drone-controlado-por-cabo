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
        builder.BuildBeam(mesh, msection_cable2, 10,
        chrono.ChVector3d(0, 0, -0.1),
        chrono.ChVector3d(1, 0.5, -0.1))

        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)

        first_node = builder.GetLastBeamNodes().front()
        last_node = builder.GetLastBeamNodes().back()
        
        # ⭐⭐ CORREÇÃO: Guardar como atributo da classe usando self. ⭐⭐
        self.body_move = chrono.ChBody()  # Agora é self.body_move
        self.body_move.SetPos(last_node.GetPos())
        self.body_move.SetFixed(False)
        self.body_move.SetMass(0.1)
        system.Add(self.body_move)
        
        # Corpo móvel
        body_move = chrono.ChBody()
        body_move.SetPos(last_node.GetPos())
        body_move.SetFixed(False)
        body_move.SetMass(0.1)
        system.Add(body_move)
            
        # Primeira extremidade fixa
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(first_node, mtruss)
        system.Add(constraint_hinge)
        
        # Conexão da última extremidade
        constraint_last = fea.ChLinkNodeFrame()
        constraint_last.Initialize(last_node, body_move)
        system.Add(constraint_last)
        
        # Corpo de referência fixo
        body_ref = chrono.ChBody()
        body_ref.SetFixed(True)
        body_ref.SetPos(body_move.GetPos())
        system.Add(body_ref)
        
        # **CORREÇÃO: Motor para movimento VERTICAL**
        motor = chrono.ChLinkMotorLinearPosition()
        
        # Frame rotacionado para movimento VERTICAL (Y)
        # Rotacionar 90 graus no eixo Z para mudar X -> Y
        motor_frame = chrono.ChFramed(
            body_move.GetPos(), 
            chrono.Q_ROTATE_Z_TO_Y  # Ou chrono.QuatFromAngleZ(-m.pi/2)
        )
        
        motor.Initialize(body_move, body_ref, motor_frame)
        
        move_function = chrono.ChFunctionSine(0.3, 0.4)  # 30cm amplitude, 0.4Hz
        motor.SetMotionFunction(move_function)
        system.Add(motor)
        
        # Visualização
        msphere_visual = chrono.ChVisualShapeSphere(0.02)
        constraint_hinge.AddVisualShape(msphere_visual)
        constraint_last.AddVisualShape(msphere_visual)
        
        # Visualização para o corpo móvel - COR VERMELHA
        box_visual = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
        box_visual.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
        body_move.AddVisualShape(box_visual)
        
        # Visualização para referência fixa - COR AZUL
        ref_sphere = chrono.ChVisualShapeSphere(0.015)
        ref_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
        body_ref.AddVisualShape(ref_sphere)

        print("Motor configurado para movimento VERTICAL")

    def PrintBodyPositions(self):
        pass
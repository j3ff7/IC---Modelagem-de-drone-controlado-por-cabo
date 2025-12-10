import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pandas as pd

class Model2:
    def __init__(self, system, mesh):
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetDensity(1000)
        msection_cable2.SetYoungModulus(0.001e9)
        msection_cable2.SetRayleighDamping(0.000)

        self.builder = fea.ChBuilderCableANCF()
        self.builder.BuildBeam(mesh, msection_cable2, 10,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(1, 0.5, 0))

        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)

        first_node = self.builder.GetLastBeamNodes().front()
        last_node = self.builder.GetLastBeamNodes().back()
        
        self.start_pos = last_node.GetPos()
        
        vector_pos = last_node.GetPos()
        self.x0 = vector_pos.x
        self.y0 = vector_pos.y
        self.z0 = vector_pos.z
        
        self.body_move = chrono.ChBody()  # Agora é self.body_move
        self.body_move.SetPos(chrono.ChVector3d(self.x0,self.y0,self.z0))
        self.body_move.SetFixed(True)
        self.body_move.SetMass(0.1)
        system.Add(self.body_move)  
        
         # Corpo de referência fixo
        self.body_ref = chrono.ChBody()
        self.body_ref.SetFixed(True)
        self.body_ref.SetPos(self.start_pos)
        system.Add(self.body_ref)
        
            
        # Primeira extremidade fixa
        self.constraint_hinge = fea.ChLinkNodeFrame()
        self.constraint_hinge.Initialize(first_node, mtruss)
        system.Add(self.constraint_hinge)
        
        # Conexão da última extremidade
        self.constraint_last = fea.ChLinkNodeFrame()
        self.constraint_last.Initialize(last_node, self.body_move)
        system.Add(self.constraint_last)
        
       
        #motor = chrono.ChLinkMotorLinearPosition()
        #motor_frame = chrono.ChFramed(
        #    self.body_move.GetPos(), 
        #   chrono.Q_ROTATE_Z_TO_Y  
        #)
        
        #motor.Initialize(self.body_move, self.body_ref, motor_frame)
        
        #move_function = chrono.ChFunctionSine(1, 0.4)  # 30cm amplitude, 0.4Hz
        #motor.SetMotionFunction(move_function)
        #system.Add(motor)
        
        #func = self.create_square_motion(side_length=0.5, cycle_time=8)
        
        #motor = chrono.ChLinkMotorLinearPosition()
        #motor_frame = chrono.ChFramed(self.start_pos,
        #                                chrono.Q_ROTATE_Z_TO_X)
        #motor.Initialize(self.body_move, self.body_ref, motor_frame)
        #motor.SetMotionFunction(func)
        #system.Add(motor)
        
        # Visualização
        msphere_visual = chrono.ChVisualShapeSphere(0.02)
        self.constraint_hinge.AddVisualShape(msphere_visual)
        self.constraint_last.AddVisualShape(msphere_visual)
        
        # Visualização para o corpo móvel - COR VERMELHA
        box_visual = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
        box_visual.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
        self.body_move.AddVisualShape(box_visual)
        
        # Visualização para referência fixa - COR AZUL
        ref_sphere = chrono.ChVisualShapeSphere(0.015)
        ref_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
        self.body_ref.AddVisualShape(ref_sphere)
        
        self.simulation_time = 0.0

        print("Motor configurado para movimento em Quadrado")
        

    def update_motion(self, dt):
        
        self.simulation_time += dt

        L = 0.5
        cycle_time = 1
        
        section_time = cycle_time/4
        
        speed = L / section_time
        
        t = self.simulation_time % cycle_time
        
        dz = 0
        dy = 0 
        
        if t < section_time:
            dz = speed*t
            dy = 0
            
        elif t < (2*section_time):
            local_t = t - section_time
            dz = L
            dy = speed * local_t
            
        elif t < (3 * section_time):
            local_t = t - (2 * section_time)
            dz = L - (speed * local_t) 
            dy = L
            
        else:
            local_t = t - (3 * section_time)
            dz = 0.0
            dy = L - (speed * local_t) 
            
        new_pos = chrono.ChVector3d(self.x0,
                                    self.y0 + dy,
                                    self.z0 + dz)
        
        self.body_move.SetPos(new_pos)
            
    def PrintBodyPosition(self, time=None):
        # Posição do corpo móvel
        body_pos = self.body_move.GetPos()
        
        #Tensão nas extremidades do cabo
        react_movel = self.constraint_last.GetReaction2()
        force_vec_movel = react_movel.force
        tensao_movel = force_vec_movel.Length()
        
        react_fixa = self.constraint_hinge.GetReaction2()
        force_vec_fixa = react_fixa.force
        tensao_fixa = force_vec_fixa.Length()
        
        # Ângulos do cabo
        nodes = self.builder.GetLastBeamNodes()
        if len(nodes) >= 2:
            last_node = nodes[-1]
            penultimate_node = nodes[-2]
            last_segment = last_node.GetPos() - penultimate_node.GetPos()
            last_segment.Normalize()
            
            pitch_deg = m.asin(last_segment.y) * 180/m.pi
            yaw_deg = m.atan2(last_segment.x, last_segment.z) * 180/m.pi
        else:
            pitch_deg = 0.0
            yaw_deg = 0.0
        
        if time is not None:
            print(f"{time:6.2f}s | CORPO:({body_pos.x:5.3f},{body_pos.y:5.3f},{body_pos.z:5.3f}) | CABO:Pitch:{pitch_deg:6.1f}° Yaw:{yaw_deg:6.1f}° | Tensão: Móvel: {tensao_movel:.2f}N Fixa:{tensao_fixa:.2f}N")
        else:
            print(f"Corpo: ({body_pos.x:.3f}, {body_pos.y:.3f}, {body_pos.z:.3f}) | Cabo: Pitch:{pitch_deg:.1f}° Yaw:{yaw_deg:.1f}° | Tensão: Móvel: {tensao_movel:.2f}N Fixa:{tensao_fixa:.2f}N")
        
        return {
            "Tempo": time if time else 0.0,
            "Pos_X": body_pos.x,
            "Pos_Y": body_pos.y,
            "Pos_Z": body_pos.z,
            "Pitch": pitch_deg,
            "Yaw": yaw_deg,
            "Tensao_Movel": tensao_movel,
            "Tensao_Fixa": tensao_fixa
        }
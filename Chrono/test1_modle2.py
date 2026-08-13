import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pandas as pd

class Model2:
    def __init__(self, system, mesh):
        
        cable_diameter = 0.015
        cable_density = 1000
        cable__young = 70e9
        
        self.initial_pos = 2
        self.final_pos = 1
        self.move_duration = 5
        
        self.start_pos = chrono.ChVector3d(0, 1, 0)
        self.end_pos = chrono.ChVector3d(self.initial_pos, 1, 0)
        
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(cable_diameter)
        msection_cable2.SetDensity(cable_density)
        msection_cable2.SetYoungModulus(cable__young)
        msection_cable2.SetRayleighDamping(0.000)
        
        elements = 100

        self.builder = fea.ChBuilderCableANCF()
        self.builder.BuildBeam(mesh, msection_cable2, elements,
        self.start_pos,
        self.end_pos)
        
        contact_cable = chrono.ChContactMaterialSMC()
        contact_cable.SetFriction(0.4)
        contact_cable.SetRestitution(0.1)
        contact_cable.SetYoungModulus(1e7)
        contact_cable.SetPoissonRatio(0.3)
        
        self.contactcloud = fea.ChContactSurfaceNodeCloud(contact_cable, mesh)
        mesh.AddContactSurface(self.contactcloud)
        
        beam_nodes = self.builder.GetLastBeamNodes() 
        contact_radius = cable_diameter * 1.5   
        for node in beam_nodes:
            self.contactcloud.AddNode(node, contact_radius)
        
        #Chão
        contact_floor = chrono.ChContactMaterialSMC()
        contact_floor.SetFriction(0.5)
        contact_floor.SetRestitution(0.1)  
        contact_floor.SetYoungModulus(1e9)  
        contact_floor.SetPoissonRatio(0.3)  
        mtruss = chrono.ChBodyEasyBox(25, 0.1, 25, 1000, True, True, contact_floor)
        mtruss.SetPos(chrono.ChVector3d(0.5,-0.05,0))
        mtruss.SetFixed(True)
        mtruss.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.6))  # Cor azulada
        system.Add(mtruss)
        

        first_node = self.builder.GetLastBeamNodes().front()
        last_node = self.builder.GetLastBeamNodes().back()
        
        self.body_move = chrono.ChBody()  # Agora é self.body_move
        self.body_move.SetPos(self.end_pos)
        self.body_move.SetFixed(True)
        self.body_move.SetMass(0.1)
        system.Add(self.body_move)  
        
         # Corpo de referência fixo
        self.body_ref = chrono.ChBody()
        self.body_ref.SetFixed(True)
        self.body_ref.SetPos(self.start_pos)
        system.Add(self.body_ref)
        
        #self.body_last = chrono.ChBody()
        #self.body_last.SetFixed(True)  # Corpo fixo
        #self.body_last.SetPos(last_node.GetPos())
        #system.Add(self.body_last)
            
        # Primeira extremidade fixa
        self.constraint_hinge = fea.ChLinkNodeFrame()
        self.constraint_hinge.Initialize(first_node, self.body_ref)
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
        
        # Visualização para referência fixa - COR AZUL
        ref_sphere = chrono.ChVisualShapeSphere(0.015)
        ref_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
        self.body_ref.AddVisualShape(ref_sphere)
        
        # Visualização para última extremidade fixa - TAMBÉM AZUL
        move_sphere = chrono.ChVisualShapeSphere(0.015)
        move_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))  # Azul
        self.body_move.AddVisualShape(move_sphere)
        
        self.simulation_time = 0.0

    def update_motion(self, dt):
        
        self.simulation_time += dt
        
        if self.simulation_time <= self.move_duration:
            time_elapsed = self.simulation_time/self.move_duration
            factor = 0.5*(1-m.cos(time_elapsed*m.pi))
            
            new_pos = self.initial_pos + (self.final_pos - self.initial_pos) * factor
            
            current_pos = self.body_move.GetPos()
            current_pos.x = new_pos
            
            self.body_move.SetPos(current_pos)        
    
    def PrintBodyPosition(self, time=None):
        
        #Tensão nas extremidades do cabo
        react_last = self.constraint_last.GetReaction2()
        force_vec_last = react_last.force
        tensao_last = force_vec_last.Length()
        
        react_fixa = self.constraint_hinge.GetReaction2()
        force_vec_fixa = react_fixa.force
        tensao_fixa = force_vec_fixa.Length()
        
        # Ângulos do cabo
        nodes = self.builder.GetLastBeamNodes()
        length = 0  
        for i in range(len(nodes) -1):
            pos_a = nodes[i].GetPos()
            pos_b = nodes[i+1].GetPos()
            
            dist = (pos_b - pos_a).Length()
            length += dist
            
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
        
        pos_atual = self.body_move.GetPos()
        
        print(f"Comprimento do cabo: {length}m")
        if time is not None:
            print(f"{time:6.2f}s | Início:({self.start_pos.x:5.3f},{self.start_pos.y:5.3f},{self.start_pos.z:5.3f}) | Fim:({pos_atual.x:5.3f},{pos_atual.y:5.3f},{pos_atual.z:5.3f}) | Cabo:Pitch:{pitch_deg:6.1f}° Yaw:{yaw_deg:6.1f}° | Tensão: Início: {tensao_fixa:.2f}N Fim:{tensao_last:.2f}N | Comprimento:({length:.3f}")
        else:
            print(f"Início: ({self.start_pos.x:.3f}, {self.start_pos.y:.3f}, {self.start_pos.z:.3f}) | Fim: ({pos_atual.x:.3f}, {pos_atual.y:.3f}, {pos_atual.z:.3f}) | Cabo: Pitch:{pitch_deg:.1f}° Yaw:{yaw_deg:.1f}° | Tensão: Início: {tensao_fixa:.2f}N Fim:{tensao_last:.2f}N")
        
        return {
            "Tempo": time if time else 0.0,
            "Start_X": self.start_pos.x,
            "Start_Y": self.start_pos.y,
            "Start_Z": self.start_pos.z,
            "End_X": pos_atual.x,      
            "End_Y": pos_atual.y,       
            "End_Z": pos_atual.z,      
            "Pitch": pitch_deg,        
            "Yaw": yaw_deg,             
            "Tensao_Start": tensao_fixa,
            "Tensao_End": tensao_last}
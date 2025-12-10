import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
from test1_modle2 import Model2
import pandas as pd


print("Cable with moving end - PyChrono 9.0.0")

# Create system
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model
model = Model2(sys, mesh)
sys.Add(mesh)

# Simple visualization for PyChrono 9.0.0
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.01, 0.01)
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

# Visualização adicional para nós
vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Moving Cable Test')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.5, 1.5))
vis.AddTypicalLights()

# Solver
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.SetVerbose(False)

step = 0
simulation_time = 0.0
print_interval = 25
time_step = 0.01

print("Iniciando simulação...")
print("Tempo(s) | Pos_X | Pos_Y | Pos_Z | Roll° | Pitch° | Yaw° | Tensão_M | Tensão_F")
print("-" * 55)

lista_dados = []

while vis.Run():
    # Atualiza visualização
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    model.update_motion(time_step)
    
    # Avança simulação física
    sys.DoStepDynamics(time_step)
    
    # Atualiza contadores
    simulation_time += time_step
    step += 1
    
    if step % print_interval == 0:
        model.PrintBodyPosition(simulation_time)
        dados = model.PrintBodyPosition(simulation_time)
        lista_dados.append(dados)
        
df = pd.DataFrame(lista_dados)
df.to_csv("resultados_simulacao.csv", index=False)
print("Simulação finalizada!")
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
from test1_modle2 import Model2
import pandas as pd


print("Cable with moving end - PyChrono 9.0.0")

# Create system
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

#floor = chrono.ChBodyEasyBox(10, 0.2, 10, 1000)
#floor.SetPos(chrono.ChVector3d(0, 0, 0))
#floor.SetFixed(True)
#floor.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.9, 0.9))
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
time_step = 0.005


print("Iniciando simulação...")
print("Tempo(s) | Start_X | Start_Y | Start_Z | End_X | End_Y | End_Z | Pitch° | Yaw° | Tensão_Início | Tensão_Fim")
print("-" * 100)

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
        dados = model.PrintBodyPosition(simulation_time)
        lista_dados.append(dados)
        # Imprime dados formatados
        print(f"{simulation_time:6.2f}s | Start:({dados['Start_X']:5.3f},{dados['Start_Y']:5.3f},{dados['Start_Z']:5.3f}) | End:({dados['End_X']:5.3f},{dados['End_Y']:5.3f},{dados['End_Z']:5.3f}) | Pitch:{dados['Pitch']:6.1f}° | Yaw:{dados['Yaw']:6.1f}° | Tensão_S:{dados['Tensao_Start']:6.2f}N | Tensão_E:{dados['Tensao_End']:6.2f}N")

# Salva os dados em CSV após a simulação
if lista_dados:
    df = pd.DataFrame(lista_dados)
    df.to_csv("resultados_cabo_fixo.csv", index=False)
    print(f"\nDados salvos em 'resultados_cabo_fixo.csv' ({len(df)} registros)")
else:
    print("Nenhum dado coletado durante a simulação")

print("Simulação finalizada!")
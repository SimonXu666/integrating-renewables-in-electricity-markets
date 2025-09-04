import gurobipy as gp
from gurobipy import GRB


lin_cap = 100
load1 = 40
load2 = 100

vertice , deltaWs1, deltaWs2 = gp . multidict ({
'A': [10,8],
'B': [4,20],
'C': [-4,20],
'D': [-10,8],
'E': [-10,-8],
'F': [-4,-20],
'G': [4,-20],
'H': [10,-8]})

model = gp.Model('an adaptive robust optimization approach to energy and reserve dispatch')

# 创建变量
P1 = model.addVar(ub=50,name='P1')
P2 = model.addVar(ub=110,name='P2')
P3 = model.addVar(ub=100,name='P3')
P12 = model.addVar(lb=-lin_cap,ub=lin_cap,name='P12')
P12b = model.addVars(vertice,lb=-lin_cap,ub=lin_cap,name='P12b')
costb = model.addVars(vertice,name='costb')

worst_cost = model.addVar(name='worst_cost')

R1U = model.addVar(name='R1U')
R2U = model.addVar(name='R2U')
R3U = model.addVar(name='R3U')
R1D = model.addVar(name='R1D')
R2D = model.addVar(name='R2D')
R3D = model.addVar(name='R3D')

# 设置充分低的下限，非0下限
r1 = model.addVars(vertice,lb=-200,name='r1')
r2 = model.addVars(vertice,lb=-200,name='r2')
r3 = model.addVars(vertice,lb=-200,name='r3')

L1shed = model.addVars(vertice,lb=0,ub=40,name='L1shed')
L2shed = model.addVars(vertice,lb=0,ub=100,name='L2shed')

W1spill = model.addVars(vertice,name='W1spill')
W2spill = model.addVars(vertice,name='W2spill')


# 设置目标
obj_da = 10*P1+30*P2+35*P3+16*R1U+15*R1D+13*R2U+12*R2D+10*R3U+9*R3D
model.addConstrs((costb[v]==10*r1[v]+30*r2[v]+35*r3[v]+200*(L1shed[v]+L2shed[v])\
                  for v in vertice),name='costb_equation')
model.setObjective(obj_da+worst_cost,GRB.MINIMIZE)

# 添加约束
model.addConstr(P1+P2-P12+15==load1,name='bus1_day_ahead_balance')
model.addConstr(P12+P3+30==load2,name='bus2_day_ahead_balance')

# 鲁棒的转化，只要满足不确定支撑集的顶点即可
model.addConstrs((worst_cost>=costb[v] for v in vertice),name='robust_beta')

model.addConstrs((r1[v]+r2[v]-P12b[v]+P12+L1shed[v]+deltaWs1[v]-W1spill[v]==0\
                  for v in vertice),name='bus1_balance')
model.addConstrs((r3[v]+P12b[v]-P12+L2shed[v]+deltaWs2[v]-W2spill[v]==0\
                  for v in vertice),name='bus2_balance')

model.addConstr(P1+R1U<=50,name='day_ahead_unit1_up_constraint')
model.addConstr(P1-R1D>=0,name='day_ahead_unit1_down_constraint')
model.addConstr(P2+R2U<=110,name='day_ahead_unit2_up_constraint')
model.addConstr(P2-R2D>=0,name='day_ahead_unit2_down_constraint')
model.addConstr(P3+R3U<=100,name='day_ahead_unit3_up_constraint')
model.addConstr(P2-R2D>=0,name='day_ahead_unit3_down_constraint')

model.addConstrs((-R1D<=r1[v] for v in vertice),name='unit1_up_constraint')
model.addConstrs((r1[v]<=R1U for v in vertice),name='unit1_down_constraint')
model.addConstrs((-R2D<=r2[v] for v in vertice),name='unit2_up_constraint')
model.addConstrs((r2[v]<=R2U for v in vertice),name='unit2_down_constraint')
model.addConstrs((-R3D<=r3[v] for v in vertice),name='unit3_up_constraint')
model.addConstrs((r3[v]<=R3U for v in vertice),name='unit3_down_constraint')

model.write('RO.lp')

model.optimize()

for v in model.getVars():
    print(v.varName, v.x)
# 2882, 720
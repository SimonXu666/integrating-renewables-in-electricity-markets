import gurobipy as gp
from gurobipy import GRB

lin_cap = 100
load1 = 40
load2 = 100

model = gp.Model('features of the settlement scheme')

# 创建变量
P1 = model.addVar(ub=50,name='P1')
P2 = model.addVar(ub=110,name='P2')
P3 = model.addVar(ub=100,name='P3')

P12 = model.addVar(lb=-lin_cap,ub=lin_cap,name='P12')
P12h = model.addVar(lb=-lin_cap,ub=lin_cap,name='P12h')
P12l = model.addVar(lb=-lin_cap,ub=lin_cap,name='P12l')

r2hU = model.addVar(ub=20,name='r2hU')
r2hD = model.addVar(ub=30,name='r2hD')
r3hU = model.addVar(ub=100,name='r3hU')
r3hD = model.addVar(ub=100,name='r3hD')

r2lU = model.addVar(ub=20,name='r2lU')
r2lD = model.addVar(ub=30,name='r2lD')
r3lU = model.addVar(ub=100,name='r3lU')
r3lD = model.addVar(ub=100,name='r3lD')

L1hshed = model.addVar(ub=load1,name='L1hshed')
L2hshed = model.addVar(ub=load2,name='L2hshed')
L1lshed = model.addVar(ub=load1,name='L1lshed')
L2lshed = model.addVar(ub=load2,name='L2lshed')

Ws = model.addVar(name='Ws')
Whspill = model.addVar(lb=0,ub=50,name='Whspill')
Wlspill = model.addVar(lb=0,ub=10,name='Wlspill')

# 设置目标
obj = 10*P1+30*P2+35*P3+\
        0.6*(50*r2hU-20*r2hD+40*r3hU-34*r3hD+200*(L1hshed+L2hshed))+\
        0.4*(50*r2lU-20*r2lD+40*r3lU-34*r3lD+200*(L1lshed+L2lshed))
model.setObjective(obj,GRB.MINIMIZE)

# 添加约束
model.addConstr(P1+P2-P12+Ws==load1,name='bus1_day_ahead_balance')
model.addConstr(P12+P3==load2,name='bus2_day_ahead_balance')

model.addConstr(r2hU-r2hD-P12h+P12+L1hshed+50-Ws-Whspill==0,name='bus1_high_scenario_balance')
model.addConstr(r3hU-r3hD+P12h-P12+L2hshed==0,name='bus2_high_scenario_balance')

model.addConstr(r2lU-r2lD-P12l+P12+L1lshed+10-Ws-Wlspill==0,name='bus1_low_scenario_balance')
model.addConstr(r3lU-r3lD+P12l-P12+L2lshed==0,name='bus2_low_scenario_balance')

model.addConstr(P2+r2hU-r2hD>=0,name='high_scenario_unit2_up_constraint')
model.addConstr(P2+r2hU-r2hD<=110,name='high_scenario_unit2_down_constraint')
model.addConstr(P3+r3hU-r3hD>=0,name='high_scenario_unit3_up_constraint')
model.addConstr(P3+r3hU-r3hD<=100,name='high_scenario_unit3_down_constraint')

model.addConstr(P2+r2lU-r2lD>=0,name='low_scenario_unit2_up_constraint')
model.addConstr(P2+r2lU-r2lD<=110,name='low_scenario_unit2_down_constraint')
model.addConstr(P3+r3lU-r3lD>=0,name='low_scenario_unit3_up_constraint')
model.addConstr(P3+r3lU-r3lD<=100,name='low_scenario_unit3_down_constraint')

model.write('features of the settlement scheme.lp')
model.optimize()
print('optimal var:')
for v in model.getVars():
    print(v.varName, v.x)
print('\ndual variables of constr:')
for c in model.getConstrs():
    print(c.ConstrName, c.Pi)


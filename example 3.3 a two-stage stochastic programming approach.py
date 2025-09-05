'''
Juan M. Morales
Book: Intergrating Renewables in Electricity Markets
Example 3.3, the complete model formulation is as follows
rU and rL are integrated into one variable
minimiize   10P1+30P2+35P3+16R1U+15R1D+13R2U+12R2D+10R3U+9R3D
            +0.6[10r1h+30r2h+35r3h+200(L1hshed+L2hshed)] 
            +0.4[10r1l+30r2l+35r3l+200(L1lshed+L2lshed)]
            
subject to  P1+P2+P3+Ws=140
            -100<=P3-100<=100
            P1+R1U<=50
            P1-R1D>=0
            P2+R2U<=110
            P2-R2D>=0
            P3+R3U<=100
            P2-R2D>=0
            
high scenario
            r1h+r2h+r3h+L1hshed+L2hshed+50-Ws-Whspill=0
            -100<=P3+r3h+L2hshed-100<=100
            -R1D<=r1h<=R1U
            -R2D<=r2h<=R2U
            -R3D<=r3h<=R3U
            Whspill<=50
            L1hshed<=40
            L2hshed<=100
            
low scenario
            r1l+r2l+r3l+L1lshed+L2lshed+10-Ws-Wlspill=0
            -100<=P3+r3l+L2lshed-100<=100
            -R1D<=r1l<=R1U
            -R2D<=r2l<=R2U
            -R3D<=r3l<=R3U
            Whspill<=10
            L1hshed<=40
            L2hshed<=100
            
non-negative
            P1,P2,P3,Ws,R1U,R1D,R2U,R2D,R3U,R3D
            r1h,r2h,r3h,r1l,r2l,r3l
            L1hshed,L2hshed,L1lshed,L2lshed,Ws
'''

import gurobipy as gp
from gurobipy import GRB


model = gp.Model('A_two-stage_stochastic_programming')

transmission_capacity = 100

# 创建变量
P1 = model.addVar(lb=0,name='P1')
P2 = model.addVar(lb=0,name='P2')
P3 = model.addVar(lb=100-transmission_capacity,name='P3')

R1U = model.addVar(lb=0,name='R1U')
R2U = model.addVar(lb=0,name='R2U')
R3U = model.addVar(lb=0,name='R3U')
R1D = model.addVar(lb=0,name='R1D')
R2D = model.addVar(lb=0,name='R2D')
R3D = model.addVar(lb=0,name='R3D')

# 注意！！如果r这里不加入下边界的约束，则默认下界为0.则求出来的结果是2660
# P1=50,P2=40,R3U=40,r3L=40,Ws=50,obj=2660
r1h = model.addVar(lb=-200,name='r1h')
r2h = model.addVar(lb=-200,name='r2h')
r3h = model.addVar(lb=-200,name='r3h')
r1l = model.addVar(lb=-200,name='r1l')
r2l = model.addVar(lb=-200,name='r2l')
r3l = model.addVar(lb=-200,name='r3l')

L1hshed = model.addVar(lb=0,ub=40,name='L1hshed')
L2hshed = model.addVar(lb=0,ub=100,name='L2hshed')
L1lshed = model.addVar(lb=0,ub=40,name='L1lshed')
L2lshed = model.addVar(lb=0,ub=100,name='L2lshed')

Ws = model.addVar(lb=0,name='Ws')
Whspill = model.addVar(lb=0,ub=50,name='Whspill')
Wlspill = model.addVar(lb=0,ub=10,name='Wlspill')

# 设置目标
obj_da = 10*P1+30*P2+35*P3+16*R1U+15*R1D+13*R2U+12*R2D+10*R3U+9*R3D
obj_h = 0.6*(10*r1h+30*r2h+35*r3h+200*(L1hshed+L2hshed))
obj_l = 0.4*(10*r1l+30*r2l+35*r3l+200*(L1lshed+L2lshed))
model.setObjective(obj_da+obj_h+obj_l,GRB.MINIMIZE)

# 添加约束
model.addConstr(P1+P2+P3+Ws==140,name='day_ahead_balance')
model.addConstr(P3<=100+transmission_capacity,name='day_ahead_transmission_constraint') # 200
model.addConstr(P1+R1U<=50,name='day_ahead_unit1_up_constraint')
model.addConstr(P1-R1D>=0,name='day_ahead_unit1_down_constraint')
model.addConstr(P2+R2U<=110,name='day_ahead_unit2_up_constraint')
model.addConstr(P2-R2D>=0,name='day_ahead_unit2_down_constraint')
model.addConstr(P3+R3U<=100,name='day_ahead_unit3_up_constraint')
model.addConstr(P2-R2D>=0,name='day_ahead_unit3_down_constraint')

model.addConstr(r1h+r2h+r3h+L1hshed+L2hshed+50-Ws-Whspill==0,name='high_scenario_balance')
model.addConstr(P3+r3h+L2hshed<=100+transmission_capacity,name='high_scenario_transmission_up_constraint') # 200
model.addConstr(P3+r3h+L2hshed>=100-transmission_capacity,name='high_scenario_transmission_down_constraint') # 0

model.addConstr(r1l+r2l+r3l+L1lshed+L2lshed+10-Ws-Wlspill==0,name='low_scenario_balance')
model.addConstr(P3+r3l+L2lshed<=100+transmission_capacity,name='low_scenario_transmission_up_constraint') # 200
model.addConstr(P3+r3l+L2lshed>=100-transmission_capacity,name='low_scenario_transmission_down_constraint') # 0

model.addConstr(-R1D<=r1h,name='high_scenario_unit1_up_constraint')
model.addConstr(r1h<=R1U,name='high_scenario_unit1_down_constraint')
model.addConstr(-R2D<=r2h,name='high_scenario_unit2_up_constraint')
model.addConstr(r2h<=R2U,name='high_scenario_unit2_down_constraint')
model.addConstr(-R3D<=r3h,name='high_scenario_unit3_up_constraint')
model.addConstr(r3h<=R3U,name='high_scenario_unit3_down_constraint')

model.addConstr(-R1D<=r1l,name='low_scenario_unit1_up_constraint')
model.addConstr(r1l<=R1U,name='low_scenario_unit1_down_constraint')
model.addConstr(-R2D<=r2l,name='low_scenario_unit2_up_constraint')
model.addConstr(r2l<=R2U,name='low_scenario_unit2_down_constraint')
model.addConstr(-R3D<=r3l,name='low_scenario_unit3_up_constraint')
model.addConstr(r3l<=R3U,name='low_scenario_unit3_down_constraint')

model.write('A_two-stage_stochastic_programming.lp')

model.optimize()
# P1=50,P2=40,P3=40,R3D=40,r3h=-40,Ws=10,obj=2620

for v in model.getVars():
    print(v.varName, v.x)
    
'''
# another version
model = gp.Model('A_two-stage_stochastic_programming')

Ps, Pmax, C, CRU, CRD = gp.multidict ({
'P1': [50,10,16,15],
'P2': [110,30,13,12],
'P3': [100,35,10,9]})

buses, load = gp.multidict ({
'bus1': 40,
'bus2': 100})

P = model.addVars(Ps,name='P')
RU = model.addVars(Ps,name='RU')
RD = model.addVars(Ps,name='RD')

P12 = model.addVar(lb=-100,ub=100,name='P12')
P12h = model.addVar(lb=-100,ub=100,name='P12h')
P12l = model.addVar(lb=-100,ub=100,name='P12l')

rh = model.addVars(Ps,lb=-200,name='rh')
rl = model.addVars(Ps,lb=-200,name='rl')

Lhshed = model.addVars(buses,lb=0,ub=load,name='Lhshed')
Llshed = model.addVars(buses,lb=0,ub=load,name='Llshed')

Ws = model.addVar(lb=0,name='Ws')
Whspill = model.addVar(lb=0,ub=50,name='Whspill')
Wlspill = model.addVar(lb=0,ub=10,name='Wlspill')

# 设置目标
obj_da = gp.quicksum(C[p]*P[p]+CRU[p]*RU[p]+CRD[p]*RD[p] for p in Ps)
obj_h = 0.6*(gp.quicksum(C[p]*rh[p] for p in Ps)+200*(gp.quicksum(Lhshed[bus] for bus in buses)))
obj_l = 0.4*(gp.quicksum(C[p]*rl[p] for p in Ps)+200*(gp.quicksum(Llshed[bus] for bus in buses)))
model.setObjective(obj_da+obj_h+obj_l,GRB.MINIMIZE)

# 添加约束
model.addConstr(P['P1']+P['P2']-P12+Ws==load['bus1'],name='bus1_day_ahead_balance')
model.addConstr(P12+P['P3']==load['bus2'],name='bus2_day_ahead_balance')
model.addConstr(rh['P1']+rh['P2']-P12h+P12+Lhshed['bus1']+50-Ws-Whspill==0,name='high_scenario_bus1_balance')
model.addConstr(rh['P3']+P12h-P12+Lhshed['bus2']==0,name='high_scenario_bus2_balance')
model.addConstr(rl['P1']+rl['P2']-P12l+P12+Llshed['bus1']+10-Ws-Wlspill==0,name='low_scenario_bus1_balance')
model.addConstr(rl['P3']+P12l-P12+Llshed['bus2']==0,name='low_scenario_bus2_balance')

model.addConstrs((P[p]+RU[p]<=Pmax[p] for p in Ps),name='day_ahead_up_constraint')
model.addConstrs((P[p]+RU[p]>=0 for p in Ps),name='day_ahead_down_constraint')

model.addConstrs((-RD[p]<=rh[p] for p in Ps),name='high_scenario_down_constraint')
model.addConstrs((rh[p]<=RU[p] for p in Ps),name='high_scenario_up_constraint')
model.addConstrs((-RD[p]<=rl[p] for p in Ps),name='low_scenario_down_constraint')
model.addConstrs((rl[p]<=RU[p] for p in Ps),name='low_scenario_up_constraint')

model.write('A_two-stage_stochastic_programming.lp')

model.optimize()
# P1=50,P2=40,P3=40,R3D=40,r3h=-40,Ws=10,obj=2620

for v in model.getVars():
    print(v.varName, v.x)
    
'''

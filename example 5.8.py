
import gurobipy as gp
from gurobipy import GRB

model = gp.Model('A_two-stage_stochastic_programming')

# 基础参数

Ps, Pmin, Pmax, C, CRU, CRD = gp.multidict({
'P1': [0,50,10,16,15],
'P2': [0,110,30,13,12],
'P3': [0,100,35,10,9]})

buses = ['bus1','bus2']

load = {
('bus1',0): 40,
('bus1',1): 25,
('bus1',2): 45,
('bus2',0): 100,
('bus2',1): 80,
('bus2',2): 95}

Pwh = {0:50,1:65,2:35}
Pwl = {0:10,1:30,2:15}





line_capacity = 100

# 创建变量

flowd = model.addVars(range(3),ub=20,name='flowd')
rflowdh = model.addVars(range(3),name='rflowdh')
rflowdl = model.addVars(range(3),name='rflowdl')
flowc = model.addVars(range(3),ub=20,name='flowd')
rflowch = model.addVars(range(3),name='rflowdh')
rflowcl = model.addVars(range(3),name='rflowdl')


P = model.addVars(Ps,range(3),name='P')
RU = model.addVars(Ps,range(3),name='RU')
RD = model.addVars(Ps,range(3),name='RD')

P12 = model.addVars(range(3),lb=-line_capacity,ub=line_capacity,name='P12')
P12h = model.addVars(range(3),lb=-line_capacity,ub=line_capacity,name='P12h')
P12l = model.addVars(range(3),lb=-line_capacity,ub=line_capacity,name='P12l')

rh = model.addVars(Ps,range(3),lb=-200,name='rh')
rl = model.addVars(Ps,range(3),lb=-200,name='rl')

Lhshed = model.addVars(buses,range(3),ub=load,name='Lhshed')
Llshed = model.addVars(buses,range(3),ub=load,name='Llshed')

Ws = model.addVars(range(3),name='Ws')
Whspill = model.addVars(range(3),ub=Pwh,name='Whspill')
Wlspill = model.addVars(range(3),ub=Pwl,name='Wlspill')


# 设置目标
obj_da = gp.quicksum(C[p]*P[p,t]+CRU[p]*RU[p,t]+CRD[p]*RD[p,t] for p in Ps for t in range(3))
obj_h = 0.6*(gp.quicksum(C[p]*rh[p,t] for p in Ps for t in range(3))+\
             200*(gp.quicksum(Lhshed[bus,t] for bus in buses for t in range(3))))
obj_l = 0.4*(gp.quicksum(C[p]*rl[p,t] for p in Ps for t in range(3))+\
             200*(gp.quicksum(Llshed[bus,t] for bus in buses for t in range(3))))

model.setObjective(obj_da+obj_h+obj_l,GRB.MINIMIZE)

# 添加约束
model.addConstrs((P['P1',t]+P['P2',t]-P12[t]+Ws[t]==load['bus1',t] for t in range(3)),name='bus1_day_ahead_balance')
model.addConstrs((P12[t]+flowd[t]*0.8-flowc[t]*1.2+P['P3',t]==load['bus2',t] for t in range(3)),name='bus2_day_ahead_balance')

model.addConstrs((rh['P1',t]+rh['P2',t]-P12h[t]+P12[t]+Lhshed['bus1',t]+Pwh[t]-Ws[t]-Whspill[t]==0\
                  for t in range(3)),name='high_scenario_bus1_balance')
model.addConstrs((rh['P3',t]+rflowdh[t]*0.8-rflowch[t]*1.2+P12h[t]-P12[t]+Lhshed['bus2',t]==0\
                  for t in range(3)),name='high_scenario_bus2_balance')

model.addConstrs((rl['P1',t]+rl['P2',t]-P12l[t]+P12[t]+Llshed['bus1',t]+Pwl[t]-Ws[t]-Wlspill[t]==0\
                  for t in range(3)),name='low_scenario_bus1_balance')
model.addConstrs((rl['P3',t]+rflowdl[t]*0.8-rflowcl[t]*1.2+P12l[t]-P12[t]+Llshed['bus2',t]==0\
                  for t in range(3)),name='low_scenario_bus2_balance')

model.addConstrs((-RD[p,t]<=rh[p,t] for p in Ps for t in range(3)),name='high_scenario_down_constraint')
model.addConstrs((rh[p,t]<=RU[p,t] for p in Ps for t in range(3)),name='high_scenario_up_constraint')
model.addConstrs((-RD[p,t]<=rl[p,t] for p in Ps for t in range(3)),name='low_scenario_down_constraint')
model.addConstrs((rl[p,t]<=RU[p,t] for p in Ps for t in range(3)),name='low_scenario_up_constraint')


# 日内的出力约束
model.addConstrs((P[p,t]+rh[p,t]<=Pmax[p] for p in Ps for t in range(3)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t]+rh[p,t]>=Pmin[p] for p in Ps for t in range(3)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t]+rl[p,t]<=Pmax[p] for p in Ps for t in range(3)),name='low_scenario_P_constraint')
model.addConstrs((P[p,t]+rl[p,t]>=Pmin[p] for p in Ps for t in range(3)),name='low_scenario_P_constraint')

# 泄流有界性
model.addConstrs((flowc[t]+rflowch[t]<=20 for t in range(3)),name='')
model.addConstrs((flowc[t]+rflowch[t]>=-20 for t in range(3)),name='')
model.addConstrs((flowc[t]+rflowcl[t]<=20 for t in range(3)),name='')
model.addConstrs((flowc[t]+rflowcl[t]>=-20 for t in range(3)),name='')

model.addConstrs((flowd[t]+rflowdh[t]<=20 for t in range(3)),name='')
model.addConstrs((flowd[t]+rflowdh[t]>=-20 for t in range(3)),name='')
model.addConstrs((flowd[t]+rflowdl[t]<=20 for t in range(3)),name='')
model.addConstrs((flowd[t]+rflowdl[t]>=-20 for t in range(3)),name='')

# 初末库容相等
model.addConstr(gp.quicksum(flowc[t]-flowd[t] for t in range(3))==0,name='')
model.addConstr(gp.quicksum(rflowch[t]-rflowdh[t] for t in range(3))==0,name='')
model.addConstr(gp.quicksum(rflowcl[t]-rflowdl[t] for t in range(3))==0,name='')


# 库容的约束还没加上


# 求解
model.write('base case.lp')
model.optimize()

for v in model.getVars():
    print(v.varName, v.x)

# =============================================================================
# base case:6305
# =============================================================================

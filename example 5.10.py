
import gurobipy as gp
from gurobipy import GRB

model = gp.Model('A_two-stage_stochastic_programming')

# 基础参数

Ps, Pmin, Pmax, SU, SD, C, CRU, CRD = gp.multidict({
'P1': [10,50,200,200,10,16,15],
'P2': [10,110,200,200,30,13,12],
'P3': [5,100,200,200,35,10,9]})

buses = ['bus1','bus2','bus3']

load = {
('bus1',0): 40,
('bus1',1): 25,
('bus1',2): 45,
('bus2',0): 100,
('bus2',1): 80,
('bus2',2): 95}

Pwh = {0:50,1:65,2:35}
Pwl = {0:10,1:30,2:15}

# 创建变量
# 机组启停状态是日前、日内统一的
v = model.addVars(Ps, range(3), vtype=GRB.BINARY, name='v')
y = model.addVars(Ps, range(3), vtype=GRB.BINARY, name='y')
z = model.addVars(Ps, range(3), vtype=GRB.BINARY, name='z')

P = model.addVars(Ps,range(3),name='P')
RU = model.addVars(Ps,range(3),name='RU')
RD = model.addVars(Ps,range(3),name='RD')

P12 = model.addVars(range(3),lb=-100,ub=100,name='P12')
P12h = model.addVars(range(3),lb=-100,ub=100,name='P12h')
P12l = model.addVars(range(3),lb=-100,ub=100,name='P12l')

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
model.addConstrs((P12[t]+P['P3',t]==load['bus2',t] for t in range(3)),name='bus2_day_ahead_balance')

model.addConstrs((rh['P1',t]+rh['P2',t]-P12h[t]+P12[t]+Lhshed['bus1',t]+Pwh[t]-Ws[t]-Whspill[t]==0\
                  for t in range(3)),name='high_scenario_bus1_balance')
model.addConstrs((rh['P3',t]+P12h[t]-P12[t]+Lhshed['bus2',t]==0\
                  for t in range(3)),name='high_scenario_bus2_balance')

model.addConstrs((rl['P1',t]+rl['P2',t]-P12l[t]+P12[t]+Llshed['bus1',t]+Pwl[t]-Ws[t]-Wlspill[t]==0\
                  for t in range(3)),name='low_scenario_bus1_balance')
model.addConstrs((rl['P3',t]+P12l[t]-P12[t]+Llshed['bus2',t]==0\
                  for t in range(3)),name='low_scenario_bus2_balance')

# 日前就确定是否开机，可省略，在日内限制的情况下目标函数会引导
#model.addConstrs((P[p,t]+RU[p,t]<=v[p,t]*Pmax[p] for p in Ps for t in range(3)),name='day_ahead_up_constraint')
#model.addConstrs((P[p,t]-RD[p,t]>=v[p,t]*Pmin[p] for p in Ps for t in range(3)),name='day_ahead_down_constraint')

model.addConstrs((-RD[p,t]<=rh[p,t] for p in Ps for t in range(3)),name='high_scenario_down_constraint')
model.addConstrs((rh[p,t]<=RU[p,t] for p in Ps for t in range(3)),name='high_scenario_up_constraint')
model.addConstrs((-RD[p,t]<=rl[p,t] for p in Ps for t in range(3)),name='low_scenario_down_constraint')
model.addConstrs((rl[p,t]<=RU[p,t] for p in Ps for t in range(3)),name='low_scenario_up_constraint')


# 日内的出力约束
model.addConstrs((P[p,t]+rh[p,t]<=v[p,t]*Pmax[p] for p in Ps for t in range(3)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t]+rh[p,t]>=v[p,t]*Pmin[p] for p in Ps for t in range(3)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t]+rl[p,t]<=v[p,t]*Pmax[p] for p in Ps for t in range(3)),name='low_scenario_P_constraint')
model.addConstrs((P[p,t]+rl[p,t]>=v[p,t]*Pmin[p] for p in Ps for t in range(3)),name='low_scenario_P_constraint')

# 爬坡约束
model.addConstrs((P[p,t+1]+rh[p,t+1]-P[p,t]-rh[p,t]<=SU[p] for p in Ps for t in range(2)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t+1]+rh[p,t+1]-P[p,t]-rh[p,t]>=-SD[p] for p in Ps for t in range(2)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t+1]+rl[p,t+1]-P[p,t]-rl[p,t]<=SU[p] for p in Ps for t in range(2)),name='high_scenario_P_constraint')
model.addConstrs((P[p,t+1]+rl[p,t+1]-P[p,t]-rl[p,t]>=-SD[p] for p in Ps for t in range(2)),name='high_scenario_P_constraint')



model.addConstrs((v[p,t]+v[p,t+1]>=2*y[p,t] for p in ['P1','P2'] for t in range(2)),name='')
model.addConstrs((v[p,t]+v[p,t+1]<=2-2*z[p,t] for p in ['P1','P2'] for t in range(2)),name='')

model.addConstr(v['P3',0]+v['P3',1]+v['P3',2]>=3*y['P3',0],name='')
model.addConstr(v['P3',1]+v['P3',2]>=2*y['P3',1],name='')


model.addConstrs((y[p,t]+z[p,t]<=1 for p in Ps for t in range(3)),name='')
model.addConstrs((y[p,t+1]-z[p,t+1]==v[p,t+1]-v[p,t] for p in Ps for t in range(2)),name='')

model.addConstrs((y[p,0]-z[p,0]==v[p,0]-1 for p in ['P1','P2']),name='')
model.addConstr(y['P3',0]-z['P3',0]==v['P3',0],name='')
# model.addConstr(y['P3',0]==1,name='')

# =============================================================================
# model.addConstrs((v[p,t]+v[p,t+1]>=2*y[p,t] for p in Ps for t in range(2)),name='')
# model.addConstrs((v[p,t]+v[p,t+1]<=2-2*z[p,t] for p in Ps for t in range(2)),name='')
# 
# model.addConstrs((y[p,t]+z[p,t]<=1 for p in Ps for t in range(3)),name='')
# model.addConstrs((y[p,t+1]-z[p,t+1]==v[p,t+1]-v[p,t] for p in Ps for t in range(2)),name='')
# model.addConstrs((y[p,0]-z[p,0]==v[p,0]-1 for p in Ps),name='')
# =============================================================================

# 求解s
model.write('base case.lp')
model.optimize()

for v in model.getVars():
    print(v.varName, v.x)

# 最小出力约束+2h启停约束6435


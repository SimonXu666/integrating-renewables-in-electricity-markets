
import gurobipy as gp
from gurobipy import GRB

model = gp.Model('A_two-stage_stochastic_programming')

# 基础参数

Ps, Pmin, Pmax, C, CRU, CRD = gp.multidict({
'P1': [0,50,10,16,15],
'P2': [0,110,30,13,12],
'P3': [0,100,35,10,9]})


Ds = ['D1','D2','D3','D4']
buses = ['bus1','bus2']

_, Dmax, CD = gp.multidict({
('D1',0): [40,40],('D1',1): [25,25],('D1',2): [45,35],
('D2',0): [50,15],('D2',1): [45,15],('D2',2): [50,15],
('D3',0): [60,45],('D3',1): [35,30],('D3',2): [40,40],
('D4',0): [40,55],('D4',1): [45,40],('D4',2): [55,50]})

minD = {'D1':100,'D2':0,'D3':120,'D4':130}
deltaD = {'D1':25,'D2':15,'D3':30,'D4':20}

# 负荷，以及其在高低情景下的上调、下调量
D = model.addVars(Ds,range(3),name='D')
DhU = model.addVars(Ds,range(3),name='DhU')# ,ub=0
DhD = model.addVars(Ds,range(3),name='DhD')
DlU = model.addVars(Ds,range(3),name='DlU')
DlD = model.addVars(Ds,range(3),name='DlD')


Pwh = {0:50,1:65,2:35}
Pwl = {0:10,1:30,2:15}

# 创建变量

P = model.addVars(Ps,range(3),name='P')
RU = model.addVars(Ps,range(3),name='RU')
RD = model.addVars(Ps,range(3),name='RD')

P12 = model.addVars(range(3),lb=-100,ub=100,name='P12')
P12h = model.addVars(range(3),lb=-100,ub=100,name='P12h')
P12l = model.addVars(range(3),lb=-100,ub=100,name='P12l')

rh = model.addVars(Ps,range(3),lb=-200,name='rh')
rl = model.addVars(Ps,range(3),lb=-200,name='rl')

Lhshed = model.addVars(buses,range(3),name='Lhshed')
Llshed = model.addVars(buses,range(3),name='Llshed')

Ws = model.addVars(range(3),name='Ws')
Whspill = model.addVars(range(3),name='Whspill')
Wlspill = model.addVars(range(3),name='Wlspill')


# 设置目标
obj_da = gp.quicksum(C[p]*P[p,t]+CRU[p]*RU[p,t]+CRD[p]*RD[p,t] for p in Ps for t in range(3))
obj_h = 0.6*(gp.quicksum(C[p]*rh[p,t] for p in Ps for t in range(3))+\
             200*(gp.quicksum(Lhshed[bus,t] for bus in buses for t in range(3))))
obj_l = 0.4*(gp.quicksum(C[p]*rl[p,t] for p in Ps for t in range(3))+\
             200*(gp.quicksum(Llshed[bus,t] for bus in buses for t in range(3))))
obj_d = gp.quicksum(CD[d,t]*D[d,t] for d in Ds for t in range(3))
# 日前不买，日内买就便宜了
deltaCD = 0.05
# obj_dU = 0.95*gp.quicksum(CD[d,t]*(0.6*DhU[d,t]+0.4*DlU[d,t]) for d in Ds for t in range(3))
# obj_dD = 1.05*gp.quicksum(CD[d,t]*(0.6*DhD[d,t]+0.4*DlD[d,t]) for d in Ds for t in range(3))
obj_dh = 0.6*gp.quicksum(CD[d,t]*((1-deltaCD)*DhU[d,t]-(1+deltaCD)*DhD[d,t]) for d in Ds for t in range(3))
obj_dl = 0.4*gp.quicksum(CD[d,t]*((1-deltaCD)*DlU[d,t]-(1+deltaCD)*DlD[d,t]) for d in Ds for t in range(3))


model.setObjective(obj_da+obj_h+obj_l-obj_d-obj_dh-obj_dl,GRB.MINIMIZE)


# 添加约束

model.addConstrs((P['P1',t]+P['P2',t]-P12[t]+Ws[t]==D['D1',t]+D['D2',t] for t in range(3)),name='bus1_day_ahead_balance')
model.addConstrs((P12[t]+P['P3',t]==D['D3',t]+D['D4',t] for t in range(3)),name='bus2_day_ahead_balance')

model.addConstrs((rh['P1',t]+rh['P2',t]-P12h[t]+P12[t]+Lhshed['bus1',t]\
                  +Pwh[t]-Ws[t]-Whspill[t]==DhU['D1',t]+DhU['D2',t]-DhD['D1',t]-DhD['D2',t]\
                  for t in range(3)),name='high_scenario_bus1_balance')
model.addConstrs((rh['P3',t]+P12h[t]-P12[t]+Lhshed['bus2',t]==DhU['D3',t]+DhU['D4',t]-DhD['D3',t]-DhD['D4',t]\
                  for t in range(3)),name='high_scenario_bus2_balance')

model.addConstrs((rl['P1',t]+rl['P2',t]-P12l[t]+P12[t]+Llshed['bus1',t]\
                   +Pwl[t]-Ws[t]-Wlspill[t]==DlU['D1',t]+DlU['D2',t]-DlD['D1',t]-DlD['D2',t]\
                  for t in range(3)),name='low_scenario_bus1_balance')
model.addConstrs((rl['P3',t]+P12l[t]-P12[t]+Llshed['bus2',t]==DlU['D3',t]+DlU['D4',t]-DlD['D3',t]-DlD['D4',t]\
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

# 负荷消纳约束
model.addConstrs((gp.quicksum(D[d,t]+DhU[d,t]-DhD[d,t] for t in range(3))>=minD[d] for d in Ds),name='')
model.addConstrs((gp.quicksum(D[d,t]+DlU[d,t]-DlD[d,t] for t in range(3))>=minD[d] for d in Ds),name='')

# 负荷约束
model.addConstrs((D[d,t]+DhU[d,t]-DhD[d,t]<=Dmax[d,t] for d in Ds for t in range(3)),name='')
model.addConstrs((D[d,t]+DlU[d,t]-DlD[d,t]<=Dmax[d,t] for d in Ds for t in range(3)),name='')

# 负荷爬坡约束
model.addConstrs((D[d,t+1]+DhU[d,t+1]-DhD[d,t+1]-(D[d,t]+DhU[d,t]-DhD[d,t])<=deltaD[d] for d in Ds for t in range(2)),name='')
model.addConstrs((D[d,t+1]+DhU[d,t+1]-DhD[d,t+1]-(D[d,t]+DhU[d,t]-DhD[d,t])>=-deltaD[d] for d in Ds for t in range(2)),name='')
model.addConstrs((D[d,t+1]+DlU[d,t+1]-DlD[d,t+1]-(D[d,t]+DlU[d,t]-DlD[d,t])<=deltaD[d] for d in Ds for t in range(2)),name='')
model.addConstrs((D[d,t+1]+DlU[d,t+1]-DlD[d,t+1]-(D[d,t]+DlU[d,t]-DlD[d,t])>=-deltaD[d] for d in Ds for t in range(2)),name='')


# 求解
model.write('base case.lp')
model.optimize()

for v in model.getVars():
    print(v.varName, v.x)

# =============================================================================
# base case:-9595
# =============================================================================


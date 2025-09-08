# book 《Power System Optimization Modeling in GAMS》9.1
# transmission expansion planning with New Lines Option
import gurobipy as gp
from gurobipy import GRB
import math

# 创建模型
model = gp.Model("6NodeTransmission")

# 定义集合
bus = [1, 2, 3, 4, 5, 6]
slack_bus = 1
Gen = ['g1', 'g2', 'g3']
k = ['k1', 'k2', 'k3', 'k4']

# 定义参数
Sbase = 100  # MVA
M = 1000     # 大M值

# 发电机数据
GenData = {
    'g1': {'b': 20, 'pmin': 0, 'pmax': 400},
    'g2': {'b': 30, 'pmin': 0, 'pmax': 400},
    'g3': {'b': 10, 'pmin': 0, 'pmax': 600}}

# 发电机连接关系
GBconect = {
    (1, 'g1'): True,
    (3, 'g2'): True,
    (6, 'g3'): True}

# 节点负荷数据 (MW)
BusData = {1: 80, 2: 240, 3: 40, 4: 160, 5: 240, 6: 0}

# 线路数据 (格式: (bus_i, bus_j): [X, LIMIT, Cost, stat])
branch = {
    (1, 2): [0.4, 100, 40, 1],
    (1, 4): [0.6, 80, 60, 1],
    (1, 5): [0.2, 100, 20, 1],
    (2, 3): [0.2, 100, 20, 1],
    (2, 4): [0.4, 100, 40, 1],
    (2, 6): [0.3, 100, 30, 0],
    (3, 5): [0.2, 100, 20, 1],
    (4, 6): [0.3, 100, 30, 0]}

# 创建对称连接 (无向图)
conex = {}
for (i, j) in list(branch.keys()):
    conex[(i, j)] = True
    conex[(j, i)] = True
    x = branch[i, j][0]
    branch[i, j].append(1/x)  # bij at index 4
    branch[j, i] = branch[i, j][:]  # 创建对称副本

branch, X, LIMIT, Cost, stat, b = gp.multidict(branch)

# 定义变量
Pg = model.addVars(Gen, name="Pg")  # 发电机出力
LS = model.addVars(bus, name="LS")  # 切负荷
delta = model.addVars(bus, name="delta", lb=-math.pi/3, ub=math.pi/3)  # 相角
alpha = model.addVars(branch, k, vtype=GRB.BINARY, name="alpha")
Pij = model.addVars(branch, k, lb=-M, ub=M, name="Pij")

# 固定松弛节点相角
model.addConstr(delta[slack_bus] == 0, name="slack_bus_fix")

# 约束1: 现有线路的第一个回路固定为1
model.addConstrs((alpha[i, j, 'k1'] == 1 for (i, j) in branch if stat[i, j]), name="fix_alpha")

# 约束2: 潮流方程线性化
model.addConstrs((Pij[i, j, k_val] - b[i, j] * (delta[i] - delta[j]) <= M * (1 - alpha[i, j, k_val])
                  for (i, j) in branch for k_val in k), name="A")
model.addConstrs((Pij[i, j, k_val] - b[i, j] * (delta[i] - delta[j]) >= -M * (1 - alpha[i, j, k_val])
                  for (i, j) in branch for k_val in k), name="B")
model.addConstrs((Pij[i, j, k_val] <= alpha[i, j, k_val] * LIMIT[i, j] / Sbase
                  for (i, j) in branch for k_val in k), name="C")
model.addConstrs((Pij[i, j, k_val] >= -alpha[i, j, k_val] * LIMIT[i, j] / Sbase
                  for (i, j) in branch for k_val in k), name="D")
model.addConstrs((alpha[i, j, k_val] == alpha[j, i, k_val]
                  for (i, j) in branch for k_val in k), name="E")
# 节点功率平衡约束
model.addConstrs((LS[i] + gp.quicksum(Pg[g] for g in Gen if GBconect.get((i, g))) - BusData[i] / Sbase ==\
                  gp.quicksum(Pij[i, j, k_val] for j in bus if (i, j) in conex for k_val in k) for i in bus), name="power_balance")
# 发电机出力上下限
for g in Gen:
    Pg[g].lb = GenData[g]['pmin'] / Sbase
    Pg[g].ub = GenData[g]['pmax'] / Sbase

# 目标函数
gen_cost = gp.quicksum(Pg[g] * GenData[g]['b'] * Sbase for g in Gen)
load_shed_cost = 100000 * gp.quicksum(LS[i] for i in bus)
line_cost = gp.quicksum(0.5 * Cost[i, j] * alpha[i, j, k_val]\
            for (i, j) in branch for k_val in k if (k_val != 'k1') or (stat[i, j] == 0))

# 总目标函数 (年化成本)
obj = 2 * 8760 * (gen_cost + load_shed_cost) + 1e6 * line_cost
model.setObjective(obj, GRB.MINIMIZE)

# 求解模型
model.write('TEP.lp')
model.optimize()

# 输出结果
if model.status == GRB.OPTIMAL:
    print("\n最优目标值:", model.objVal)
    print("\n发电机出力:")
    for g in Gen: print(f"{g}: {Pg[g].X * Sbase:.2f} MW")
    print("\n切负荷:")
    for i in bus:
        if LS[i].X > 0:
            print(f"Bus {i}: {LS[i].X * Sbase:.2f} MW")
    print("\n线路建设决策:")
    for (i, j, k_val) in alpha:
        if alpha[i, j, k_val].X > 0.5:
            print(f"线路 ({i},{j}) 回路 {k_val}: 建设")
    print("\n相角 (度):")
    for i in bus: print(f"Bus {i}: {delta[i].X * 180/math.pi:.2f}°")
else:
    print("未找到最优解")

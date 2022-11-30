from math import exp, sqrt
import json

n = 7;

# Bridge class A
length_irl = 28 * 100;

length = 80;
width = 20;
height = 25;

gain = 3;

STEP_PERCENT_A = .30;
STEP_PERCENT_B = .30;
BASE_RESCALE = 2;
BEAM_RAD = 0.7;
BAL_RAD = 1.5;

JOINT_RAD = 0.9;
MARGIN = 3;

ROAD_H = 1;

mod2irl = length_irl / length;

cosh = lambda x: (exp (x) + exp(-x)) / 2
norm_gain_cosh = lambda x: (-cosh(x * gain) + cosh(gain))/(cosh(gain) - 1)

p2pm = lambda x: 2 * ((x) - 0.5)
pm2p = lambda x: (x + 1) / 2

dst = lambda a, b: sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
inc = length / n

# Calculate forces

l_m_irl = length_irl/100
w_m_irl = (mod2irl * width)/100

print(l_m_irl, "m", w_m_irl,"m")

len_car = 4.5
mass_car = 3.5e3
lain = 2
g = 9.81

total_force_irl = (2 * mass_car * l_m_irl / len_car) * g
print(f"{total_force_irl=}","N")

pascal = total_force_irl / (l_m_irl * w_m_irl)
print(f"{pascal=}","Pa")

total_force_model = pascal * (width / 100) * (length / 100)
print(f"{total_force_model=}","N",f"{round(total_force_model/g,2)=}","kg")

distribution = total_force_model / (n - 1)
print(f"{round(distribution,2)=}", "N")

# Build Bridge
nodes = [ ]

for i in range(n):
    iz = i + .5
    x = p2pm(iz / n)
    y = norm_gain_cosh(x)

    nodes.append((round(i * inc,  2), 0))
    nodes.append((round(iz * inc, 2), round(y * height, 2)))

nodes.append((length,0))

members = []
member_dst = {}

for i in range(0,2*n-1,2):
    a_dst = f'{i},{i+2}'
    member_dst[a_dst] = dst(nodes[i],nodes[i+2])
    members.append(a_dst)
    if i+3 < n * 2:
        b_dst = f'{i+1},{i+3}'
        member_dst[b_dst] = dst(nodes[i+1],nodes[i+3])
        members.append(b_dst)

for i in range(n*2):
    a_dst = f'{i},{i+1}'
    members.append(a_dst)
    member_dst[a_dst] = dst(nodes[i],nodes[i+1])

with open('out.json','w') as f:
    f.write(json.dumps({
        "nodes": list(map(lambda x: f'{x[0]},{x[1]}',nodes)),
        "members": members,
        "supports": { "0": "P", f"{n*2}": "Rh" },
        "forces": list(map(lambda x: f'{x*2},0,{-distribution}',range(1,n))),
        "workspace": {
          "workspace-width": 187,
          "workspace-height": 102,
          "workspace-width-pixels": 1412,
          "Yaxis-dist-from-left": 88.9971671388102,
          "Xaxis-dist-from-bottom": 30.433852691218124,
          "grid-x": 1,
          "grid-y": 1,
          "force-scale": 100
        }
    }))


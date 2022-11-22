from math import exp
import json

n = 7;

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

cosh = lambda x: (exp (x) + exp(-x)) / 2
norm_gain_cosh = lambda x: (-cosh(x * gain) + cosh(gain))/(cosh(gain) - 1)

p2pm = lambda x: 2 * ((x) - 0.5)
pm2p = lambda x: (x + 1) / 2

nodes = [ ]

inc = length / n

for i in range(n):
    iz = i + .5
    x = p2pm(iz / n)
    y = norm_gain_cosh(x)

    nodes.append(f'{round(i * inc,  2)},0')
    nodes.append(f'{round(iz * inc, 2)},{round(y * height, 2)}')

nodes.append(f'{length},0')

members = []
for i in range(0,2*n-1,2):
    members.append(f'{i},{i+2}')
    if i+3 < n * 2:
        members.append(f'{i+1},{i+3}')

for i in range(n*2):
    members.append(f'{i},{i+1}')

with open('out.json','w') as f:
    f.write(json.dumps({
        "nodes": nodes,
        "members": members,
        "supports": { "0": "P", f"{n*2}": "Rh" },
        "forces": [],
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



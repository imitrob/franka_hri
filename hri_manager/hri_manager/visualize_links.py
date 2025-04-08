import yaml
import os
import plotly.graph_objects as go
import dash
import dash_core_components as dcc
import dash_html_components as html
from collections import defaultdict

import hri_manager, trajectory_data

user = "bruno"

# Load user configuration from YAML
with open(f'{hri_manager.package_path}/links/{user}_links.yaml') as f:
    user_data = yaml.safe_load(f)
actions = user_data['actions']
objects = user_data['objects']

# Load skill files
skill_files = os.listdir(f'{trajectory_data.package_path}/trajectories')
skill_files = [f for f in skill_files if f.endswith('.npz')]

zero_actions = []
single_actions = []
double_actions = []

for action in actions:
    # Check zero-object actions
    if f"{action}.npz" in skill_files:
        zero_actions.append(action)
        continue
    
    # Check single-object actions
    single_skills = [f for f in skill_files if f.startswith(f"{action}_")]
    if single_skills:
        single_actions.append(action)
        continue
    
    # Check double-object actions
    part1 = any(f.startswith(f"{action}1_") for f in skill_files)
    part2 = any(f.startswith(f"{action}2_") for f in skill_files)
    if part1 and part2:
        double_actions.append(action)

nodes = []
node_info = {}  # {node_id: {'label': str, 'group': str}}
edges = []

# Add action nodes
groups = ['zero', 'single', 'double']
for idx, group in enumerate(groups):
    for action in eval(f"{group}_actions"):
        node_id = len(nodes)
        nodes.append(node_id)
        node_info[node_id] = {
            'label': f"{action} ({group[0].upper()})",
            'group': f"action_{group}"
        }

# Add object nodes
for obj in objects:
    node_id = len(nodes)
    nodes.append(node_id)
    node_info[node_id] = {'label': obj, 'group': 'object'}

# Add skill nodes
skill_nodes = defaultdict(list)
for skill in skill_files:
    node_id = len(nodes)
    nodes.append(node_id)
    node_info[node_id] = {'label': skill, 'group': 'skill'}
    skill_nodes[skill] = node_id

# Create edges
def find_node(label_part, group):
    for node_id, info in node_info.items():
        if info['group'] == group and label_part in info['label']:
            return node_id
    return None

# Zero-object edges
for action in zero_actions:
    action_node = find_node(f"{action} (Z)", "action_zero")
    skill_node = skill_nodes.get(f"{action}.npz")
    if action_node is not None and skill_node is not None:
        edges.append((action_node, skill_node))

# Single-object edges
for action in single_actions:
    action_node = find_node(f"{action} (S)", "action_single")
    for obj in objects:
        skill_name = f"{action}_{obj}.npz"
        if skill_name in skill_nodes:
            obj_node = find_node(obj, "object")
            skill_node = skill_nodes[skill_name]
            edges.append((action_node, obj_node))
            edges.append((obj_node, skill_node))

# Double-object edges
for action in double_actions:
    action_node = find_node(f"{action} (D)", "action_double")
    for obj1 in objects:
        for obj2 in objects:
            skill1 = f"{action}1_{obj1}.npz"
            skill2 = f"{action}2_{obj2}.npz"
            if skill1 in skill_nodes and skill2 in skill_nodes:
                obj_node1 = find_node(obj1, "object")
                obj_node2 = find_node(obj2, "object")
                edges.append((action_node, obj_node1))
                edges.append((action_node, obj_node2))
                edges.append((obj_node1, skill_nodes[skill1]))
                edges.append((obj_node2, skill_nodes[skill2]))

# Create network graph
node_x = []
node_y = []
colors = []
text = []
group_positions = {
    'action_zero': (0, 0),
    'action_single': (0, 1),
    'action_double': (0, 2),
    'object': (1, 0),
    'skill': (2, 0)
}

for node_id in nodes:
    group = node_info[node_id]['group']
    x, y = group_positions.get(group, (3, 0))
    node_x.append(x)
    node_y.append(y)
    text.append(node_info[node_id]['label'])
    colors.append(1 if 'action' in group else (2 if group == 'object' else 3))

edge_x = []
edge_y = []
for edge in edges:
    x0, y0 = node_x[edge[0]], node_y[edge[0]]
    x1, y1 = node_x[edge[1]], node_y[edge[1]]
    edge_x.extend([x0, x1, None])
    edge_y.extend([y0, y1, None])

fig = go.Figure()
fig.add_trace(go.Scatter(
    x=edge_x, y=edge_y,
    line=dict(width=0.5, color='#888'),
    hoverinfo='none',
    mode='lines'))

fig.add_trace(go.Scatter(
    x=node_x, y=node_y,
    mode='markers+text',
    text=text,
    textposition="bottom center",
    marker=dict(
        color=colors,
        size=20,
        line=dict(width=2, color='DarkSlateGrey'))
))

fig.update_layout(
    showlegend=False,
    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
    plot_bgcolor='white'
)

# Create Dash app
app = dash.Dash(__name__)
app.layout = html.Div([
    dcc.Graph(figure=fig)
])

if __name__ == '__main__':
    app.run(debug=True, port=8051)
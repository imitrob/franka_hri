import yaml, os
import hri_manager, trajectory_data, object_localization
import yaml
import os
import io
import base64
import plotly.graph_objects as go
import dash
from dash import dcc, html, Input, Output, State
from collections import defaultdict
from PIL import Image

from trajectory_data.skill_visualizer import minimal_trajectory_png, trajectories_fig, load_traj, load_grip

CFG_DIR = f"{object_localization.package_path}/cfg"


def _action_files(action, files):
    """The action's skill recordings, grouped: plain/single-object ones and
    the part1/part2 halves of a double-object skill."""
    ok = lambda f: "trial" not in f and "branch" not in f
    singles = sorted(f for f in files if ok(f) and (f.startswith(f"{action}_") or f == f"{action}.npz"))
    part1 = sorted(f for f in files if ok(f) and f.startswith(f"{action}1_"))
    part2 = sorted(f for f in files if ok(f) and f.startswith(f"{action}2_"))
    return singles, part1, part2


def _file_to_data_uri(path, max_px=200):
    """Small base64 data URI of an image file (downscaled to keep page light)."""
    img = Image.open(path)
    img.thumbnail((max_px, max_px))
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()

app = dash.Dash(__name__)

action_nodes = [] # Store node positions globally
object_nodes = []

# Get all available users from YAML files in the links directory
links_dir = f'{hri_manager.package_path}/links'
user_files = [f for f in os.listdir(links_dir) if f.endswith('_links.yaml')]

# Create a mapping of display names to filenames
user_mapping = {}
for filename in user_files:
    base_name = filename.replace('_links.yaml', '')
    
    display_name = base_name.replace('_', ' ').title()
    user_mapping[display_name] = filename

app.layout = html.Div([
    dcc.Store(id='user-store'),
    dcc.Store(id='current-action'),
    html.Div(id='dummy-output', style={'display': 'none'}),
    html.Div([
        dcc.Dropdown(
            id='user-dropdown',
            options=[{'label': display_name, 'value': filename} 
                    for display_name, filename in user_mapping.items()],
            value=list(user_mapping.values())[0] if user_mapping else None,
            style={'width': '200px', 'margin': '10px'}
        ),
    ]),
    html.Div([
        # Static graph: zoom/pan disabled (embedded pngs don't rescale well),
        # clicks still work
        dcc.Graph(id='main-graph', style={'width': '60%', 'height': '90vh', 'display': 'inline-block'},
                  config={'displayModeBar': False, 'scrollZoom': False, 'doubleClick': False}),
        dcc.Graph(id='trajectory-display', style={'width': '40%', 'height': '90vh', 'display': 'inline-block'}),
        html.Div(  # bottom bar that appears with the command
            id='command-bar',
            children="",
            style={
                'display': 'none',
                'position': 'fixed',
                'left': 0, 'right': 0, 'bottom': 0,
                'zIndex': 9999,
                'padding': '10px 14px',
                'backgroundColor': '#111',
                'color': '#eee',
                'fontFamily': 'monospace',
                'fontSize': '14px',
                'borderTop': '1px solid #444',
                'whiteSpace': 'pre'
            }
        ),
    ])
])

# Store data globally
actions = []
objects = []
skill_db = defaultdict(dict)
action_thumbs = {}   # action -> data URI of the minimalist 3D trajectory plot
object_images = {}   # object -> data URI of the cfg "Template Cropped" image

@app.callback(
    Output('dummy-output', 'children'),
    Input('user-dropdown', 'value'),
    prevent_initial_call=True
)
def load_user_data(selected_filename):
    global actions, objects, skill_db, action_thumbs, object_images

    # Clear previous data
    actions = []
    objects = []
    skill_db = defaultdict(dict)
    action_thumbs = {}
    object_images = {}
    
    # Load user data (`actions` is derived from the arity lists)
    from hri_manager.user_links import load_user_links
    user_data = load_user_links(selected_filename.removesuffix('_links.yaml'))
    actions = user_data['actions']
    objects = user_data['objects']

    # Load skills
    skill_files = os.listdir(f'{trajectory_data.package_path}/trajectories')
    skill_files = [f for f in skill_files if f.endswith('.npz')]

    # Build skill database
    for f in skill_files:
        if '_' in f:
            parts = f.split('_')
            if len(parts) == 2:
                skill_part, obj = parts
                obj = obj.replace('.npz', '')
                if skill_part[-1].isdigit():
                    action = skill_part[:-1]
                    num = skill_part[-1]
                    skill_db[(action, num)][obj] = True
                else:
                    action = skill_part
                    skill_db[(action, 'single')][obj] = True

    # Minimalist 3D trajectory preview for each action node. A double-object
    # action (e.g. put) shows its part1 + part2 trajectories overlaid in two
    # colors; otherwise the first recording of the action is shown
    # (thumbnails are disk-cached, so only the first load renders anything).
    for action in actions:
        singles, part1, part2 = _action_files(action, skill_files)
        candidates = part1[:1] + part2[:1] if (part1 or part2) else singles[:1]
        if not candidates:
            continue
        try:
            png = minimal_trajectory_png(candidates)
            if png is not None:
                action_thumbs[action] = _file_to_data_uri(png)
        except Exception as e:
            print(f"Trajectory thumbnail failed for {candidates}: {e}")

    # "Template Cropped" image (cfg/<object>/template.png) for each object node
    for obj in objects:
        for dirname in (obj, f"{obj}_template"):
            path = os.path.join(CFG_DIR, dirname, "template.png")
            if os.path.isfile(path):
                try:
                    object_images[obj] = _file_to_data_uri(path)
                except Exception as e:
                    print(f"Template image failed for {obj}: {e}")
                break

    return ""  # Returns empty string to dummy output
@app.callback(
    Output('main-graph', 'figure'),
    [Input('dummy-output', 'children'),
     Input('main-graph', 'relayoutData')],
    prevent_initial_call=True
)
def create_main_graph(trigger, _):
    # Check which input triggered the callback
    ctx = dash.callback_context
    if not ctx.triggered:
        return dash.no_update
        
    # Only proceed if we have data
    if not actions or not objects:
        return go.Figure()
    fig = go.Figure()
    global action_nodes, object_nodes
    
    # Clear previous nodes
    action_nodes = []
    object_nodes = []
    
    # Create action nodes (left column)
    for idx, action in enumerate(actions):
        action_nodes.append({
            'label': action,
            'x': 0,
            'y': 1 - idx/len(actions),
            'type': 'action'
        })
    
    # Create object nodes (right column)
    for idx, obj in enumerate(objects):
        object_nodes.append({
            'label': obj,
            'x': 1,
            'y': 1 - idx/len(objects),
            'type': 'object'
        })

    # Add action nodes to figure (with showlegend=True)
    fig.add_trace(go.Scatter(
        x=[n['x'] for n in action_nodes],
        y=[n['y'] for n in action_nodes],
        mode='markers+text',
        text=[n['label'] for n in action_nodes],
        marker=dict(size=26, color='#FF9AA2'),
        textfont=dict(size=17, color='black'),
        textposition="middle center",
        hoverinfo='text',
        name='Actions (Left)',
        showlegend=True
    ))

    # Add object nodes to figure (with showlegend=True)
    fig.add_trace(go.Scatter(
        x=[n['x'] for n in object_nodes],
        y=[n['y'] for n in object_nodes],
        mode='markers+text',
        text=[n['label'] for n in object_nodes],
        marker=dict(size=26, color='#B5EAD7'),
        textfont=dict(size=17, color='black'),
        textposition="middle center",
        hoverinfo='text',
        name='Objects (Right)',
        showlegend=True
    ))

    # Add edges for all valid actions
    soa_leg = True
    fo_leg = True
    so_leg = True
    for action in actions:
        # Single object actions (gray solid lines)
        if (action, 'single') in skill_db:
            for obj in skill_db[(action, 'single')]:
                if obj not in objects:
                    continue
        
                start = next(n for n in action_nodes if n['label'] == action)
                end = next(n for n in object_nodes if n['label'] == obj)
                fig.add_trace(go.Scatter(
                    x=[start['x'], end['x']],
                    y=[start['y'], end['y']],
                    mode='lines',
                    line=dict(color='#888', width=2),
                    hoverinfo='none',
                    name='Single-object action',
                    showlegend=True if soa_leg else False  # Only show once
                ))
                soa_leg = False
        
        # Double object actions: the edge carries a "1" / "2" badge at its
        # midpoint so first- and second-object edges are told apart at a
        # glance (matching the legend names)
        if (action, '1') in skill_db or (action, '2') in skill_db:
            # First objects (yellow dashed lines)
            if (action, '1') in skill_db:
                for obj in skill_db[(action, '1')]:
                    if obj not in objects:
                        continue
                    start = next(n for n in action_nodes if n['label'] == action)
                    end = next(n for n in object_nodes if n['label'] == obj)
                    fig.add_trace(go.Scatter(
                        x=[start['x'], (start['x'] + end['x']) / 2, end['x']],
                        y=[start['y'], (start['y'] + end['y']) / 2, end['y']],
                        mode='lines+text',
                        text=["", "1", ""],
                        textfont=dict(size=15, color='#B8860B'),
                        textposition="top center",
                        line=dict(color='#FFD700', width=2, dash='dot'),
                        hoverinfo='none',
                        name='1st object (yellow dotted)',
                        showlegend=True if fo_leg else False  # Only show once
                    ))
                    fo_leg = False

            # Second objects (blue dashed lines)
            if (action, '2') in skill_db:
                for obj in skill_db[(action, '2')]:
                    if obj not in objects:  # Add this check
                        continue
                    start = next(n for n in action_nodes if n['label'] == action)
                    end = next(n for n in object_nodes if n['label'] == obj)
                    fig.add_trace(go.Scatter(
                        x=[start['x'], (start['x'] + end['x']) / 2, end['x']],
                        y=[start['y'], (start['y'] + end['y']) / 2, end['y']],
                        mode='lines+text',
                        text=["", "2", ""],
                        textfont=dict(size=15, color='#4682B4'),
                        textposition="top center",
                        line=dict(color='#4682B4', width=1, dash='dashdot'),
                        hoverinfo='none',
                        name='2nd object (blue dash-dot)',
                        showlegend=True if so_leg else False  # Only show once
                    ))
                    so_leg = False

    # Embed the minimalist 3D trajectory next to each action node and the
    # cfg "Template Cropped" picture next to each object node
    for n in action_nodes:
        uri = action_thumbs.get(n['label'])
        if uri:
            fig.add_layout_image(
                source=uri, xref='x', yref='y',
                x=n['x'] - 0.05, y=n['y'], xanchor='right', yanchor='middle',
                sizex=0.30, sizey=min(0.17, 0.9 / max(len(action_nodes), 1)),
                layer='above')
    for n in object_nodes:
        uri = object_images.get(n['label'])
        if uri:
            fig.add_layout_image(
                source=uri, xref='x', yref='y',
                x=n['x'] + 0.05, y=n['y'], xanchor='left', yanchor='middle',
                sizex=0.30, sizey=min(0.17, 0.9 / max(len(object_nodes), 1)),
                layer='above')

    # Customize legend and layout
    fig.update_layout(
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1,
            title_text="Legend:"
        ),
        showlegend=True,
        xaxis=dict(showgrid=False, zeroline=False, showticklabels=False, range=[-0.45, 1.45], fixedrange=True),
        yaxis=dict(showgrid=False, zeroline=False, showticklabels=False, fixedrange=True),
        dragmode=False,
        plot_bgcolor='white',
        margin=dict(l=20, r=20, t=40, b=20),
        title=""
    )
    
    # Hide duplicate legend entries (we only want one per type)
    names = set()
    fig.for_each_trace(
        lambda trace:
            trace.update(showlegend=False)
            if (trace.name in names) else names.add(trace.name)
    )
    
    return fig

# Callback for the trajectory display: clicking an action node shows the
# whole End-Effector Trajectory of every recording of that action, overlaid
# in one 3D scene (a double-object action like put shows its part1 and part2
# trajectories together). The valid objects are already visible as edges in
# the left graph.
@app.callback(
    Output('trajectory-display', 'figure'),
    Output('current-action', 'data'),
    Input('main-graph', 'clickData'))
def _blank_fig(title=""):
    """Empty white panel: no axes, optionally just a title."""
    fig = go.Figure()
    fig.update_layout(title=title, xaxis=dict(visible=False), yaxis=dict(visible=False),
                      plot_bgcolor='white', paper_bgcolor='white')
    return fig


def update_trajectory_display(clickData):
    fig = _blank_fig()
    if not clickData:
        return fig, dash.no_update

    try:
        point_index = clickData['points'][0]['pointIndex']
        clicked_trace = clickData['points'][0]['curveNumber']

        # Only respond to clicks on action nodes (trace 0)
        if clicked_trace != 0:
            return fig, dash.no_update

        action = action_nodes[point_index]['label']

        skill_files = [f for f in os.listdir(f'{trajectory_data.package_path}/trajectories')
                       if f.endswith('.npz')]
        singles, part1, part2 = _action_files(action, skill_files)
        files = singles + part1 + part2
        if not files:
            return _blank_fig(f"No recordings for {action}"), dash.no_update

        named_trajs, named_grips = {}, {}
        for f in files:
            label = f.replace('.npz', '')
            try:
                named_trajs[label] = load_traj(f)
            except Exception as e:
                print(f"Failed to load trajectory {f}: {e}")
                continue
            try:
                named_grips[label] = load_grip(f)  # marks gripper open/close points
            except Exception as e:
                print(f"No gripper data for {f}: {e}")

        fig = trajectories_fig(named_trajs, named_grips)
        # More top margin so the title sits above the horizontal legend
        # (legend y=1.02) instead of being pushed out of view
        fig.update_layout(title=dict(text=f"End-Effector Trajectory: {action}", y=0.98, yanchor="top"),
                          margin=dict(t=90))
        return fig, {'action': action, 'files': list(named_trajs.keys())}

    except Exception as e:
        print(f"Error updating trajectory display: {e}")

    return fig, dash.no_update

@app.callback(
    Output('command-bar', 'children'),
    Output('command-bar', 'style'),
    Input('trajectory-display', 'clickData'),
    State('current-action', 'data')
)
def show_command_bar(clickData, data):
    """Clicking one of the displayed trajectories shows the launch command
    for that particular skill recording."""
    # Base style for the bar; toggling 'display' controls visibility
    base_style = {
        'position': 'fixed',
        'left': 0, 'right': 0, 'bottom': 0,
        'zIndex': 9999,
        'padding': '10px 14px',
        'backgroundColor': '#111',
        'color': '#eee',
        'fontFamily': 'monospace',
        'fontSize': '14px',
        'borderTop': '1px solid #444',
        'whiteSpace': 'pre'
    }

    if not clickData or not data or not data.get('files'):
        return dash.no_update, dash.no_update

    action = data['action']
    files = data['files']
    curve = clickData['points'][0].get('curveNumber', 0)
    if curve >= len(files):
        return dash.no_update, dash.no_update
    skill = files[curve]  # trace order == files order in the trajectory figure

    # Template = the object part of the skill name (put1_box -> box); a skill
    # named exactly like the action has no object part -> template = skill.
    template = skill
    for prefix in (f"{action}_", f"{action}1_", f"{action}2_"):
        if skill.startswith(prefix):
            template = skill[len(prefix):]
            break
    cmd = (
        f"ros2 launch skills_manager play_skill_launch.py "
        f"name_skill:={skill} name_template:={template}"
    )

    content = html.Div([
        html.Strong("Command: "),
        html.Code(cmd, style={'whiteSpace': 'pre-wrap'})
    ])

    style = dict(base_style, **{'display': 'block'})
    return content, style

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8077)
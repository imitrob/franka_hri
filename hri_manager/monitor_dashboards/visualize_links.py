import yaml, os
import hri_manager, trajectory_data
import yaml
import os
import plotly.graph_objects as go
import dash
from dash import dcc, html, Input, Output
import numpy as np
from collections import defaultdict

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
        dcc.Graph(id='main-graph', style={'width': '60%', 'height': '90vh', 'display': 'inline-block'}),
        dcc.Graph(id='matrix-display', style={'width': '40%', 'height': '90vh', 'display': 'inline-block'})
    ])
])

# Store data globally
actions = []
objects = []
skill_db = defaultdict(dict)

@app.callback(
    Output('dummy-output', 'children'),
    Input('user-dropdown', 'value'),
    prevent_initial_call=True
)
def load_user_data(selected_filename):
    global actions, objects, skill_db
    
    # Clear previous data
    actions = []
    objects = []
    skill_db = defaultdict(dict)
    
    # Load user data
    with open(f'{hri_manager.package_path}/links/{selected_filename}') as f:
        user_data = yaml.safe_load(f)
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
        marker=dict(size=20, color='#FF9AA2'),
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
        marker=dict(size=20, color='#B5EAD7'),
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
        
        # Double object actions
        if (action, '1') in skill_db or (action, '2') in skill_db:
            # First objects (yellow dashed lines)
            if (action, '1') in skill_db:
                for obj in skill_db[(action, '1')]:
                    if obj not in objects:
                        continue
                    start = next(n for n in action_nodes if n['label'] == action)
                    end = next(n for n in object_nodes if n['label'] == obj)
                    fig.add_trace(go.Scatter(
                        x=[start['x'], end['x']],
                        y=[start['y'], end['y']],
                        mode='lines',
                        line=dict(color='#FFD700', width=2, dash='dot'),
                        hoverinfo='none',
                        name='First object',
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
                        x=[start['x'], end['x']],
                        y=[start['y'], end['y']],
                        mode='lines',
                        line=dict(color='#4682B4', width=1, dash='dashdot'),
                        hoverinfo='none',
                        name='Second object',
                        showlegend=True if so_leg else False  # Only show once
                    ))
                    so_leg = False

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
        xaxis=dict(showgrid=False, zeroline=False, showticklabels=False, range=[-0.1, 1.1]),
        yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
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

# Callback for matrix display
@app.callback(
    Output('matrix-display', 'figure'),
    Input('main-graph', 'clickData'))
def update_matrix(clickData):
    fig = go.Figure()
    if not clickData:
        return fig
    
    try:
        point_index = clickData['points'][0]['pointIndex']
        clicked_trace = clickData['points'][0]['curveNumber']
        
        # Only respond to clicks on action nodes (trace 0)
        if clicked_trace != 0:
            return fig
        
        action = action_nodes[point_index]['label']
        
        # Check if it's a double action
        if (action, '1') in skill_db and (action, '2') in skill_db:
            first_objs = list(skill_db[(action, '1')].keys())
            second_objs = list(skill_db[(action, '2')].keys())
            
            # Create matrix of valid combinations
            matrix = np.zeros((len(first_objs), len(second_objs)))
            annotations = []
            
            for i, f_obj in enumerate(first_objs):
                for j, s_obj in enumerate(second_objs):
                    # Check if both parts exist and objects are different
                    if f_obj in skill_db[(action, '1')] and s_obj in skill_db[(action, '2')] and f_obj != s_obj:
                        matrix[i,j] = 1
                        annotations.append(
                            dict(text="✓",
                                 x=j, y=i, 
                                 xref='x', yref='y',
                                 showarrow=False,
                                 font=dict(color='black'))
                        )
            
            fig = go.Figure(data=go.Heatmap(
                z=matrix,
                x=second_objs,
                y=first_objs,
                colorscale=[[0, 'white'], [1, '#90EE90']],
                showscale=False,
                hoverinfo='none'
            ))
            
            fig.update_layout(
                title=f"Valid pairs for {action}",
                annotations=annotations,
                xaxis_title="Second Object",
                yaxis_title="First Object",
                plot_bgcolor='white',
                yaxis=dict(autorange='reversed')  # To match matrix convention
            )
        elif (action, 'single') in skill_db:
            # Show single object connections
            valid_objs = list(skill_db[(action, 'single')].keys())
            fig.add_trace(go.Scatter(
                x=[0.5] * len(valid_objs),
                y=valid_objs,
                mode='markers',
                marker=dict(size=20, color='#90EE90'),
                text=valid_objs,
                hoverinfo='text'
            ))
            fig.update_layout(
                title=f"Valid objects for {action}",
                xaxis=dict(showgrid=False, showticklabels=False, range=[0, 1]),
                yaxis=dict(title="Objects"),
                plot_bgcolor='white'
            )
        
    except Exception as e:
        print(f"Error updating matrix: {e}")
    
    return fig

if __name__ == '__main__':
    app.run(debug=True, port=8077)
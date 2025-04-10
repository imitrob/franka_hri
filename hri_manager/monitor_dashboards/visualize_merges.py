import dash
from dash import html, dcc, Input, Output, dash_table
import plotly.express as px
import json
import glob
import pandas as pd
from datetime import datetime

app = dash.Dash(__name__)

# Load and preprocess data
save_files = sorted(glob.glob("save_*.json"))
data = [json.load(open(f)) for f in save_files]

def create_timeline_data(entry):
    """Process voice and gesture events into timeline format"""
    timeline = []
    print("voicesm", entry['voice_stamped'])
    # Process voice commands
    if 'voice_stamped' in entry and isinstance(entry['voice_stamped'], list):
        for i, (time, word) in enumerate(entry['voice_stamped']):
            word = list(word.values())[0]
            if isinstance(time, (int, float)) and isinstance(word, str):
                timeline.append({
                    "Event": "Voice",
                    "Start": time,
                    "End": time + 0.2,  # Assume 0.2s duration per word
                    "Label": word,
                    "Details": f"Word {i+1}"
                })
    
    # Process gestures
    if 'gesture_stamped' in entry and isinstance(entry['gesture_stamped'], list):
        for time, gesture in entry['gesture_stamped']:
            gesture = list(gesture.values())[0]
            if isinstance(time, (int, float)) and isinstance(gesture, str):
                timeline.append({
                    "Event": "Gesture",
                    "Start": time,
                    "End": time + 0.5,  # Assume 0.5s duration for gestures
                    "Label": gesture,
                    "Details": "Object interaction"
                })
    
    # Return as DataFrame
    timeline_df = pd.DataFrame(timeline)
    # timeline_df["Start"] = pd.to_datetime(timeline_df['Start'], unit='s')
    # timeline_df["End"] =   pd.to_datetime(timeline_df['End'], unit='s')
    return timeline_df

def parse_scene_objects(scene_text):
    """Convert scene description to structured data"""
    objects = []
    for line in scene_text.split('. '):
        if line.startswith("The "):
            parts = line[4:].split(" is ")
            if len(parts) == 2:
                obj_name = parts[0].strip()
                attributes = parts[1].split()
                obj_data = {
                    "Object": obj_name,
                    "Type": attributes[-1],
                    "Color": next((a for a in attributes if a in ['red', 'green', 'blue']), ''),
                    "Size": next((a for a in attributes if a in ['small', 'medium', 'large', 'big', 'wide']), '')
                }
                objects.append(obj_data)
    return objects

app.layout = html.Div([
    html.Div([
        dcc.Dropdown(
            id='file-selector',
            options=[{'label': f"Run {i}", 'value': i} for i in range(len(save_files))],
            value=0,
            style={'width': '300px'}
        ),
    ], style={'padding': '20px'}),
    
    html.Div(
        style={
            'display': 'grid',
            'grid-template-columns': '1fr 1fr',  # Two equal columns
            'gap': '20px',  # Space between columns
            'margin': '20px 0'
        },
        children=[

        html.Div(id='status-banner', style={'padding': '10px', 'margin-bottom': '0px'}),
            
        html.Div([
            html.Pre(id='config-panel', style={
                'backgroundColor': '#f5f5f5',
                'padding': '0px',
                'overflowX': 'auto'
            })
        ])
    ]),
    html.Div(
        style={
            'display': 'grid',
            'grid-template-columns': '1fr 1fr',  # Two equal columns
            'gap': '20px',  # Space between columns
            'margin': '20px 0'
        },
        children=[
            html.Div([
                html.H4("Ground Truth", className='header'),
                html.Div(id='true-sentence', style={'padding': '10px', 'border': '1px solid #ddd'})
            ]),
            html.Div([
                html.H4("Model Prediction", className='header'),
                html.Div(id='predicted-sentence', style={'padding': '10px', 'border': '1px solid #ddd'})
            ])
        ]
    ),
    dcc.Graph(id='timeline', style={'height': '250px', 'margin': '20px 0'}),
    

    html.Div(
    style={
        'display': 'grid',
        'grid-template-columns': '1fr 1fr',  # Two equal columns
        'gap': '20px',  # Space between columns
        'margin': '20px 0'
    },
    children=[
        html.Div([
            html.H4("Scene Objects"),
            dash_table.DataTable(
                id='scene-table',
                style_cell={'padding': '5px', 'textAlign': 'left'},
                style_header={
                    'backgroundColor': 'lightgrey',
                    'fontWeight': 'bold'
                }
            )
        ], style={'margin': '20px 0', "width": "400px"}),
        
        html.Div([
            html.H4("Action Breakdown"),
            dash_table.DataTable(
                id='action-table',
                columns=[
                    {'name': 'Component', 'id': 'component'},
                    {'name': 'Value', 'id': 'value'}
                ],
                style_cell={'padding': '5px'},
                style_header={
                    'backgroundColor': 'lightgrey',
                    'fontWeight': 'bold'
                }
            )
        ], style={'margin': '20px 0'}),
    ]),

])



    
@app.callback(
    [Output('status-banner', 'children'),
     Output('true-sentence', 'children'),
     Output('predicted-sentence', 'children'),
     Output('timeline', 'figure'),
     Output('scene-table', 'data'),
     Output('scene-table', 'columns'),
     Output('action-table', 'data'),
     Output('config-panel', 'children')],
    [Input('file-selector', 'value')]
)
def update_all_components(selected_index):
    entry = data[selected_index]
    
    # Status Banner
    status_color = 'green' if entry['successful'] else 'red'
    status_banner = html.Div(
        f"Model: {entry['model_name']} | Status: {'SUCCESS' if entry['successful'] else 'FAILURE'}",
        style={
            'backgroundColor': status_color,
            'color': 'white',
            'padding': '10px',
            'borderRadius': '5px'
        }
    )
    
    # Text Comparison
    true_sentence = entry['true_sentence']
    predicted_sentence = entry['predicted_sentence']

    # Create timeline data
    timeline_df = create_timeline_data(entry)
    
    # Handle empty timeline data
    if timeline_df.empty:
        timeline_fig = px.scatter(title="No timeline data available")
        timeline_fig.update_layout(
            xaxis={"visible": False},
            yaxis={"visible": False},
            annotations=[{
                "text": "No timeline data available",
                "xref": "paper",
                "yref": "paper",
                "showarrow": False,
                "font": {"size": 16}
            }]
        )
    else:
        timeline_fig = px.timeline(
            timeline_df, 
            x_start="Start", 
            x_end="End", 
            y="Event",
            color="Event",
            text="Label",
            title="Temporal Sequence of Voice and Gesture Events"
        )
    
    # Rest of your code remains the same...
    scene_objects = parse_scene_objects(entry['scene'])
    scene_columns = [{"name": col, "id": col} for col in ['Object', 'Type', 'Color', 'Size']]
    
    # Action Breakdown
    action_data = []
    if 'predicted' in entry:
        action_str = entry['predicted'].strip('`').split(', ')
        for part in action_str:
            key, sep, val = part.partition(': ')  # Always returns 3 parts
            if sep:  # If separator was found
                action_data.append({'component': key.capitalize(), 'value': val})
            else:
                print(f"Skipping malformed part: {part}")  # Debug output
    
    # Config Panel
    config = json.dumps({
        'model': entry['model_name'],
        'temperature': entry['temperature'],
        'top_p': entry['top_p'],
        'repetition_penalty': entry['repetition_penalty']
    }, indent=2)
    
    return (
        status_banner,
        true_sentence,
        predicted_sentence,
        timeline_fig,
        scene_objects,
        scene_columns,
        action_data,
        config
    )

if __name__ == '__main__':
    app.run(debug=True, port=8075, host='127.0.0.1')
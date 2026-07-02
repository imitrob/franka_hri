#!/usr/bin/env python
"""Dashboard for the merge logs written by ReasoningMerger.save_log().

Each log_*.json in this directory is one merge: the timestamped voice + gesture
words, the merged prediction, and the ground truth. Run:

    python visualize_merges.py      # then open http://127.0.0.1:8075

Reads log_*.json from this script's own directory, so it works from any cwd.
"""
import glob
import json
from pathlib import Path

import pandas as pd
import plotly.express as px
from dash import ALL, Dash, ctx, dcc, html, Input, Output

LOG_DIR = Path(__file__).resolve().parent
PORT = 8075


# Voice/gesture words are [timestamp, "word"]; tolerate the legacy
# probabilistic format where a word was {"word": prob, ...}.
def _word(w):
    return next(iter(w)) if isinstance(w, dict) else w


def load_runs():
    runs = []
    for path in sorted(glob.glob(str(LOG_DIR / "log_*.json")),
                       key=lambda p: int(Path(p).stem.split("_")[1])):
        with open(path) as f:
            d = json.load(f)
        d["_id"] = Path(path).stem
        runs.append(d)
    return runs


def timeline_figure(run):
    """Voice + gesture words laid out on a shared time axis."""
    rows = []
    for t, w in run.get("voice_stamped", []):
        rows.append({"Time [s]": t, "Modality": "Voice", "Word": _word(w)})
    for t, w in run.get("gesture_stamped", []):
        rows.append({"Time [s]": t, "Modality": "Gesture", "Word": _word(w)})
    if not rows:
        return px.scatter(title="(no modality data)")

    df = pd.DataFrame(rows)
    fig = px.scatter(
        df, x="Time [s]", y="Modality", text="Word", color="Modality",
        color_discrete_map={"Voice": "#4C78A8", "Gesture": "#F58518"},
    )
    # cliponaxis=False lets the word labels render past the axis instead of being
    # clipped; the padded x-range gives the first/last words room inside the figure.
    fig.update_traces(textposition="top center", marker_size=14, cliponaxis=False)
    tmin, tmax = df["Time [s]"].min(), df["Time [s]"].max()
    pad = 0.15 * ((tmax - tmin) or 1.0) + 0.05
    fig.update_xaxes(range=[tmin - pad, tmax + pad])
    fig.update_layout(
        showlegend=False, height=240,
        margin=dict(l=70, r=40, t=30, b=40),
        yaxis=dict(categoryorder="array", categoryarray=["Gesture", "Voice"]),
    )
    return fig


app = Dash(__name__)
RUNS = load_runs()


def summary_text():
    if not RUNS:
        return f"No log_*.json in {LOG_DIR}"
    ok = sum(1 for r in RUNS if r.get("successful"))
    models = sorted({r.get("model_name", "?") for r in RUNS})
    return f"{ok}/{len(RUNS)} ok · {', '.join(models)}"


def item_style(selected):
    s = {"padding": "8px 12px", "cursor": "pointer", "borderBottom": "1px solid #eee",
         "fontSize": "13px", "whiteSpace": "nowrap", "overflow": "hidden",
         "textOverflow": "ellipsis", "borderLeft": "3px solid transparent"}
    if selected:
        s.update({"background": "#e8f0fe", "borderLeft": "3px solid #4C78A8"})
    return s


def run_item(i, r):
    ok = r.get("successful")
    return html.Div(
        [html.Span("✓ " if ok else "✗ ",
                   style={"color": "#2a8000" if ok else "#c00", "fontWeight": "bold"}),
         html.Span(f"{r['_id']}: {r.get('predicted_sentence') or '(empty)'}")],
        id={"type": "run-item", "index": i}, n_clicks=0, style=item_style(i == 0),
    )


def _kv(label, value, color="#000"):
    return html.Div([
        html.Span(f"{label}: ", style={"fontWeight": "bold"}),
        html.Span(str(value), style={"color": color}),
    ], style={"margin": "4px 0"})


def detail_view(idx):
    if not RUNS:
        return html.Div(f"No log_*.json found in {LOG_DIR}")
    run = RUNS[idx]
    ok = run.get("successful")
    return html.Div([
        _kv("Result", "✓ success" if ok else "✗ mismatch",
            color="#2a8000" if ok else "#c00"),
        _kv("Predicted", run.get("predicted_sentence") or "(empty)"),
        _kv("Ground truth", run.get("true_sentence", "?")),
        _kv("Model", run.get("model_name", "?")),
        _kv("Scene", run.get("scene") or "(none)", color="#555"),
        html.H4("Timeline", style={"marginTop": "16px", "marginBottom": "0"}),
        dcc.Graph(figure=timeline_figure(run), config={"displayModeBar": False}),
        html.Details([
            html.Summary("Raw LLM output"),
            html.Pre(run.get("predicted", ""),
                     style={"whiteSpace": "pre-wrap", "background": "#f6f6f6",
                            "padding": "10px", "fontSize": "12px",
                            "maxHeight": "300px", "overflow": "auto"}),
        ], style={"marginTop": "12px"}),
    ])


app.layout = html.Div(
    style={"display": "flex", "fontFamily": "sans-serif", "height": "100vh"},
    children=[
        html.Div(  # left sidebar: all logs
            style={"width": "280px", "flex": "0 0 auto", "borderRight": "1px solid #ddd",
                   "overflowY": "auto"},
            children=[
                html.Div("Merge logs", style={"fontWeight": "bold", "padding": "12px"}),
                html.Div(summary_text(), style={"color": "#555", "fontSize": "12px",
                                                 "padding": "0 12px 10px"}),
                html.Div([run_item(i, r) for i, r in enumerate(RUNS)]),
            ],
        ),
        html.Div(  # right: selected run detail
            id="detail", style={"flex": "1 1 auto", "overflowY": "auto",
                                 "padding": "16px 24px", "maxWidth": "900px"},
            children=detail_view(0) if RUNS else detail_view(None),
        ),
    ],
)


@app.callback(
    Output("detail", "children"),
    Output({"type": "run-item", "index": ALL}, "style"),
    Input({"type": "run-item", "index": ALL}, "n_clicks"),
)
def select(_clicks):
    idx = ctx.triggered_id["index"] if ctx.triggered_id else 0
    return detail_view(idx), [item_style(i == idx) for i in range(len(RUNS))]


if __name__ == "__main__":
    app.run(debug=True, port=PORT, host="127.0.0.1")

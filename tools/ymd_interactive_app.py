import traceback

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import pandas as pd
import numpy as np
import os

# ==========================================
# 1. WCZYTANIE DANYCH I GENEROWANIE MOCKÓW
# ==========================================
script_dir = os.path.dirname(__file__)
csv_path = os.path.abspath(os.path.join(script_dir, "..", "build", "yaw_diagram_test.csv"))

if os.path.exists(csv_path):
    df = pd.read_csv(csv_path)
    df.fillna(0, inplace=True)
    print(f"Wczytano plik: {csv_path}")
else:
    print(f"File not found: {csv_path}.")
    df = pd.DataFrame() # Zabezpieczenie

customdata_cols = ['steering', 'slip']
wheels = ['FL', 'FR', 'RL', 'RR']
for w in wheels:
    customdata_cols.extend([f'FxCar_{w}', f'FyCar_{w}', f'load_{w}'])


# ==========================================
# 2. RYSOWANIE GŁÓWNEGO WYKRESU YMD
# ==========================================
def create_ymd_figure(data):
    fig = go.Figure()
    if data.empty:
        return fig

    # Izolinie stałego kąta skrętu (Steering)
    grouped_steer = data.groupby('steering')
    for i, (steer_val, group) in enumerate(grouped_steer):
        sorted_group = group.sort_values(by='slip')
        fig.add_trace(go.Scatter(
            x=sorted_group['latAcc'],
            y=sorted_group['yawMoment'],
            mode='lines+markers',
            line=dict(color='rgba(31, 119, 180, 0.7)', width=1.5),
            marker=dict(size=4, color='rgba(31, 119, 180, 0.9)'),
            name=f'Steer: {steer_val:.2f}°',
            legendgroup='steering',
            showlegend=True if i == 0 else False,
            # Przekazujemy wszystkie kolumny sił!
            customdata=sorted_group[customdata_cols].values.tolist(),
            hovertemplate=(
                "<b>[Constant Steering]</b><br>" +
                "Steering: %{customdata[0]:.2f}°<br>" +
                "Slip: %{customdata[1]:.2f}°<br>" +
                "Ay: %{x:.2f} m/s²<br>" +
                "Mz: %{y:.2f} Nm<br>" +
                "<extra></extra>"
            )
        ))
    
    # Izolinie stałego kąta poślizgu (Slip)
    grouped_slip = data.groupby('slip')
    for i, (slip_val, group) in enumerate(grouped_slip):
        sorted_group = group.sort_values(by='steering')
        fig.add_trace(go.Scatter(
            x=sorted_group['latAcc'],
            y=sorted_group['yawMoment'],
            mode='lines+markers',
            line=dict(color='rgba(214, 39, 40, 0.8)', width=1.5), 
            marker=dict(size=4, color='rgba(214, 39, 40, 0.9)'),
            name=f'Slip: {slip_val:.2f}°',
            showlegend=True,
            customdata=sorted_group[customdata_cols].values.tolist(),
            hovertemplate=(
                "<b>[Constant Slip]</b><br>" +
                "Chassis Slip: %{customdata[1]:.2f}°<br>" +
                "Steering: %{customdata[0]:.2f}°<br>" +
                "Lat Acc (Ay): %{x:.2f} m/s²<br>" +
                "Yaw Moment (Mz): %{y:.2f} Nm<br>" +
                "<extra></extra>"
            )
        ))

    fig.update_layout(
        title="<b>Yaw Moment Diagram</b>",
        xaxis_title="Lateral Acceleration A_y [m/s²]",
        yaxis_title="Yaw Moment M_z [Nm]",
        hovermode="closest",
        template="plotly_white",
        margin=dict(l=40, r=40, t=40, b=40)
    )
    return fig


# ==========================================
# 3. RYSOWANIE SAMOCHODU I WEKTORÓW
# ==========================================
def create_car_figure(forces=None, steer=0, slip=0):
    fig = go.Figure()

    wb_front, wb_rear, track = 1.0, 1.0, 0.7
    car_w, car_l_f, car_l_r = track * 0.8, wb_front * 1.2, wb_rear * 1.2
    wheel_w, wheel_l = 0.15, 0.35

    def rotate(x, y, angle_deg):
        theta = np.radians(angle_deg)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        return x * cos_t - y * sin_t, x * sin_t + y * cos_t

    # --- Chassis ---
    chassis_x = [-car_w, car_w, car_w, -car_w, -car_w]
    chassis_y = [car_l_f, car_l_f, -car_l_r, -car_l_r, car_l_f]
    
    rot_chassis_x, rot_chassis_y = [], []
    for cx, cy in zip(chassis_x, chassis_y):
        rx, ry = rotate(cx, cy, slip)  # Chassis rotates only by slip
        rot_chassis_x.append(rx)
        rot_chassis_y.append(ry)

    fig.add_trace(go.Scatter(
        x=rot_chassis_x, y=rot_chassis_y, fill='toself', mode='lines',
        line=dict(color='black', width=2), fillcolor='rgba(150, 150, 150, 0.3)',
        name="Chassis", showlegend=False, hoverinfo='skip'
    ))

    wheel_data = {
        'FL': (-track, wb_front, slip + steer),
        'FR': (track, wb_front, slip + steer),
        'RL': (-track, -wb_rear, slip),
        'RR': (track, -wb_rear, slip)
    }

    for name, (cx, cy, w_angle) in wheel_data.items():
        # Środek koła obraca się tylko o slip
        rot_cx, rot_cy = rotate(cx, cy, slip)
        
        # Punkty lokalne kół
        wx = [-wheel_w, wheel_w, wheel_w, -wheel_w, -wheel_w]
        wy = [wheel_l, wheel_l, -wheel_l, -wheel_l, wheel_l]
        
        rot_wx, rot_wy = [], []
        for x, y in zip(wx, wy):
            # Same koła obracają się o slip + steer
            rx, ry = rotate(x, y, w_angle)
            rot_wx.append(rx + rot_cx)
            rot_wy.append(ry + rot_cy)

        fig.add_trace(go.Scatter(
            x=rot_wx, y=rot_wy, fill='toself', mode='lines',
            line=dict(color='black', width=2), fillcolor='black',
            name=name, showlegend=False, hoverinfo='skip'
        ))
        
        tire_color = 'black'
        
        if forces:
            fz = forces[name][2]
            
            intensity = max(0, min(1, fz / 3085.0)) 
            
            gray_val = int(220 * (1 - intensity)) 
            tire_color = f'rgb({gray_val}, {gray_val}, {gray_val})'

        fig.add_trace(go.Scatter(
            x=rot_wx, y=rot_wy, fill='toself', mode='lines',
            line=dict(color='black', width=1), fillcolor=tire_color,
            name=name, showlegend=False, hoverinfo='skip'
        ))

        if forces:
            fx, fy, fz = forces[name]
            force_scale = 0.0006
            
            vec_x, vec_y = rotate(fy * force_scale, fx * force_scale, slip)
            
            if abs(vec_x) > 0.01 or abs(vec_y) > 0.01:
                fig.add_annotation(
                    ax=rot_cx, ay=rot_cy,
                    x=rot_cx + vec_x, y=rot_cy + vec_y,
                    xref="x", yref="y", axref="x", ayref="y",
                    showarrow=True, arrowhead=2, arrowsize=1.5, arrowwidth=2.5, arrowcolor="red"
                )

    fig.add_annotation(
        ax=0, ay=1.4, x=0, y=1.7,
        xref="x", yref="y", axref="x", ayref="y",
        showarrow=True, arrowhead=1, arrowsize=2, arrowwidth=2, arrowcolor="rgba(0,0,255,0.4)",
        text="Direction of Travel", font=dict(color="blue", size=10)
    )

    fig.update_layout(
        title=f"Vehicle Orientation<br>(Steer: {steer:.1f}°, Slip: {slip:.1f}°)",
        xaxis=dict(range=[-1.1, 1.1], autorange=False, showgrid=False, zeroline=False, visible=False),
        yaxis=dict(range=[-1.5, 1.6], autorange=False, showgrid=False, zeroline=False, visible=False, scaleanchor="x", scaleratio=1),
        template="plotly_white", margin=dict(l=10, r=10, t=60, b=10)
    )
    return fig

# ==========================================
# 4. INICJALIZACJA APLIKACJI DASH I LAYOUT
# ==========================================
app = dash.Dash(__name__)

app.layout = html.Div(style={'display': 'flex', 'flex-direction': 'row', 'height': '100vh'}, children=[
    
    html.Div(style={'width': '65%', 'padding': '10px'}, children=[
        dcc.Graph(id='ymd-graph', figure=create_ymd_figure(df), style={'height': '100%'})
    ]),
    
    html.Div(style={'width': '35%', 'padding': '10px', 'display': 'flex', 'flex-direction': 'column'}, children=[
        dcc.Graph(id='car-graph', figure=create_car_figure(), style={'flex-grow': '1', 'max-height': '55%'}),
        html.Div(id='force-values', style={
            'padding': '12px', 'font-family': 'Arial', 'font-size': '12px', 
            'background-color': '#f9f9f9', 'border-radius': '10px'
        })
    ])
])

# ==========================================
# 5. CALLBACK
# ==========================================
@app.callback(
    [Output('car-graph', 'figure'),
     Output('force-values', 'children')],
    [Input('ymd-graph', 'hoverData')]
)

def update_car_vectors(hoverData):
    
    try:
        if hoverData is None:
            return create_car_figure(steer=0, slip=0), "Hover over a point on the YMD graph..."

        point_info = hoverData['points'][0]
        
        if 'customdata' not in point_info:
            return create_car_figure(), html.Div(f"BŁĄD: Brak 'customdata' pod kursorem! Dostępne klucze to: {list(point_info.keys())}", style={'color': 'red'})
        # Odczytywanie zdefiniowanych kolumn z customdata
        pt = point_info['customdata']
        steer, slip = pt[0], pt[1]
        
        forces = {
            'FL': [pt[2], pt[3], pt[4]],
            'FR': [pt[5], pt[6], pt[7]],
            'RL': [pt[8], pt[9], pt[10]],
            'RR': [pt[11], pt[12], pt[13]]
        }

        text_info = html.Div([
            html.H3(f"Parameters: Steer {steer:.1f}°, Slip {slip:.1f}°", style={'margin-top': '0'}),
            html.Hr(),
            html.P(f"Front Left (FL): Fx={forces['FL'][0]:.0f} N | Fy={forces['FL'][1]:.0f} N | Load={forces['FL'][2]:.0f} N"),
            html.P(f"Front Right (FR): Fx={forces['FR'][0]:.0f} N | Fy={forces['FR'][1]:.0f} N | Load={forces['FR'][2]:.0f} N"),
            html.P(f"Rear Left (RL): Fx={forces['RL'][0]:.0f} N | Fy={forces['RL'][1]:.0f} N | Load={forces['RL'][2]:.0f} N"),
            html.P(f"Rear Right (RR): Fx={forces['RR'][0]:.0f} N | Fy={forces['RR'][1]:.0f} N | Load={forces['RR'][2]:.0f} N"),
        ])

        return create_car_figure(forces, steer, slip), text_info

    except Exception as e:
        # TWARDE PRZECHWYCENIE BŁĘDU I WYSWIETLENIE GO NA EKRANIE (ZAMIAST UKRYWANIA)
        err_msg = traceback.format_exc()
        return create_car_figure(), html.Pre(f"KRYTYCZNY BŁĄD DASH:\n{err_msg}", style={'color': 'red'})

if __name__ == '__main__':
    app.run(debug=True)
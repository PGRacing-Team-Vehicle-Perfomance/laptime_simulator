import pandas as pd
import plotly.graph_objects as go
import os

script_dir = os.path.dirname(__file__)
csv_path = os.path.abspath(os.path.join(script_dir, "..", "build", "yaw_diagram_test.csv"))

def plot_interactive_ymd(data_path):
    if not os.path.exists(data_path):
        print(f"File not found: {data_path}")
        return

    df = pd.read_csv(data_path)
    fig = go.Figure()

    steer_col = 'steering'
    x_col = 'latAcc'
    y_col = 'yawMoment'
    slip_col = 'slip'

    # 1. CONSTANT STEERING ANGLE ISOLINES
    grouped_steer = df.groupby(steer_col)
    for i, (steer_val, group) in enumerate(grouped_steer):
        sorted_group = group.sort_values(by=slip_col)
        fig.add_trace(go.Scatter(
            x=sorted_group[x_col],
            y=sorted_group[y_col],
            mode='lines+markers',
            line=dict(color='rgba(31, 119, 180, 0.7)', width=1.5),
            marker=dict(size=4, color='rgba(31, 119, 180, 0.9)'),
            name=f'Steer: {steer_val:.2f}°',
            legendgroup='steering',
            showlegend=True if i == 0 else False,
            customdata=sorted_group[[steer_col, slip_col]],
            hovertemplate=(
                "<b>[Constant Steering]</b><br>" +
                "Steering: %{customdata[0]:.2f}°<br>" +
                "Chassis Slip: %{customdata[1]:.2f}°<br>" +
                "Lat Acc (Ay): %{x:.2f} m/s²<br>" +
                "Yaw Moment (Mz): %{y:.2f} Nm<br>" +
                "<extra></extra>"
            )
        ))

    # 2. CONSTANT CHASSIS SLIP ANGLE ISOLINES
    grouped_slip = df.groupby(slip_col)
    for i, (slip_val, group) in enumerate(grouped_slip):
        sorted_group = group.sort_values(by=steer_col)
        fig.add_trace(go.Scatter(
            x=sorted_group[x_col],
            y=sorted_group[y_col],
            mode='lines',
            line=dict(color='rgba(214, 39, 40, 0.5)', width=1, dash='dot'),
            name=f'Slip: {slip_val:.2f}°',
            legendgroup='slip',
            showlegend=True if i == 0 else False,
            customdata=sorted_group[[steer_col, slip_col]],
            hovertemplate=(
                "<b>[Constant Slip]</b><br>" +
                "Chassis Slip: %{customdata[1]:.2f}°<br>" +
                "Steering: %{customdata[0]:.2f}°<br>" +
                "Lat Acc (Ay): %{x:.2f} m/s²<br>" +
                "Yaw Moment (Mz): %{y:.2f} Nm<br>" +
                "<extra></extra>"
            )
        ))

    # PLOT LAYOUT AND STYLING
    fig.update_layout(
        title="<b>Yaw Moment Diagram</b>",
        xaxis_title="Lateral Acceleration A_y [m/s²]",
        yaxis_title="Yaw Moment M_z [Nm]",
        hovermode="closest",
        template="plotly_white",
        showlegend=True
    )

    fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor='LightGray')
    fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='LightGray')

    output_html = os.path.join(script_dir, "ymd_interactive.html")
    fig.write_html(output_html)
    print(f"Generated interactive YMD, saved to: {output_html}")
    

if __name__ == "__main__":
    plot_interactive_ymd(csv_path)
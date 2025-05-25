import pandas as pd
import plotly.express as px

# Load the CSV file
df = pd.read_csv('BibiRecord.csv')

# Ensure the necessary columns are present
required_columns = ['Speed', 'Acceleration', 'Angle']
if not all(col in df.columns for col in required_columns):
    missing = [col for col in required_columns if col not in df.columns]
    raise ValueError(f"Missing columns in CSV: {missing}")


import pandas as pd
import plotly.graph_objects as go

# Average every 10 rows
df_avg = df.groupby(df.index // 138).mean()

# Select only the first 30,000 averaged points
df_avg_subset = df_avg.iloc[:10000]

# Plot
trace = go.Scatter3d(
    x=df_avg_subset['Speed'],
    y=df_avg_subset['Acceleration'],
    z=df_avg_subset['Angle'],
    mode='markers',
    marker=dict(
        size=3,
        color=df_avg_subset['Speed'],  # color by Speed
        colorscale='Viridis',
        opacity=0.7
    )
)

fig = go.Figure(data=[trace])

fig.update_layout(
    title='3D Scatter Plot with Averaged Data (First 30,000 points)',
    scene=dict(
        xaxis_title='Speed',
        yaxis_title='Acceleration',
        zaxis_title='Angle'
    )
)

fig.show()
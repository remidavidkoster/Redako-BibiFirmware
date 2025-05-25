import pandas as pd
import numpy as np
import plotly.graph_objects as go
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

# Load and average the data
df = pd.read_csv('BibiRecord.csv')
df_avg = df.groupby(df.index // 138).mean()
df_avg_subset = df_avg.iloc[:10000]

# Get X, Y, Z
x = df_avg_subset['Speed'].values
y = df_avg_subset['Acceleration'].values
z = df_avg_subset['Angle'].values

# Prepare polynomial features
degree = 3
poly = PolynomialFeatures(degree)
XY_poly = poly.fit_transform(np.column_stack((x, y)))

# Fit the model
model = LinearRegression().fit(XY_poly, z)

# Create a grid to evaluate the model
grid_x, grid_y = np.meshgrid(
    np.linspace(x.min(), x.max(), 100),
    np.linspace(y.min(), y.max(), 100)
)
grid_xy = np.column_stack((grid_x.ravel(), grid_y.ravel()))
grid_xy_poly = poly.transform(grid_xy)
grid_z = model.predict(grid_xy_poly).reshape(grid_x.shape)

# Plot original points
scatter = go.Scatter3d(
    x=x, y=y, z=z,
    mode='markers',
    marker=dict(size=3, color=x, colorscale='Viridis', opacity=0.6),
    name='Data Points'
)

# Plot fitted surface
surface = go.Surface(
    x=grid_x, y=grid_y, z=grid_z,
    colorscale='Viridis',
    opacity=0.7,
    name='Polynomial Surface'
)

# Show the figure
fig = go.Figure(data=[scatter, surface])
fig.update_layout(
    title='3D Polynomial Surface Fit (2nd Order)',
    scene=dict(
        xaxis_title='Speed',
        yaxis_title='Acceleration',
        zaxis_title='Angle'
    )
)
fig.show()

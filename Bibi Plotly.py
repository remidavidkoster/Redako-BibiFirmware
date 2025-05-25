import pandas as pd
import numpy as np
import plotly.graph_objects as go
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from numpy.polynomial.polynomial import Polynomial

# Load and average data
df = pd.read_csv('BibiRecord.csv')
df_avg = df.groupby(df.index // 138).mean()
df_avg_subset = df_avg.iloc[:10000]  # Now it's defined

# Step 1: Filter near speed = 0
filtered_df = df_avg_subset[(df_avg_subset['Speed'] > -0.5) & (df_avg_subset['Speed'] < 0.5)]

# Step 2: Fit Angle = f(Acceleration)
accel = filtered_df['Acceleration'].values
angle = filtered_df['Angle'].values

degree = 3
poly = Polynomial.fit(accel, angle, degree).convert()  # convert() gives standard power basis
coeffs = poly.coef

# Step 3: Evaluate over a range
accel_range = np.linspace(accel.min(), accel.max(), 200)
angle_fit = poly(accel_range)
speed_fixed = np.zeros_like(accel_range)  # For plotting at Speed = 0

# Step 4: Create the fitted curve trace
fitted_curve = go.Scatter3d(
    x=speed_fixed,
    y=accel_range,
    z=angle_fit,
    mode='lines',
    line=dict(color='red', width=4),
    name='Fitted Curve (Speed=0)'
)

# Step 5: Reuse your scatter plot
scatter = go.Scatter3d(
    x=df_avg_subset['Speed'],
    y=df_avg_subset['Acceleration'],
    z=df_avg_subset['Angle'],
    mode='markers',
    marker=dict(size=3, color=df_avg_subset['Speed'], colorscale='Viridis', opacity=0.6),
    name='Data Points'
)

# Plot everything
fig = go.Figure(data=[scatter, fitted_curve])
fig.update_layout(
    title='Fitted Angle vs Acceleration Curve at Speed = 0',
    scene=dict(
        xaxis_title='Speed',
        yaxis_title='Acceleration',
        zaxis_title='Angle'
    )
)
fig.show()

# Optional: print the formula
terms = [f"{c:.4f} * x^{i}" if i > 0 else f"{c:.4f}" for i, c in enumerate(coeffs)]
formula = " + ".join(terms)
print("Fitted formula for Angle vs Acceleration (Speed ≈ 0):")
print("Angle = " + formula)

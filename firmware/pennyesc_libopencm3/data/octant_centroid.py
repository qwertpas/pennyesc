import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# ==========================================
# CONFIGURATION
# ==========================================
LUT_BITS = 7             
SEGMENT_SIZE = 1 << LUT_BITS # 256
TOTAL_LUT_SIZE = 8 * SEGMENT_SIZE # 2048

# Update filenames as needed
# TRAIN_CSV_FILENAME = 'MCUViewer_log_212000_20251223_filt.csv'
TRAIN_CSV_FILENAME = 'MCUViewer_log_132745_20260121_filt.csv'

# ==========================================
# 1. Load Data
# ==========================================
print(f"Loading training data from {TRAIN_CSV_FILENAME}...")
df_train = pd.read_csv(TRAIN_CSV_FILENAME)

# ==========================================
# 2. Octant Logic (Same as before)
# ==========================================
def get_pseudo_index(x, y, seg_size):
    # (Copy the exact function from your previous script here)
    # ... [Assuming same function body as provided in your upload] ...
    abs_x = np.abs(x)
    abs_y = np.abs(y)
    indices = np.zeros_like(x, dtype=int)
    for i in range(len(x)):
        ax = abs_x[i]; ay = abs_y[i]
        if ax == 0 and ay == 0: indices[i] = 0; continue
        octant = 0; ratio = 0.0
        if ax > ay: 
            if x[i] >= 0: octant = (0 if y[i] >= 0 else 7)
            else:         octant = (3 if y[i] >= 0 else 4)
            ratio = ay / ax
        else:       
            if y[i] >= 0: octant = (1 if x[i] >= 0 else 2)
            else:         octant = (6 if x[i] >= 0 else 5)
            ratio = ax / ay
        scaled_ratio = int(ratio * (seg_size - 1))
        if octant % 2 == 1: idx_in_seg = (seg_size - 1) - scaled_ratio
        else:               idx_in_seg = scaled_ratio
        indices[i] = (octant * seg_size) + idx_in_seg
    return indices

# ==========================================
# 3. Centroid Calculation (THE FIX)
# ==========================================
# Calculate the pseudo-index for every sample
train_indices = get_pseudo_index(df_train['magx'].values, df_train['magy'].values, SEGMENT_SIZE)

# Add index to dataframe to group by step
df_train['pseudo_index'] = train_indices

# Calculate the mean Index for each Step
# Note: This simple mean fails if a step cluster wraps around 0/2048. 
# We use a heuristic: if std_dev is huge, unwrap before mean.
step_groups = df_train.groupby('step_count')
centroids_idx = []
centroids_angle = []

for step, group in step_groups:
    indices = group['pseudo_index'].values
    
    # Check for wrapping (e.g. indices like 5 and 2040 in same step)
    if np.max(indices) - np.min(indices) > TOTAL_LUT_SIZE / 2:
        # Unwrap: Add total size to small values
        indices = np.where(indices < TOTAL_LUT_SIZE/2, indices + TOTAL_LUT_SIZE, indices)
        mean_idx = np.mean(indices)
        if mean_idx >= TOTAL_LUT_SIZE: mean_idx -= TOTAL_LUT_SIZE
    else:
        mean_idx = np.mean(indices)
    
    # Calculate True Angle for this step
    # (Using your formula)
    true_angle = ((step / 36.0) * 2 * np.pi- 4*2*np.pi/6.0) % (2 * np.pi) 
    # true_angle = (true_angle + 2*np.pi) % (2*np.pi) # Normalize 0..2pi
    
    centroids_idx.append(mean_idx)
    centroids_angle.append(true_angle)

# Convert to arrays and Sort by Index (crucial for interpolation)
centroids = pd.DataFrame({'idx': centroids_idx, 'angle': centroids_angle})
centroids = centroids.sort_values('idx')

# Handle Angle Wrap for Interpolation
# If angle goes 6.2 -> 0.1, we need it to be monotonic for interp
# Unwrap the centroid angles
unwrapped_angles = np.unwrap(centroids['angle'].values)

# ==========================================
# 4. Generate Smooth LUT
# ==========================================
# We now have ~36 clean points. We interpolate between them.
# To handle the 2048 -> 0 wrap correctly, we pad the data.

# Pad Start: Append the last point (shifted back by one period)
idx_pad_start = centroids['idx'].values[-1] - TOTAL_LUT_SIZE
ang_pad_start = unwrapped_angles[-1] - 2*np.pi

# Pad End: Append the first point (shifted forward by one period)
idx_pad_end = centroids['idx'].values[0] + TOTAL_LUT_SIZE
ang_pad_end = unwrapped_angles[0] + 2*np.pi

# Combine
idx_final = np.concatenate(([idx_pad_start], centroids['idx'].values, [idx_pad_end]))
ang_final = np.concatenate(([ang_pad_start], unwrapped_angles, [ang_pad_end]))

# Create Interpolator
lut_interpolator = interp1d(idx_final, ang_final, kind='linear') # 'cubic' is also an option

# Generate Table
lut_x = np.arange(TOTAL_LUT_SIZE)
lut_y = lut_interpolator(lut_x)
lut_y = np.mod(lut_y, 2*np.pi) # Wrap back to 0..2pi for C array

# ==========================================
# 5. Plotting to Verify
# ==========================================
plt.figure(figsize=(10, 5))
plt.plot(lut_x, lut_y, label='Corrected Smooth LUT')
plt.scatter(centroids['idx'], centroids['angle'], color='red', label='Step Centroids')
plt.title("LUT Generation: Centroid Interpolation")
plt.xlabel("Pseudo Index")
plt.ylabel("Angle (Rad)")
plt.legend()
plt.grid(True)
plt.show()

# ==========================================
# 6. C Code Gen
# ==========================================
print(f"const float OCTANT_LUT[{TOTAL_LUT_SIZE}] = {{")
for i, val in enumerate(lut_y):
    end = ",\n" if (i+1)%8==0 else ", "
    if i == TOTAL_LUT_SIZE-1: end = "\n"
    print(f"{val:.5f}f", end=end)
print("};")
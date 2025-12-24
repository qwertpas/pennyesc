import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# ==========================================
# CONFIGURATION
# ==========================================
LUT_BITS = 8             # Table size = 8 * 2^LUT_BITS (e.g., 8 * 32 = 256 entries)
SEGMENT_SIZE = 1 << LUT_BITS # 32 if bits=5
TOTAL_LUT_SIZE = 8 * SEGMENT_SIZE

# CSV files for training (LUT generation) and testing (error validation)
# TRAIN_CSV_FILENAME = 'MCUViewer_log_171949_20251223.csv_filt.csv'
# TEST_CSV_FILENAME = 'MCUViewer_log_175950_20251223.csv_filt.csv'  # Change to your test CSV

TRAIN_CSV_FILENAME = 'MCUViewer_log_212000_20251223_filt.csv'
TEST_CSV_FILENAME = 'MCUViewer_log_210922_20251223_filt.csv'  # Change to your test CSV

# ==========================================
# 1. Load Training Data from CSV (for LUT generation)
# ==========================================
print(f"Loading training data from {TRAIN_CSV_FILENAME}...")
df_train = pd.read_csv(TRAIN_CSV_FILENAME)

# Extract magnetometer data for training
train_mag_x = df_train['magx'].values
train_mag_y = df_train['magy'].values

# Calculate true angle from step_count (assuming 36 steps per electrical rotation)
# True angle based on step_count: 0 to 2pi
train_true_angle = ((df_train['step_count'].values / 36.0) * 2 * np.pi) % (2 * np.pi) - 4*2*np.pi/6.0

# ==========================================
# 1b. Load Test Data from CSV (for error validation)
# ==========================================
print(f"Loading test data from {TEST_CSV_FILENAME}...")
df_test = pd.read_csv(TEST_CSV_FILENAME)

# Extract magnetometer data for testing
test_mag_x = df_test['magx'].values
test_mag_y = df_test['magy'].values

# Calculate true angle from step_count for test data
test_true_angle = ((df_test['step_count'].values / 36.0) * 2 * np.pi) % (2 * np.pi) - 1*2*np.pi/6.0

# ==========================================
# 2. Define the "Octant-Slope" Logic
# ==========================================
def get_pseudo_index(x, y, seg_size):
    """
    Maps (x, y) to a continuous integer index [0, 8*seg_size).
    This mimics the logic we will use in C.
    """
    abs_x = np.abs(x)
    abs_y = np.abs(y)
    
    # 1. Determine Octant (0..7)
    # 2. Calculate Ratio (0..1)
    # We carefully handle the "Zig-Zag" of the ratio to keep the index monotonic
    
    indices = np.zeros_like(x, dtype=int)
    
    for i in range(len(x)):
        ax = abs_x[i]
        ay = abs_y[i]
        
        # Avoid divide by zero
        if ax == 0 and ay == 0:
            indices[i] = 0
            continue
            
        octant = 0
        ratio = 0.0
        
        if ax > ay: # X dominant (0..45 degs, 135..180, etc.)
            if x[i] >= 0:
                if y[i] >= 0: octant = 0 # 0-45
                else:         octant = 7 # 315-360
            else:
                if y[i] >= 0: octant = 3 # 135-180
                else:         octant = 4 # 180-225
            ratio = ay / ax
        else:       # Y dominant (45..90 degs, etc.)
            if y[i] >= 0:
                if x[i] >= 0: octant = 1 # 45-90
                else:         octant = 2 # 90-135
            else:
                if x[i] >= 0: octant = 6 # 270-315
                else:         octant = 5 # 225-270
            ratio = ax / ay

        # Calculate Index within segment (0..seg_size)
        # For even octants (0, 2, 4, 6), angle increases as ratio increases (y/x goes 0->1)
        # For odd octants (1, 3, 5, 7), angle increases as ratio decreases (x/y goes 1->0)
        
        # NOTE: To make the LUT continuous, we act as if:
        # Oct 0: 0 -> 1
        # Oct 1: 1 -> 0 
        
        scaled_ratio = int(ratio * (seg_size - 1))
        
        if octant % 2 == 1:
            # Odd octants: We want 1.0 ratio to be the "start" of the octant
            # and 0.0 ratio to be the "end" (near 90, 180, etc)
            # Actually, standard logic:
            # Oct 0 (0-45): y/x grows 0->1.
            # Oct 1 (45-90): x/y shrinks 1->0. 
            # If we simply map ratio 0->31, we get discontinuities.
            # We want a single monotonic index 0 -> 8*seg_size.
            
            # Let's map Ratio 1.0 -> 0.0 so it connects with previous octant
            idx_in_segment = (seg_size - 1) - scaled_ratio
        else:
            idx_in_segment = scaled_ratio
            
        indices[i] = (octant * seg_size) + idx_in_segment
        
    return indices

# ==========================================
# 3. Generate the Unified LUT
# ==========================================
# Compute input indices for our training data
train_indices = get_pseudo_index(train_mag_x, train_mag_y, SEGMENT_SIZE)
plt.figure()
plt.plot(train_indices, '.')
plt.title("train_indices from get_pseudo_index")
plt.xlabel("Sample Index")
plt.ylabel("Pseudo-Index")
plt.grid(True)
plt.show()

# Map Input Index -> True Output Angle
# We simply interpolate our data points to fill the table
# Sort by index
sort_mask = np.argsort(train_indices)
sorted_indices = train_indices[sort_mask]
sorted_angles = train_true_angle[sort_mask]

# Handle Unwrap/Wrap for interpolation
sorted_angles = np.unwrap(sorted_angles)

# Create the master LUT (0 to 8*SEGMENT_SIZE)
lut_x = np.arange(TOTAL_LUT_SIZE)
# Interpolate to fill gaps
lut_y = np.interp(lut_x, sorted_indices, sorted_angles)
# Modulo back to 0..2pi
lut_y = np.mod(lut_y, 2*np.pi)

# ==========================================
# 4. Benchmarking/Validation (using test data)
# ==========================================
# Calculate angles using the new "Fast Slope" method
test_indices = get_pseudo_index(test_mag_x, test_mag_y, SEGMENT_SIZE)
fast_angles = lut_y[test_indices] # Direct Lookup

# Calculate standard atan2 for reference (uncorrected)
raw_atan = np.arctan2(test_mag_y, test_mag_x)
raw_atan = np.mod(raw_atan, 2*np.pi)

# Calc Errors
def get_err(meas, truth):
    d = np.abs(meas - truth)
    d = np.minimum(d, 2*np.pi - d)
    return np.degrees(d)

err_fast = get_err(fast_angles, test_true_angle)
err_raw = get_err(raw_atan, test_true_angle)

# Plot
fig, ax = plt.subplots(1, 2, figsize=(12, 5))

# Plot 1: The Unified Transfer Function
ax[0].plot(lut_x, np.degrees(lut_y), '.')
ax[0].set_title(f"Combined LUT (Size {TOTAL_LUT_SIZE})")
ax[0].set_xlabel("Pseudo-Index (Octant + Slope)")
ax[0].set_ylabel("Corrected Angle (Deg)")
ax[0].grid(True)
# ax[0].text(10, 10, "Notice the waviness?\nThat is the calibration\nbaked into the slope lookup!", bbox=dict(facecolor='white', alpha=0.8))

# Plot 2: Error Comparison
# ax[1].plot(err_raw, label="Standard Atan2 (Uncorrected)", alpha=0.5)
ax[1].plot(err_fast, '.', label="Fast Combined LUT", color="red")
ax[1].set_title(
    f"Error: Max {np.max(err_fast):.4f}° (vs {np.max(err_raw):.4f}°), "
    f"Avg {np.mean(err_fast):.4f}°"
)
ax[1].set_ylabel("Error (Degrees)")
ax[1].legend()
ax[1].grid(True)

plt.tight_layout()
plt.show()

# ==========================================
# 5. C Code Generator
# ==========================================
print("\n// CONSTANTS")
print(f"#define LUT_BITS {LUT_BITS}")
print(f"#define SEGMENT_SIZE {SEGMENT_SIZE}")
print(f"#define SEGMENT_MASK {SEGMENT_SIZE - 1}")
print(f"// Combined Octant+Linearization LUT (Size {TOTAL_LUT_SIZE})")
print(f"const float OCTANT_LUT[{TOTAL_LUT_SIZE}] = {{")
for i, val in enumerate(lut_y):
    end = ",\n" if (i+1)%8==0 else ", "
    if i == TOTAL_LUT_SIZE-1: end = "\n"
    print(f"{val:.5f}f", end=end)
print("};")
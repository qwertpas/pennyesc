import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# ==========================================
# CONFIGURATION
# ==========================================
CSV_FILENAME = 'filtered_after_0.1s.csv'  # Your file
LUT_SIZE = 256                   # Size of the C array (power of 2 is best)
USE_SYNTHETIC_DATA = False        # Set to False to load your CSV

# ==========================================
# 1. Load or Generate Data
# ==========================================
if USE_SYNTHETIC_DATA:
    print("Generating synthetic test data...")
    # Simulating the distortion seen in your image (approx 60 deg phase shift, unequal amps)
    steps_total = 36 * 5
    t_angle = np.linspace(0, 10*np.pi, steps_total)
    # MagX (Red): smaller amp, shifted phase
    magx = 15000 * np.cos(t_angle - np.deg2rad(60)) + np.random.normal(0, 200, steps_total)
    # MagY (Green): larger amp, shifted phase
    magy = 32000 * np.cos(t_angle - np.deg2rad(120)) + np.random.normal(0, 200, steps_total)
    step_count = (np.arange(steps_total) % 36).astype(int)
    
    df = pd.DataFrame({'magx': magx, 'magy': magy, 'step_count': step_count})
else:
    print(f"Loading {CSV_FILENAME}...")
    df = pd.read_csv(CSV_FILENAME)

# ==========================================
# 2. Calibration Algorithm
# ==========================================

# A. Calculate Mean Vector for each Step
# We average the readings for each step to remove noise
calib = df.groupby('step_count')[['magx', 'magy']].mean().reset_index()

# B. Define Input vs Output
# Input: The "Wrong" Angle calculated by raw atan2
calib['measured_angle'] = np.arctan2(calib['magy'], calib['magx'])

# Output: The "True" Angle based on the step count (0 to 2pi)
# We assume steps are linearly spaced over one electrical rotation
calib['true_angle'] = (calib['step_count'] / 36.0) * 2 * np.pi

# C. Sort and Unwrap for Interpolation
# We must sort by the measured input to create a valid lookup function
calib = calib.sort_values('measured_angle')

input_angles = calib['measured_angle'].values
output_angles = calib['true_angle'].values

# Crucial: Unwrap the output angles to handle the 0/360 wrap-around
# This ensures the transition from step 35 -> 0 is smooth in the math
output_angles_unwrapped = np.unwrap(output_angles, period=2*np.pi)

# D. Create Interpolator (with circular wrapping)
# We pad the data with -2pi and +2pi to ensure the LUT handles the -180/180 boundary smoothly
in_pad = np.concatenate(([input_angles[-1]-2*np.pi], input_angles, [input_angles[0]+2*np.pi]))
out_pad = np.concatenate(([output_angles_unwrapped[-1]-2*np.pi], output_angles_unwrapped, [output_angles_unwrapped[0]+2*np.pi]))

interpolator = interp1d(in_pad, out_pad, kind='linear')

# E. Generate the Lookup Table (LUT)
# We map inputs from -PI to +PI to the corrected angle
lut_inputs = np.linspace(-np.pi, np.pi, LUT_SIZE)
lut_outputs = interpolator(lut_inputs)
# Modulo 2pi to bring back to 0..2pi range for the final table
lut_outputs = np.mod(lut_outputs, 2*np.pi)

# ==========================================
# 3. Validation Plots
# ==========================================
fig, ax = plt.subplots(1, 2, figsize=(12, 5))

# Plot 1: The "Lissajous" Ellipse
ax[0].plot(df['magx'], df['magy'], 'k.', alpha=0.1, label='Raw Data')
ax[0].plot(calib['magx'], calib['magy'], 'r-o', label='Calibration Locus')
ax[0].set_title('Magnetic Vector (Distorted)')
ax[0].axis('equal')
ax[0].legend()
ax[0].grid(True)

# Plot 2: Error Analysis
raw_calc = np.arctan2(df['magy'], df['magx'])
corrected_calc = interpolator(raw_calc) # Apply calibration with interpolation
ideal_calc = np.unwrap((df['step_count']/36.0)*2*np.pi) # Approximation of true

# Direct lookup simulation (as implemented in C)
# Map angle from [-PI, PI] to [0, LUT_SIZE-1] index
def direct_lookup(angles, lut, lut_size):
    """Simulate C-style direct lookup without interpolation"""
    # Normalize angle to [0, 2*PI] range
    normalized = (angles + np.pi) / (2 * np.pi)
    # Map to index
    indices = (normalized * lut_size).astype(int)
    # Clamp to valid range
    indices = np.clip(indices, 0, lut_size - 1)
    # Direct lookup
    return lut[indices]

corrected_direct = direct_lookup(raw_calc, lut_outputs, LUT_SIZE)
# Unwrap for comparison with ideal
corrected_direct_unwrapped = np.unwrap(corrected_direct, period=2*np.pi)

# Circular error calculation
def get_error(meas, true):
    diff = meas - true
    return np.arctan2(np.sin(diff), np.cos(diff))

err_raw = np.degrees(get_error(raw_calc, ideal_calc))
err_lut = np.degrees(get_error(corrected_calc, ideal_calc))
err_direct = np.degrees(get_error(corrected_direct_unwrapped, ideal_calc))

ax[1].plot(err_raw, label='Raw Error', alpha=0.5)
ax[1].plot(err_lut, label='Corrected (Interpolated)', color='red')
ax[1].plot(err_direct, label='Corrected (Direct Lookup)', color='green', alpha=0.7)
ax[1].set_title(f'Error Reduction\nMax Raw: {np.max(np.abs(err_raw)):.1f}° -> Max Interp: {np.max(np.abs(err_lut)):.1f}° -> Max Direct: {np.max(np.abs(err_direct)):.1f}°')
ax[1].set_ylabel('Electrical Degrees')
ax[1].legend()
ax[1].grid(True)

plt.tight_layout()
plt.show()

# ==========================================
# 4. Generate C Code
# ==========================================
print("\n" + "="*30)
print(" C CODE FOR MICROCONTROLLER")
print("="*30)
print(f"// Linearization LUT (Size {LUT_SIZE})")
print(f"const float MAG_LUT[{LUT_SIZE}] = {{")
for i, val in enumerate(lut_outputs):
    end_char = ",\n" if (i+1) % 8 == 0 else ", "
    if i == LUT_SIZE - 1: end_char = "\n"
    print(f"{val:.5f}f", end=end_char)
print("};")
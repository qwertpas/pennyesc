# Pulse Acceleration Metrics

runs: 42
valid: 34
invalid: 8

## Best Acceleration 20-120ms
dir=+1 advance=130 accel=5835.2 std=2111.6 baseline=2175.6 ripple=0.0572 n=3
dir=+1 advance=150 accel=5366.8 std=1616.8 baseline=2022.7 ripple=0.0647 n=3
dir=+1 advance=90 accel=4899.4 std=348.4 baseline=1913.5 ripple=0.0666 n=2
dir=+1 advance=110 accel=4613.7 std=1971.7 baseline=1868.9 ripple=0.0602 n=3
dir=+1 advance=70 accel=2179.9 std=1642.2 baseline=2104.8 ripple=0.0976 n=2
dir=+1 advance=50 accel=-1924.4 std=1959.9 baseline=1901.2 ripple=0.1692 n=3
dir=-1 advance=50 accel=6140.4 std=1011.6 baseline=2301.3 ripple=0.0613 n=3
dir=-1 advance=30 accel=5286.3 std=1622.3 baseline=1986.1 ripple=0.0652 n=3
dir=-1 advance=70 accel=4683.9 std=443.1 baseline=1982.4 ripple=0.0575 n=3
dir=-1 advance=90 accel=3105.8 std=1988.5 baseline=2045.6 ripple=0.0625 n=3
dir=-1 advance=110 accel=2600.9 std=212.2 baseline=2196.1 ripple=0.0957 n=3
dir=-1 advance=130 accel=1398.0 std=2025.8 baseline=2209.5 ripple=0.1633 n=3

## Invalid
dir=+1 advance=30 repeat=0 note=angle_not_monotonic
dir=+1 advance=30 repeat=1 note=angle_not_monotonic
dir=+1 advance=30 repeat=2 note=TimeoutError
dir=+1 advance=70 repeat=1 note=TimeoutError
dir=+1 advance=90 repeat=1 note=TimeoutError
dir=-1 advance=150 repeat=0 note=angle_not_monotonic
dir=-1 advance=150 repeat=1 note=angle_not_monotonic
dir=-1 advance=150 repeat=2 note=angle_not_monotonic

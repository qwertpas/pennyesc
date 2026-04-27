# Metric-Only Dynamic Metrics Report

raw run: firmware/dynamic_metrics/run_20260427_124731_metrics_scored
metric runs: 56
valid: 29
invalid: 27

invalid reasons: angle_not_monotonic=16, mct_fault_delta=18, no_motion=3

## Best Speed
dir=-1 duty=100 advance=50 speed=2041.2 early=6178.6 t3000=80.0ms ripple=0.1844
dir=-1 duty=150 advance=30 speed=4307.5 early=14250.0 t3000=24.0ms ripple=0.0820
dir=-1 duty=200 advance=30 speed=6009.1 early=12614.7 t3000=10.0ms ripple=0.0613
dir=-1 duty=250 advance=30 speed=7788.1 early=14747.8 t3000=10.0ms ripple=0.0540
dir=+1 duty=100 advance=150 speed=2707.5 early=9764.3 t3000=58.0ms ripple=0.1535
dir=+1 duty=150 advance=110 speed=4908.7 early=9000.3 t3000=12.0ms ripple=0.0650
dir=+1 duty=200 advance=70 speed=4431.9 early=1559.1 t3000=10.0ms ripple=0.0889
dir=+1 duty=250 advance=30 speed=8743.5 early=80484.4 t3000=44.0ms ripple=0.0658

## Best Early Acceleration 20-100ms
dir=-1 duty=100 advance=50 speed=2041.2 early=6178.6 t3000=80.0ms ripple=0.1844
dir=-1 duty=150 advance=50 speed=4115.4 early=16109.9 t3000=20.0ms ripple=0.1031
dir=-1 duty=200 advance=50 speed=5694.2 early=13185.9 t3000=12.0ms ripple=0.0633
dir=-1 duty=250 advance=30 speed=7788.1 early=14747.8 t3000=10.0ms ripple=0.0540
dir=+1 duty=100 advance=150 speed=2707.5 early=9764.3 t3000=58.0ms ripple=0.1535
dir=+1 duty=150 advance=90 speed=4688.4 early=15054.3 t3000=12.0ms ripple=0.0682
dir=+1 duty=200 advance=30 speed=1155.3 early=1862.9 t3000=infms ripple=0.2380
dir=+1 duty=250 advance=30 speed=8743.5 early=80484.4 t3000=44.0ms ripple=0.0658

## Best Time To 3000 RPM
dir=-1 duty=100 advance=70 speed=2018.5 early=1015.2 t3000=42.0ms ripple=0.1652
dir=-1 duty=150 advance=70 speed=4113.7 early=1705.7 t3000=16.0ms ripple=0.0980
dir=-1 duty=200 advance=30 speed=6009.1 early=12614.7 t3000=10.0ms ripple=0.0613
dir=-1 duty=250 advance=30 speed=7788.1 early=14747.8 t3000=10.0ms ripple=0.0540
dir=+1 duty=100 advance=90 speed=2615.7 early=5935.5 t3000=18.0ms ripple=0.1451
dir=+1 duty=150 advance=90 speed=4688.4 early=15054.3 t3000=12.0ms ripple=0.0682
dir=+1 duty=200 advance=70 speed=4431.9 early=1559.1 t3000=10.0ms ripple=0.0889
dir=+1 duty=250 advance=30 speed=8743.5 early=80484.4 t3000=44.0ms ripple=0.0658

## Lowest Ripple
dir=-1 duty=100 advance=70 speed=2018.5 early=1015.2 t3000=42.0ms ripple=0.1652
dir=-1 duty=150 advance=30 speed=4307.5 early=14250.0 t3000=24.0ms ripple=0.0820
dir=-1 duty=200 advance=70 speed=5592.6 early=11275.9 t3000=10.0ms ripple=0.0553
dir=-1 duty=250 advance=30 speed=7788.1 early=14747.8 t3000=10.0ms ripple=0.0540
dir=+1 duty=100 advance=90 speed=2615.7 early=5935.5 t3000=18.0ms ripple=0.1451
dir=+1 duty=150 advance=110 speed=4908.7 early=9000.3 t3000=12.0ms ripple=0.0650
dir=+1 duty=200 advance=70 speed=4431.9 early=1559.1 t3000=10.0ms ripple=0.0889
dir=+1 duty=250 advance=30 speed=8743.5 early=80484.4 t3000=44.0ms ripple=0.0658

## Clean Valid Runs
dir=-1 duty=100: a50 speed=2041 ripple=0.184, a70 speed=2019 ripple=0.165, a90 speed=1903 ripple=0.207, a110 speed=1631 ripple=0.214
dir=-1 duty=150: a30 speed=4308 ripple=0.082, a50 speed=4115 ripple=0.103, a70 speed=4114 ripple=0.098, a90 speed=3661 ripple=0.103, a110 speed=2678 ripple=0.155
dir=-1 duty=200: a30 speed=6009 ripple=0.061, a50 speed=5694 ripple=0.063, a70 speed=5593 ripple=0.055, a110 speed=3501 ripple=0.120
dir=-1 duty=250: a30 speed=7788 ripple=0.054
dir=+1 duty=100: a50 speed=1776 ripple=0.203, a70 speed=2224 ripple=0.150, a90 speed=2616 ripple=0.145, a110 speed=2683 ripple=0.156, a130 speed=2609 ripple=0.167, a150 speed=2708 ripple=0.154
dir=+1 duty=150: a50 speed=2246 ripple=0.145, a70 speed=3843 ripple=0.089, a90 speed=4688 ripple=0.068, a110 speed=4909 ripple=0.065, a130 speed=4656 ripple=0.078, a150 speed=4793 ripple=0.071
dir=+1 duty=200: a30 speed=1155 ripple=0.238, a70 speed=4432 ripple=0.089
dir=+1 duty=250: a30 speed=8743 ripple=0.066

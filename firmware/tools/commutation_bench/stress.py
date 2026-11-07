import sys
from seq import run_seq, Esc
from pny import PsuLogger
e = Esc()
tests = {
  'hold400_8s': [(8.0, 400), (0.8, 0)],
  'reverse400': [(2.0, -400), (0.8, 0)],
  'flip': [(1.5, 400), (1.5, -400), (1.5, 400), (0.8, 0)],
  'steps': [(0.7, 150), (0.7, 450), (0.7, 200), (0.7, 500), (0.7, 300), (0.8, 0)],
  'd500': [(2.5, 500), (0.8, 0)],
  'd600': [(2.5, 600), (0.8, 0)],
}
for name in sys.argv[1:]:
    p = PsuLogger(); p.start()
    run_seq(e, tests[name], name)
    p.stop()
    amps = [a for _, _, a in p.rows]
    volts = [v for _, v, _ in p.rows]
    print(f"   supply: max {max(amps):.2f} A, V range {min(volts):.2f}-{max(volts):.2f}")

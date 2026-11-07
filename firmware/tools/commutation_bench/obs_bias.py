# Exact-arithmetic replica of observer_ab_update + observer_position_at_tick,
# driven by a rotor with constant acceleration. Reports the electrical-angle
# error of the commutation phase vs the a=0 case.
import numpy as np
PP=6
def pdelta(v,us): return (v*(us*4295))>>32
def run(a_rpm_s, v0_rpm=30000, T=100, lead=120, tau=120, n=400, alpha=8, beta=128):
    # true position in turn16 (float), velocity turn16/s
    v0=v0_rpm/60*65536; a=a_rpm_s/60*65536
    th=lambda t: v0*t+0.5*a*t*t          # t in s
    pos=int(th(0-tau*1e-6))&0xffff; vel=int(v0); rem=0
    errs=[]
    for k in range(1,n):
        t=k*T*1e-6
        meas=int(round(th(t-tau*1e-6)))&0xffff   # data is tau old at stamp time
        dt=T
        pred=pos+pdelta(vel,dt)
        e=((meas-pred)+32768)%65536-32768
        pos=(pred+int(e/alpha))&0xffff
        num=e*(1000000//beta)+rem
        c=int(num/dt); rem=num-c*dt; vel+=c
        if k>n//2:
            # evaluate commutation phase across the next interval (control tick ~ +60us, edges thereafter)
            for s in np.linspace(0,T,11)[:-1]:
                est=pos+pdelta(vel,int(s)+lead)
                tru=th(t+s*1e-6)
                d=((est-tru)%65536+32768)%65536-32768
                errs.append(d)
    return np.mean(errs)*PP*360/65536, np.std(errs)*PP*360/65536
base,_=run(0)
print('steady offset (elec deg, lead==tau):',round(base,2))
for a in [-1.4e6,-1e6,-0.5e6,-2.4e5,0,2.4e5,1e6]:
    m,s=run(a)
    print(f"a={a/1e6:+.2f}M rpm/s  commutation error {m-base:+7.1f} deg elec (+=ahead/over-advanced)  ripple sd {s:.1f}")
for T in [100,125]:
    m,_=run(-1.4e6,T=T); b,_=run(0,T=T)
    print('T',T,'decel1.4M bias',round(m-b,1))

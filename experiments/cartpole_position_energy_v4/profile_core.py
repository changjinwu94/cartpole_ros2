"""Fixed geometry and reference-profile bounds, no hardware dependencies."""
import math
SCALE=.04/(3000*9.7)
CENTRE=.0735
PERIOD=.25
EXPECTED={'GAC':13092,'GDEC':3928,'GSP':4000,'GPOR':5,'GI':30,
          'GPP':64,'GPD':2,'GPC':1620,'GCC':540}

def targets(start):return (start+11000,start-9000)
def cart_position(counts,reference):
    return reference['cart_m']+(counts-reference['position_counts'])*SCALE

def validate_reference(r,now_ns,boot):
    if r.get('boot_id')!=boot:raise ValueError('Reference is from a different Pi boot')
    age=(now_ns-r['captured_ns'])/1e9
    if not 0<=age<=120:raise ValueError('Reference older than 120s; capture again')
    if not .070<=r['cart_m']<=.077:raise ValueError('Capture cart near centre70–77mm')
    if r.get('counts_per_rev')!=3000 or r.get('gear_ratio')!=9.7 or r.get('pulley_m')!=.04:
        raise ValueError('Unexpected reference geometry')
    return age

def position_guard(pos,reference):
    x=cart_position(pos,reference)
    return math.isfinite(x) and .050<=x<=.100 and abs(pos-reference['position_counts'])*SCALE<=.022

def pole_angle(count,down):return ((count-down)*2*math.pi/2000+2*math.pi)%(2*math.pi)-math.pi

def parse_frame(raw):
    if not raw.endswith(b'\n'):raise ValueError('Incomplete frame')
    f=raw.decode('ascii').strip().split(',')
    if len(f)!=9 or f[0]!='CP1':raise ValueError('Malformed frame')
    seq,us,count,left,right,z,bad,boot=map(int,f[1:])
    if left not in (0,1) or right not in (0,1) or z not in (0,1) or boot!=1 or bad:
        raise ValueError('Invalid sensor/encoder state')
    return dict(sequence=seq,board_us=us,count=count,left=bool(left),right=bool(right))

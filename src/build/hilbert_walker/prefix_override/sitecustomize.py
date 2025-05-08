import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dgy/fractal_walker_ws/src/install/hilbert_walker'

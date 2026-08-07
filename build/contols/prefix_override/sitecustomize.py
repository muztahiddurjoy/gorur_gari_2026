import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/muz/gorur_gari/gorur_gari_2026/install/contols'

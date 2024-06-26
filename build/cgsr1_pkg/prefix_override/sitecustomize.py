import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/snowlar/Desktop/SNOWLAR/install/cgsr1_pkg'

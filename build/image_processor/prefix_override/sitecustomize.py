import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/easha/git/Pick-Place_RS2-1/install/image_processor'

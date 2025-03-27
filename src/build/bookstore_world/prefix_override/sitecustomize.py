import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/niweshsah/py_trees1_ws/src/install/bookstore_world'

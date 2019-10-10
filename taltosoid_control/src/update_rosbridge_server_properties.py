#!/usr/bin/env python

'''
TODO documentation
'''

import sys
import json

from find_ip import get_ip


fout = open(sys.argv[1],'w')
json.dump({
    'host': get_ip(),
    'port': sys.argv[2]
}, fount, indent=2)
fout.close()

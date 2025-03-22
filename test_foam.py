#!/usr/bin/env python
from pprint import pprint
from xtc3d.foam import read_foam
from xtc3d import config

panels=read_foam(config)
pprint(panels)
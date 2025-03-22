#!/usr/bin/env python
"""Extract the elevations from the xtc file.
The awk method did not allow for external configuration. This version
takes all the T4 records and picks out the x,y values directly from their fields
For z takes either column 8 (origin 0). If > 0 then keeps the record.
"""
from os import path
from xtc3d import config,logger

def main():
  fn=config['source']

  data_info=config['data']
  folder=data_info["folder"]
  ofn=path.join(folder,data_info["elevations"])

  logger.info("Reading file %s" % ofn)

  with open(ofn,'w') as out_file:
    out_file.write('x,y,y\n')
    with open(fn) as in_file:
      lines = [line.rstrip() for line in in_file]
    for line in lines:
      line=line.replace("\t","")
      if line.startswith('T4'):
        values=line.split(' ')
        z_fld=8
        z=float(values[z_fld])
        if z > 0:
          x=float(values[2])
          y=float(values[3])
          out=format(f"{x:.6f},{y:.6f},{z:.6f}\n")
          out_file.write(out)
  logger.info("Wrote file %s" % ofn)

if __name__=="__main__":
  main()
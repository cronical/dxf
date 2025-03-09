#!/usr/bin/env python
"""Extract the elevations from the xtc file.
The awk method was too simplistic. This version
takes all the T4 records and picks out the x,y values directly from their fields
but for z takes either column 8 (origin 0) or if that is zero, then column 13.
"""

fn="/Users/george/track_plans/nmra-wimrr-2-flat.xtc"
ofn="data/elevations.csv"

with open(ofn,'w') as out_file:
  out_file.write('x,y,y\n')
  with open(fn) as in_file:
    lines = [line.rstrip() for line in in_file]
  output=[]
  for line in lines:
    line=line.replace("\t","")
    if line.startswith('T4'):
      values=line.split(' ')
      z_fld=8
      z=float(values[z_fld])
      if z == 0:
        z_fld=13
        z=float(values[z_fld])
      x=float(values[2])
      y=float(values[3])
      out=format(f"{x:.6f},{y:.6f},{z:.6f}\n")
      out_file.write(out)

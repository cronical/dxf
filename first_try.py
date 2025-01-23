#!/usr/bin/env python
import sys
import ezdxf
from ezdxf.math import ConstructionArc

try:
    doc = ezdxf.readfile("data/small.dxf")
except IOError:
    print("Not a DXF file or a generic I/O error.")
    sys.exit(1)
except ezdxf.DXFStructureError:
    print("Invalid or corrupted DXF file.")
    sys.exit(2)
# helper function
def process_entity(msp,e):
    """Remove the continuous lines, leaving only the center line
    """
    typ=e.dxf.linetype


    match e.dxftype():
        case 'ARC':
            print (f"{typ} {e.dxftype()} radius: {e.dxf.radius} center: {e.dxf.center} {e.dxf.start_angle} {e.dxf.end_angle}")
        case 'LINE':
            print (f"{typ} {e.dxftype()} {e.dxf.start} at {e.dxf.start.angle_deg}- {e.dxf.end} at {e.dxf.end.angle_deg}")
        case _:
            pass

    match typ:
        case "DASHED":
            pass
        case "CONTINUOUS":
            e.destroy() # this just marks for deletion until purge is called
            print ("\tdeleted")
        case _:
            pass

# iterate over all entities in modelspace
msp = doc.modelspace()
print (f"Starting with {len(msp)} entities")
for ix,e in enumerate(msp):
    print(ix)
    if e.dxftype() in ("ARC","LINE"):
        process_entity(msp,e)
    else:
        print(e.dxftype())
msp.purge()
print (f"Reduced to {len(msp)} entities by taking only the center lines")

#create another line above each element on the z axis
# base=list(msp)
# offset=0.5
# def add_vertical(msp,low3,offset):
#     """Add the vertical line. This adds it to the collection
#     then removes it if needed. It may have already existed.
#     This requires a purge when all done.
#     """
#     high3=list(low3)
#     high3[2]=offset
#     line=msp.add_line(low3,high3)
#     sel=[line==t for t in msp]
#     if sum(sel) > 1:
#         line.destroy() # oops it was already there
#     return msp

# for ix,e in enumerate(base):
#     match e.dxftype():
#         case "LINE":
#             start=list(e.dxf.start)
#             start[2]=offset
#             end=list(e.dxf.end)
#             end[2]=offset
#             line=msp.add_line(start,end)# the line offset above the input line

#             msp=add_vertical(msp,e.dxf.start,offset)# the vertical lines connecting the start points
#             msp=add_vertical(msp,e.dxf.end,offset)# the vertical line connecting the end points
                
#         case "ARC":
#             center=list(e.dxf.center)
#             center[2]=offset
#             radius=e.dxf.radius
#             start_angle=e.dxf.start_angle
#             end_angle=e.dxf.end_angle
#             msp.add_arc(center,radius,start_angle,end_angle)
#             msp=add_vertical(msp,e.start_point,offset)# the vertical lines connecting the start points
#             msp=add_vertical(msp,e.end_point,offset)# the vertical line connecting the end points
#             pass

# msp.purge()       
print (f"Ended with {len(msp)} entities")

fn="data/revised.dxf"
doc.saveas(fn)
print(f"Wrote to {fn}")

"""Take the roadbed file and cut into sections for the foam.
This is to be run as a blender script, once for each work_piece.
The caller specifies the file containing the roadbed which is assumed open.
Run this with

/Applications/Blender.app/Contents/MacOS/Blender $file --python $script -- args

where args is one required positional value which is the part_no from xtrkcad of the foam.
"""

import argparse
from os.path import join
import sys

import bpy
from mathutils import Vector

from xtc3d import logger,config, DATA_FOLDER,SCALE, SCENE_HEIGHT
from xtc3d.io import save_file
from xtc3d.transform import delete_objects_by_name

def read_foam(config):
    """Get the vertices and other attributes of the foam panels
    vertices are defined in the xtrackcad file in layer "foam"
    other attributes are from the config file
    returns list defining the panels
    the values are dicts with keys (values):
    - panel_name (String f"foam_{part_no}")
    - part_no (int)
    - height (float)
    - source (string indicating source stock from which to cut this panel)
    - vertices (list of 4 x,y tuples)

    If the config does not match the foam layer definitions, will return None
    """

    xtc_fn=config['source']
    layer=config['layers']['foam']
    work_pieces=config['work_pieces']

    logger.info(f"Getting foam definitions from file {xtc_fn} layer {layer}" )
    panels={}
    layer=f"{layer-1}"

    # read the vertices
    with open(xtc_fn) as in_file:
        lines = [line.rstrip() for line in in_file]
        in_object=False
        for line in lines:
            line=line.replace("\t","")
            values=line.split(' ')
            match in_object:
                case False:
                    if (values[0]=='DRAW') and (values[2]==layer):
                        part_no=values[1]
                        in_object=True
                        points=[]
                case True:
                    match values[0]:
                        case 'Y4':
                            continue
                        case 'END$SEGS':
                            assert len(points)==4,"wrong number of points collected for foam"
                            in_object=False
                            panels[part_no]=points
                        case _:
                            x=round(float(values[0]),6)
                            y=round(float(values[1]),6)
                            points.append((x,y))
    err_cnt=0
    panel_found=dict(zip(panels.keys(),[False]*len(panels)))
    for work_piece in work_pieces:
        part_no=work_piece["part_no"]
        part_no=str(part_no)
        if part_no not in panels:
            logger.error(f"Defined part_no {part_no} not found in xtc file.")
            err_cnt+=1
        else:
            work_piece["vertices"]=panels[part_no]
            panel_found[part_no]=True
    if not all(panel_found.values()):
        logger.error("Work piece definitions do not include:")
        for part_no,found in panel_found:
            if found:
                continue
            logger.error(f"  Part number {part_no}")
            err_cnt+=1
    result=[]
    for _,defn in enumerate(work_pieces):
        defn["panel_name"]=f"foam_{defn['part_no']}"
        result.append(defn)
    logger.info("%s foam panels defined" % len(result))
    if err_cnt>0:
        return None
    return result

def add_foam_or_intersect(foam_defn,height=SCENE_HEIGHT ):
    """Adds one new object to either 
    - intersect with the roadbed (non-default height)
    - represent the base foam (default height). 
    foam_defn is panel definition.  See read_foam.
    Assumes panels are all aligned with the x and y axes.

    returns panel name for use in intersection step
    """

    foam_points=foam_defn["vertices"]
    logger.debug(foam_points)
    anchor=foam_points[0]
    for p in foam_points[1:]:
        x,y=(p[0]-anchor[0],p[1]-anchor[1])
        logger.debug(f"x,y: {(x,y)}")
        if any([abs(n)<.001 for n in (x,y)]):
            continue
        center=Vector((anchor[0]+x/2, anchor[1]+y/2,height/2))* SCALE
        logger.debug(f"Center: {center}")
        break
    bpy.ops.mesh.primitive_cube_add(location=center)
    for obj in bpy.data.objects:
        if obj.name=='Cube':
            obj.name=foam_defn["panel_name"]
            dims=Vector((abs(x),abs(y),height)) * SCALE
            obj.dimensions=dims
            return obj.name

def make_work_piece(foam_defn):
    """Cuts off a piece of the complete model as a work piece and saves as a separate file.
    Assumes complete model already loaded in blender.
    foam_defn is dict documented in read_foam.
    Steps:
    - creating a cube with x,y location and dimension as defined with Z larger than highest point
    - intersecting that with the roadbed
    - removing that intersector
    - create new cube with z to the desired panel thickness
    - saving as the panel_name.blend
    """

    # locate the roadbed - there should be only one object, but apply filter just in case
    for roadbed in bpy.data.objects:

      if roadbed.name.startswith("foam"):
          continue
      if roadbed.name.endswith("_"): # hack to avoid referencing object not in view
          continue
      break

    intersector_name = add_foam_or_intersect(foam_defn,height=5)


    # apply the boolean intersection
    logger.info(f"Intersecting {roadbed.name} using intersector: {intersector_name}")
    roadbed.select_set(True) 
    bpy.context.view_layer.objects.active = roadbed

    bpy.ops.object.modifier_add(type='BOOLEAN')
    mname=roadbed.modifiers[-1].name
    bpy.context.object.modifiers[mname].object=bpy.data.objects[intersector_name] 
    bpy.context.object.modifiers[mname].operation = "INTERSECT"
    bpy.context.object.modifiers[mname].use_self=True
    bpy.ops.object.modifier_apply(modifier=mname)
    
    delete_objects_by_name([intersector_name]) 

    add_foam_or_intersect(foam_defn)

    work_piece_file=join(DATA_FOLDER,f"{intersector_name}.blend")
    save_file(work_piece_file)



def main():
  logger.info("Starting foam script")
  argv = sys.argv
  if  "--" not in argv:
      logger.error("Pass script arguments after ' -- '")
  argv = argv[argv.index("--") + 1:]  # get all args after "--"
  logger.info(argv)
  foam_panels=read_foam(config) # do error checking before starting the blender stuff
  part_nos=[a["part_no"]for a in foam_panels]
  parser = argparse.ArgumentParser()
  parser.add_argument("foam_part_no",type=int,choices=part_nos)
  args = parser.parse_args(argv)
  pix=part_nos.index(args.foam_part_no)
  foam_defn=foam_panels[pix]

  make_work_piece(foam_defn)

if __name__=="__main__":
    main()
"""collection of diagnostic functions
"""
import bpy
import bmesh
from xtc3d import SCALE,logger

def show_face(face,indent=None):
    """Show vertices for purpose of debugging. 
    """
    if indent:
        msg=(' '*indent)
        points=[]
        for v in face.verts:
            inches=tuple(v.co / SCALE)
            inches=tuple((round(a,6) for a in inches))
            points+=[format(f"{inches}")]
        logger.debug(msg+', '.join(points))

def show_edge(edge,indent=None):
    """Show vertices for purpose of debugging. 
    """
    if indent:
        msg=(' '*indent)
        points=[]
        for v in edge.verts:
            inches=tuple(v.co / SCALE)
            inches=tuple((round(a,6) for a in inches))
            points+=[format(f"{inches}")]
        logger.debug(msg+', '.join(points))

def show_edges(obj_name,edge_index_set):
    """ Diagnostic to show the edge set visually.  Selects the edges and exits
    Although it does not set the crossing edges as selected, it looks like it does in the view.
    This seems to be because each edge selection also selects the vertices and the view automatically 
    creates the selection when two vertices of an edge are selected.  These do not get done when
    the selection is consumed by the bevel operation.
    """
    bpy.ops.object.select_all(action='DESELECT')
    obj=bpy.data.objects[obj_name]
    obj.select_set(True)
    me = obj.data
    bm=bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.select_flush(True) # probably not needed.
    bm.select_mode={"EDGE"}
    

    logger.debug(f"Showing {len(edge_index_set)} edges")
    for edge in bm.edges:
        edge.select=False
    for eix in edge_index_set:
        edge=bm.edges[eix]
        # edge.select_set(True)
        edge.select=True
    cnt=0
    for edge in bm.edges:
        if edge.select:
            cnt+=1
    logger.debug(f"Selection is {cnt} edges")
    
    cnt=0
    for vert in bm.verts:
        if vert.select:
            cnt+=1
    logger.debug(f"Selection is {cnt} verts")
   

    bm.to_mesh(me)
    me.update()
    bm.clear()

    bm.free()
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_mode(type='EDGE')
    #bpy.ops.view3d.view_selected(use_all_regions=False)
    raise KeyboardInterrupt()

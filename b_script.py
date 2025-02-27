"""
Run inside of Blender

"""
from itertools import compress
import logging
import os
from math import dist
from pprint import pprint

import bpy
import bmesh
import debugpy
from mathutils import Matrix, Vector
from mathutils import geometry 
import numpy as np


logger = logging.getLogger(__name__)
logger.info("--------------- Start of xtrackcad import -------------------")
logger.setLevel(logging.INFO)
bl_info = {
    "name": "XTrackCad to 3D",
    "author": "George Dobbs",
    "version": "0.0.1",
    "location": "File > Import > XTrackCAD",
    "description": "Take the exported DXF file and the elevations file and created a 3D model",
    "blender": (4, 3, 0),
    "category" : "Import-Export"
}

__version__ = '.'.join([str(s) for s in 
                        bl_info['version']])

XTC_LAYER_TYPES={"XTRKCAD3_curve_":"level","XTRKCAD17_curve_":"inclined"}
LAYER_TYPES_TO_XTC={v: k for k, v in XTC_LAYER_TYPES.items()}

SCALE=0.0254 # inches
HALF_WIDTH=0.9375 # half of the roadbed width

def vertex_coordinates(vert:bpy.types.MeshVertex,matrix_world:Matrix)->tuple:
    """Return the x,y coordinates of a vertex as a tuple of floats
    """
    scale = 39.370079 # inches per meter
    return scale * matrix_world @ vert.co    

def get_z(coords,elevations:np.array,matrix_world:Matrix,context="")->float:
    """Given the coordinates of a vertex, coords
    the end_point array - passed as a numpy array
    the matrix world from the object holding the vertex:
    Check vertex against imported data to locate z value (within a tolerance)
    Return the xy,z value if found, otherwise None
    The xy value is from the vertex (not the elevation file)
    """
    # matching tolerance to that used to merge near vertices.
    tolm=.003 # meters
    tol= tolm / SCALE
    vectors=[Vector(elevations[i,:]).xy for i in range(len(elevations))]
    distance=[dist(v,coords.xy) for v in vectors]
    sel=[d<tol for d in distance]
    if sum(sel)==0:
        if min(distance)< 0.5:
            show_misses(coords,vectors,distance,context)
        return None
    # if more than one take the one closest
    d=list(compress(distance,sel))
    ixs=list(compress(range(len(sel)),sel))
    ix=ixs[d.index(min(d))]
    z=elevations[ix,2]
    return coords,z


def separate_sections(obj,vertex_sets):
    """Create a new mesh from the items in each vertex_set based on the coordinates.
    Use the coordinates since the index will likely change based on this activity.
    The new mesh will have the same name as the obj but with a suffix .001 etc attached.
    Return list of the new mesh names"""
    bpy.ops.object.mode_set(mode='OBJECT')
    known_meshes=set([o.name for o in bpy.context.scene.objects if o.type=='MESH'])
    new_meshes=[]
    for vx,vertex_set in enumerate(vertex_sets): # vx for debugging
        vertex_cos=[v[1] for v in vertex_set]
        all_cos=[v.co / SCALE for v in obj.data.vertices]
        logger.info (f"    {obj.name} has {len(obj.data.edges)} edges")
        for ex,edge in enumerate(obj.data.edges): # ex for debugging
            cos=[all_cos[v] for v in edge.vertices]

            if nearly_in(cos[0],vertex_cos) or nearly_in(cos[1],vertex_cos):
                edge.select = True
                #log.debug(f"      {cos} selected")
            else:
                edge.select = False
                #log.debug(f"      {cos} rejected")
        bpy.ops.object.mode_set(mode='EDIT') # separate only available in edit mode
        bpy.ops.mesh.separate() 
        bpy.ops.object.mode_set(mode='OBJECT')
        new_mesh_set=set([o.name for o in bpy.context.scene.objects if o.type=='MESH'])
        new_mesh_name=list(new_mesh_set-known_meshes)[0]
        known_meshes=new_mesh_set
        logger.info (f"    -- selected vertices moved to {new_mesh_name}")
        new_meshes.append(new_mesh_name)
    return new_meshes
    


def xy_to_complex(items):
    """Take the x,y values from items and return np array of complex numbers
    """
    all_coords=np.array([],dtype=complex)
    c=[complex(a.x,a.y) for a in items]
    all_coords=np.concatenate((all_coords,c))
    return all_coords

def nearly_in(coord:Vector,search_in):
    """ Drop in replacement for 'in'. Returns True if coord is in set of search_in coordinates
    Considers only x,y (not z)"""
    all_coords=xy_to_complex(search_in)
    c=complex(coord.x,coord.y)
    sel=np.isclose(all_coords,c,atol=0.01)
    return any(sel)

def end_points():
    """ For all objects returns dict of object name and coordinates of vertices with just one edge. 
    To be run before panels are raised.
    """
    bm = bmesh.new()
    end_points_map={}
    for obj in bpy.data.objects:
        verts=[]
        bm.from_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        for vert in bm.verts:
            if len(vert.link_edges)==1:
                verts.append(vert.co / SCALE)
        end_points_map[obj.name]=verts
        if len(verts)==0:
            debugpy.breakpoint()    
        pass
        bm.clear()
    bm.free()
    return end_points_map

def panel_info(obj):
    """ Returns coordinates of base (z=0) vertices from the end faces. 
    One for each end of the mesh (eg 3 for switches)
    Assumes the "raised panels" are the only faces in the obj.
    """
    coordinates=[]
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    verts={} # key is index, value is count of faces
    for face in bm.faces:
        for v in face.verts:
            if v.co.z==0:
                if v.index in verts:
                    verts[v.index]+=1
                else:
                    verts[v.index]=1
    for k,v in verts.items():
        if v==1: # keep only the ones that are used by 1 face
            vert=bm.verts[k]
            coordinates.append(vert.co / SCALE)
    logger.debug("end points?")
    for co in coordinates:
        logger.debug(co)
    return coordinates

def cluster_vertices_bmesh(obj):
    """Cluster all the vertices into several sets based on whether they are linked, working with bmesh.
    Returns a list of sets of vertex info.
    The info is index, coordinates. 
    The sets are mutually exclusive (unlike vertex groups)
    These sets then represent the sections of track in a layer (level or inclined) 
    that are isolated from other sections in the same layer by sections of the other layer.
    This is intended to run only on an object that has not yet been extruded - i.e. 2 dimensional.
    """
    me = obj.data
    matrix=obj.matrix_world
    bm = bmesh.new()
    me=obj.data
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    vert_section_map={}
    next_section=0
    # assign every vertex to a section
    for vert in bm.verts:
        if vert.index in vert_section_map.keys():
            continue
        this_section=next_section
        vert_section_map[ vert.index]=this_section
        vert_section_map=walk_connected(vert,this_section,vert_section_map)
        next_section+=1
    # covert to lists of info
    sections=[]
    for _ in range(next_section):
        sections.append([])
    for vix,section_ix in vert_section_map.items():
        co=vertex_coordinates(bm.verts[vix],matrix)
        info=(vix,co.freeze())
        sections[section_ix].append(info)
    return sections


def walk_connected(root_vert,section:int,vert_section_map:dict)->dict:
    """Recursively walk the section from root_vert, adding vertices to this section
    returns revised vert-section_map
    """
    for le in root_vert.link_edges:
        for vert in le.verts:
            if vert.index in vert_section_map:
                continue
            vert_section_map[vert.index]=section
            vert_section_map=walk_connected(vert,section,vert_section_map)
    return vert_section_map


def heights_for_obj(obj:bpy.types.Object,elevations:np.array)->dict:
    """Get the defined heights for an object
    Returns a dictionary with the (a tuple of np.float64) as a key and the z as a value
    """
    mesh=obj.data
    found={}
    for vert in mesh.vertices:
        xyz=get_z(vert,elevations=elevations,matrix_world=obj.matrix_world,context="heights for object")
        if xyz is None:
            continue
        xy,z=xyz
        found[tuple(xy)]=z
    return found

def convert_to_meshes():
    """Convert all objects to meshes
    """
    bpy.ops.object.select_all(action='DESELECT')

    for obj in bpy.data.objects:
        if obj.type == 'CURVE':
            bpy.context.view_layer.objects.active = obj # apparently needs to be active and selected
            obj.select_set(True)
            bpy.ops.object.convert(target='MESH')
            logger.info (f"{obj.name} converted to mesh")

def read_track_coordinates(filepath):
    """read the xyz coordinates of the track end points
    assume its has a header row  and the rest is x,y,z data
    """
    elevations=np.loadtxt(filepath,delimiter=',',dtype=float,skiprows=1)
    logger.info (f"Read data for {len(elevations)} end points")
    return elevations

def new_blender_file():
    """open a new file and delete the default objects"""
    # temp - next line commented to allow use of debugger
    #bpy.ops.wm.read_homefile(app_template="")

    # Cleaning the scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=True)    

def import_dxf_file(filepath):
    """Import the named dxf file from the data folder
    convert to inches"""
    # filepath parameter is being ignored, work around with files + directory
    # https://projects.blender.org/blender/blender/issues/127789
    folder,filename=os.path.split(filepath)
    bpy.ops.import_scene.dxf(files=[{"name":filename}],directory=folder,dxf_scale=format(f'{SCALE}'))

    # Use inches
    bpy.data.scenes["Scene"].unit_settings.system='IMPERIAL'
    bpy.data.scenes["Scene"].unit_settings.length_unit='INCHES'

def save_file(filepath):
    """Save the blender file under the given name"""
    #temp commented due to bug with vs code connector
    #bpy.ops.wm.save_as_mainfile(filepath= filepath)
    pass

def height_for_vertex_set(coords,elevations:np.array,matrix_world:Matrix) ->dict:
    """Determine the height(s) for a list of coordinates by
    locating the coordinates in the elevations table within tolerance defined in get_z
    Returns dict with xy as key and z as value
    Could be 1 or two items for level or inclined.
    """
    found={}
    for coord in coords:
        xyz=get_z(coord,elevations=elevations,matrix_world=matrix_world,context="in heights for vertex set")
        if xyz is None:
            continue
        xy,z=xyz
        xy=tuple(xy)
        found[xy]=z

    return found

def extrude_to_height(obj:bpy.types.Object,heights:dict,layer:str):
    """Extrude edges of obj to the largest height given along the z dimension.
    layer is either 'level' or 'inclined' 
    """
    hv=list(set(heights.values()))
    cnt=len(hv)
    rqd={"level":1,"inclined":2}[layer]
    if rqd!=cnt:
        logger.error(f"ERROR: {layer} object {obj.name} does not have exactly {rqd} height(s): {heights.values()}")
        logger.error(f"Cannot raise panels for {obj.name}")
        return
    height=max(heights.values())
    #panels_up(obj,height)
    """Extrude all edges to height along z dimension"""
    bpy.ops.object.mode_set(mode='OBJECT') 
    scaled_height=height * SCALE # got to be a better way to scale
    for edge in obj.data.edges:
        edge.select = True

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.extrude_edges_move(TRANSFORM_OT_translate={"value":(0,0,scaled_height)}) 
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.object.mode_set(mode='OBJECT')    

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

def is_face_on_top(face):
    """Return true if all vertices on a face are non zero
    all non-zero means its a face on top. 
    """
    nz=0
    for v in face.verts:
        nz+=int(v.co.z>0)
    return nz==len(face.verts)

def is_edge_on_top(edge):
    """Return true if all vertices on a edge are non zero
    all non-zero means its a edge on top.
    """
    nz=0
    for v in edge.verts:
        nz+=int(v.co.z>0)
    return nz==len(edge.verts)

def disolve_over_used_verts(bm):
    """Disolve from bmesh bm any a vertex with more than three edges non zero z-dimension
    (not currently in use... on straight switches these points are needed as part of the lines
    that create the frog.  Possibly could split the lines at the frog.)
    This is to remove anomolies caused by switches.
    """
    verts=list(bm.verts) # make a copy since we will be removing some
    for vert in verts:
        le=set(vert.link_edges)
        not_top=set()
        for e in le:
            zs=[v.co.z for v in e.verts]
            if 0 in zs:
                not_top.add(e)
        le=le - not_top                
        if len(le)>3:
            logger.info("edges attached to disolve point")
            for e in le:
                cos=[v.co / SCALE for v in e.verts]
                logger.info(cos)
            co=vert.co / SCALE
            logger.info(f"Will disolve vertex at {co} ")
            success=bmesh.utils.vert_dissolve(vert)
            logger.info(f"disolve returned {success}")
    

def bevel(obj,center_xy_coords:np.array):
    """ Bevel the top outside edges.
    Locating these edges is the hard part.
    """
    logger.info(f"Beveling {obj.name}")
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    tol=.001
    bm = bmesh.new()
    me=obj.data
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    logger.info(f"Finding edges to bevel for {obj.name}")
    logger.info(f"  There are are {len(bm.edges)} edges")
    edges_to_select=set()
    v2e_map={} # key index of vertex, values is list of up to 3 edge indices
    # disolve_over_used_verts(bm)
    # bm.to_mesh(me)
    # me.update()
    # bm.verts.ensure_lookup_table()
    # bm.edges.ensure_lookup_table()

    top_edges=[e for e in bm.edges if is_edge_on_top(e)]
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    
    # if '.002' in obj.name:
    #     for e in bm.edges:
    #         if e in top_edges:
    #             e.select=True
    #         else:
    #             e.select=False
        
    #     bm.to_mesh(me)
    #     me.update()
    #     bm.clear()

    #     bm.free()
    #     raise KeyboardInterrupt()


    for edge in top_edges:

        # eliminate crossing edges
        top_cos=[]
        for v in edge.verts:
            top_cos.append(v.co / SCALE)
        if dist(top_cos[0],top_cos[1])<tol: # bizzarely close edges seen which caused intersect None
            continue # just ignore

        # intercept method - works for level areas and the ends of the inclined ares
        cross_center=False
        for pole_bot in center_xy_coords:
            height=1+max([co.z for co in top_cos])
            pole_top =Vector((pole_bot.x,pole_bot.y,height))
            intersect=geometry.intersect_line_line(top_cos[0],top_cos[1],pole_bot,pole_top)
            if intersect is None:# should never be colinear and thus never None
                logger.error("Intersect method produced a None")
                logger.error(f"Pole: {pole_bot,pole_top}")
                logger.error(f"Top: {top_cos}")
                continue
            d=dist(intersect[0],intersect[1])
            if d<tol: 
                logger.debug(f"        Edge {edge.index} crossses center")
                cross_center=True
                break
            else:
                continue
        if cross_center:
            continue # go on to next edge
        edges_to_select.add(edge.index)
        for vert in edge.verts:
            if vert.index in v2e_map:
                v2e_map[vert.index].append(edge.index)
            else:
                v2e_map[vert.index]=[edge.index]
        logger.debug ("         Edge tentatively selected")

    # remove the crossing edges using dot product
    # this gets the ones created by the bezier curves
    ignore_vix=[]
    log_level=logging.INFO
    # if obj.name=='XTRKCAD17_curve_.001':
    #     log_level=logging.DEBUG
    logger.setLevel(log_level)

    for vix,edges in v2e_map.items():
        if vix in ignore_vix: # way to ignore other side of deleted edges
            continue
        if len(edges)!=3:
            continue
        shared_co=(bm.verts[vix].co /SCALE).freeze()
        vectors=[]
        logger.debug("Edges")
        for ix in edges: # convert the edges to vectors
            edge=bm.edges[ix]
            # remove the common point all are direction from (0,0,0)
            both_cos=[(v.co / SCALE).freeze() for v in edge.verts]            
            logger.debug(f"{ix}: {both_cos}")
            other_co=list(set(both_cos)-{shared_co})[0] # the coordinates of the non-common point
            length=dist(shared_co,other_co) # used to scale to get unit vectors
            vector=other_co-shared_co # center at (0,0,0)
            vector=vector /length # unit vector needed so dot product works right            
            vectors.append(vector)
        logger.debug("Vectors")
        for v in vectors:
            logger.debug(v)
        pairs=(0,1),(0,2),(1,2) # the three possible pairings
        dp=[]
        for pair in pairs:
            dp.append(vectors[pair[0]] @ vectors[pair[1]]) # @ is python for dot product 
        logger.debug ("Dot products by pair")

        # make a histogram to count the times each edge is a near perpendicular
        # the one with 2 will be the crossing edge
        hist={0:0,1:0,2:0}
        for pr,d in zip(pairs,dp):
            logger.debug (f"{pr},{d}")
            if abs(d)<.14: # tolerance determined empirically. 
                for edge_set_ix in pr:
                    hist[edge_set_ix]+=1
        logger.debug(hist)
        for edge_set_ix,cnt in hist.items():
            if cnt==2:
                to_remove_ix=edges[edge_set_ix]
                edges_to_select.remove(to_remove_ix)
                # other side of crossing can be removed to prevent doing it again on the other side
                edge=bm.edges[to_remove_ix]
                for v in edge.verts: # might as well add both sides to the ignore list
                    ignore_vix.append(v.index)
                logger.debug(f"Edge set item {edge_set_ix} with index {to_remove_ix} removed.")

    to_bevel=[bm.edges[ix]for ix in edges_to_select]
    bmesh.ops.bevel(bm, geom=to_bevel,offset=0.1875 * SCALE,affect='EDGES')
    bm.to_mesh(me)
    me.update()
    bm.clear()

    bm.free()

def create_bezier(control_points: list):
    """Set up a bezier curve
    control_points an ordered list of 4 x,y,z triplets in inches
    first and last are "anchor" points on the top of the level panels
    the middle two are the points where the incline stops (low,high)
    returns new curve name
    modified from: https://blender.stackexchange.com/questions/296531/script-to-import-coordinates-and-create-a-bezier-curve
    """
    
    scale=.0254 # convert to meters
    assert len(control_points)==4,'Should have 4 control points'
    cpv=[scale * Vector(cp) for cp in control_points]

    # create bezier curve and add enough control points to it
    bpy.ops.curve.primitive_bezier_curve_add(enter_editmode=True)
    curve = bpy.context.active_object
    curve_name=curve.name

    bez_points = curve.data.splines[0].bezier_points

    # note: a created bezier curve has already 2 control points
    bez_points.add(len(control_points) - 2)

    # set the locations of the points
    for i,co in enumerate(cpv):        
        bez_points[i].co = co

    # setting of the handle types is done via ops calls so that
    # the curves are redrawn.
    bpy.ops.curve.select_all(action='SELECT')
    bpy.ops.curve.handle_type_set(type='VECTOR')
    # set middle two to aligned here
    bpy.ops.curve.select_all(action='DESELECT')
    for i, bpt in enumerate(bez_points):
        if i==1:
            bpt.select_right_handle=True
            bpy.ops.curve.handle_type_set(type='ALIGNED')
            bpt.select_control_point=False
        if i==2:
            bpt.select_left_handle=True
            bpy.ops.curve.handle_type_set(type='ALIGNED')
            bpt.select_control_point=False

    bpy.ops.object.mode_set(mode='OBJECT')
    return curve_name

def solidify_roadbed():
    """Solidfy the roadbed sections.  Expect panels are up. """
    bed_sections=[]
    logger.info ("Roadbed sections will be: ")    
    for obj in bpy.data.objects:
        if obj.name.startswith("XTRKCAD"):
            bed_sections.append(obj.name)
            logger.info (f"{obj.name}")            
            obj.select_set(True)
            logger.info(f"Solidifying: {obj.name}")   
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_add(type='SOLIDIFY')
            bpy.context.object.modifiers["Solidify"].solidify_mode="NON_MANIFOLD" # i.e. complex
            bpy.context.object.modifiers["Solidify"].nonmanifold_thickness_mode="FIXED"
            bpy.context.object.modifiers["Solidify"].thickness = .9375* SCALE
            bpy.context.object.modifiers["Solidify"].offset = 0
            bpy.ops.object.modifier_apply(modifier="Solidify")

def solidify_trimmers(trimmer_map):
    """Turn the bezier curves into a 3d block than can be used to trim the
    roadbed along a grade.
    trimmer_map is dict of key: mesh_name, value: corresponding bezier curve name
    """
    logger.info ("Trimmers will be: ")
    for mesh_name,bezier_name in trimmer_map.items():    
        for obj in bpy.data.objects:
            if obj.name == bezier_name:
                logger.info (f"{obj.name} for {mesh_name}")            
                bpy.ops.object.select_all(action='DESELECT')
                obj.select_set(True)
                bpy.context.view_layer.objects.active = obj
                logger.info(f"Selected and active: {obj.name}")   
                bpy.ops.object.mode_set(mode='EDIT')
                bpy.ops.mesh.select_all(action='SELECT')
                bpy.ops.mesh.extrude_edges_move(TRANSFORM_OT_translate={"value":(0,0,.0254)}) # assume 1" is enough
                bpy.ops.object.mode_set(mode='OBJECT')
                bpy.ops.object.modifier_add(type='SOLIDIFY')
                bpy.context.object.modifiers["Solidify"].solidify_mode="NON_MANIFOLD" # i.e. complex
                bpy.context.object.modifiers["Solidify"].thickness = 3.5*.0254 # estimate wide enough to handle big curve
                bpy.context.object.modifiers["Solidify"].offset = 0
                bpy.ops.object.modifier_apply(modifier="Solidify")
    

def merge_near_vertices():
    """ Merge vertices in the object when they are close
    The dxf import leaves gaps in some cases.  
    This should eliminate the gaps by merging near points
    https://blender.stackexchange.com/questions/68093/remove-doubles-on-multiple-objects
    """
    threshold=0.00486# in meters (about 3/16 inch)
    
    objs = set(o for o in bpy.context.scene.objects if o.type == 'MESH')
    bm = bmesh.new()

    for obj in objs:
        logger.info (f"  Merging near vertices for {obj.name}")
        me=obj.data
        bm.from_mesh(me)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=threshold)
        bm.to_mesh(me)
        me.update()
        bm.clear()

    bm.free()


def meshes_of_type(mesh_type):
    """Given a type of level or inclined, return list of mesh names"""
    return [o.name for o in bpy.context.scene.objects if o.name.startswith(LAYER_TYPES_TO_XTC[mesh_type])]

def delete_objects_by_name(object_names):
    """Delete the objects in the list
    """
    bpy.ops.object.select_all(action='DESELECT')
    for obj in bpy.data.objects:
        if obj.name in object_names:
            obj.select_set(True)
    bpy.ops.object.delete(use_global=True)              
        
def split_xtc_layers():
    """Split the xtc provided layers into mesh objects
    return dict of layer names : [object names]
    """
    layer_obj_map={}
    original_objects=list(bpy.context.scene.objects)
    for obj in original_objects:
        if obj.type == 'MESH':
            logger.info(f"Splitting layer: {obj.name}")
            layer=XTC_LAYER_TYPES[obj.name]
            clusters=cluster_vertices_bmesh(obj)
            layer_obj_map[layer] = separate_sections(obj,clusters)
    return layer_obj_map

def panels_up_for_layer(layer,layer_mesh_map,elevations,end_point_map):  
    """Raise the panels for all meshes associated with a layer
    returns a dict of object names: panel info
    """
    bevel_info={} 
    for mesh_name in layer_mesh_map[layer]:
        logger.info (f"  Putting up panels for {mesh_name}")
        obj=bpy.data.objects[mesh_name]
        coords=end_point_map[mesh_name]
        heights=height_for_vertex_set(coords,elevations,obj.matrix_world)
        extrude_to_height(obj,heights,layer)
        info=panel_info(obj)
        bevel_info[obj.name]=info # do bevels after all sections are up, solid & trimmed
    return bevel_info

def show_misses(xyz,candidates,away,context):
    """display up to 5 misses sorted by distance when looking for a coordinate
    xyz is the single point, candidates are all the items that were checked,
    away is how far away they were from xyz
    context is added to the message"""
    logger.error(f"Looking for {xyz} {context}")
    logger.error(f"The best candidates out of {len(candidates)} are: ")
    z=tuple(zip(candidates,away))
    z=sorted(z,key=lambda x: x[1])
    for i,(c,d) in enumerate(z):
        if i>5:
            break
        logger.error (f"Candidate {c}   Distance: {d}")

def create_bezier_controls(layer_mesh_map,elevations,end_point_map):
    """For the inclined layer create a bezier curve.  
    For that we need heights from the adjoining level sections.
    there should a point at the same x,y in one of the level sections
    """
    bezier_controls={} # key is the mesh name, each value is an ordered list of points
    tol=.001
    for inclined_name in layer_mesh_map["inclined"]:
        logger.info (f"Creating bezier control points for {inclined_name}")
        obj=bpy.data.objects[inclined_name]
        coords=end_point_map[inclined_name]
        heights=height_for_vertex_set(coords,elevations,obj.matrix_world)
        bz_points=[]
        candidates=[] # for error reporting only
        for xy,height in heights.items():
            xyz=(xy[0],xy[1],height)
            found=False
            logger.debug(f"    looking for {xyz} in level meshes")
            for level_name in meshes_of_type("level"):
                mesh=bpy.data.objects[level_name]
                logger.debug(f"      {len(mesh.data.edges)} edges in {level_name}")
                bz=[]
                for edge in mesh.data.edges:
                    vcs=[]
                    near=[]
                    away=[]
                    for ev in edge.vertices:
                        vc=tuple(vertex_coordinates(mesh.data.vertices[ev],mesh.matrix_world ))
                        vcs.append(vc)
                        away.append(dist(xyz,vc))
                        near.append(away[-1]<tol)
                    if vcs[0][2]!=vcs[1][2]: # not on same level
                        continue
                    if vcs[0][2]==0: # only consider the ones at non zero height
                        continue
                    candidates+=vcs
                    if any(near): # if either edge vertex is near enough then both are part of bezier 
                        # the following puts the incline layer points in the center
                        # and the level points on the outside
                        bz=sorted(vcs,key=lambda v:dist(v,xyz)<tol,reverse=bool(len(bz_points)))
                        bz_points+=bz
                        for bp in bz:
                            logger.debug (f"        {bp}")
                        break # done, no more edges needed for this side of the bezier curve
                logger.debug(f"      {len(bz_points)} Bezier points")
                if len(bz)==2: # this side of the curve is satisfied by the most recent mesh
                    found=True
                    break
            if not found:
                show_misses(xyz,candidates,away,"in level meshes")
        if len(bz_points)!=4: # all meshes used up and we still don't have a match
            logger.error(f"Could not create bezier control.  Need 4 points but have {len(bz_points)}")

                
        else:
            bezier_controls[inclined_name]=(bz_points)
    return bezier_controls

class IMPORT_xtc(bpy.types.Operator):
    """Xtrackcad to 3D (.dxf, .csv)"""
    bl_idname = "import_xtrackcad.xtc"
    bl_description = 'Import XTrackCAD from(.dxf, .csv)'
    bl_label = "XTrackCAD to 3D v." + __version__
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_options = {'UNDO'}

    filepath_1: bpy.props.StringProperty(name="input dxf file",
                               subtype='FILE_PATH',
                               default="elevations.csv") # type: ignore
    filename_ext_1 = ".dxf"
    filepath_2: bpy.props.StringProperty(name="input dxf file",
                               subtype='FILE_PATH',
                               default="revised.dxf") # type: ignore
    filename_ext_2 = ".csv"

    def execute(self, context):
        # parameters
        DATA_FOLDER="/Users/george/argus/blender_scripts/addons/dxf/data/"
        
        elevations=read_track_coordinates(DATA_FOLDER+'elevations.csv')

        new_blender_file()
        import_dxf_file(DATA_FOLDER+"revised.dxf")

        convert_to_meshes()

        merge_near_vertices()

        layer_mesh_map=split_xtc_layers()

        end_point_map=end_points()

        bevel_info=panels_up_for_layer("level",layer_mesh_map,elevations,end_point_map)
        bi=panels_up_for_layer("inclined",layer_mesh_map,elevations,end_point_map)
        bevel_info.update(bi)

        bezier_controls=create_bezier_controls(layer_mesh_map,elevations,end_point_map)

        trimmer_map={}
        for mesh_name,bz_points in bezier_controls.items():
            trimmer_map[mesh_name]=create_bezier(bz_points)
        
        convert_to_meshes() # the beziers this time

        solidify_roadbed()
       
        solidify_trimmers(trimmer_map)

        # do the trim and discard the trim tools
        for mesh_name,trimmer in trimmer_map.items():
            obj=bpy.data.objects[mesh_name]
            logger.info(f"Trimming {obj.name} using trimmer: {trimmer}")
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_add(type='BOOLEAN')
            mname=obj.modifiers[-1].name
            bpy.context.object.modifiers[mname].object=bpy.data.objects[trimmer] 
            bpy.context.object.modifiers[mname].operation = "DIFFERENCE"
            bpy.context.object.modifiers[mname].use_self=True
            bpy.ops.object.modifier_apply(modifier=mname)
            delete_objects_by_name([trimmer])             


    
        # do the bevels
        for name,info in bevel_info.items():
            logger.info(f"Beveling {name}")
            obj=bpy.data.objects[name]
            bevel(obj,info)
              

        pass

        save_file(f"{DATA_FOLDER}script_generated.blend")
        return {'FINISHED'}

def menu_func(self, context):
    self.layout.operator(IMPORT_xtc.bl_idname, text="XTrackCAD (.dxf, .csv)")

# store keymaps here to access after registration

def register():
    bpy.utils.register_class(IMPORT_xtc)
    bpy.types.TOPBAR_MT_file_import.append(menu_func)

def unregister():
    bpy.utils.unregister_class(IMPORT_xtc)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func)

if __name__ in ("__main__"):
    register()

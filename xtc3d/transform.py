"""Collection of functions that do the processing to convert from dxf to blender
"""
from itertools import combinations,compress 
from logging import DEBUG,INFO
from math import acos, degrees,dist

import bmesh
import bpy
from mathutils import Vector

from numpy import array
from shapely import LineString,Point, dwithin

from xtc3d import logger, BEVEL,LAYER_TYPES_TO_XTC,ROADBED_WIDTH,SCALE,TOL_COLINEAR,TOL_PERPENDICULAR,XTC_LAYER_TYPES
from xtc3d.geometry import find_crossing_edges,fmt_coords, get_z,height_for_vertex_set,nearly_in,show_misses, vertex_coordinates

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
            pass
        bm.clear()
    bm.free()
    logger.info("End points")
    for name,verts in end_points_map.items():
        logger.info(f"{name}:")
        for v in verts:
            logger.info(f"    {v}")
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
        vert_section_map=walk_connected_by_edge(vert,this_section,vert_section_map)
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

def walk_connected_by_edge(root_vert,section:int,vert_section_map:dict)->dict:
    """Recursively walk the section from root_vert, adding vertices to this section
    returns revised vert-section_map
    """
    for le in root_vert.link_edges:
        for vert in le.verts:
            if vert.index in vert_section_map:
                continue
            vert_section_map[vert.index]=section
            vert_section_map=walk_connected_by_edge(vert,section,vert_section_map)
    return vert_section_map

def walk_boundary_edges(bm,root_edge,chain:int,edge_chain_map:dict,frog_points,other_root_edge_indexes,logging_level=INFO)->dict:
    """In the bmesh bm, recursively walk the boundary from root_edge, adding edges to this chain
    frog_points are a list of shapely points used to stop the walk.
    other_root_edge_indexes are also used to stop the walk.
    returns revised edge_chain_map
    """
    if root_edge.index in other_root_edge_indexes:
        logger.debug(f"    Stopping at edge {root_edge.index} which is an track end")
        return edge_chain_map
    
    edge_chain_map[root_edge.index]=chain

    both_cos=[(v.co / SCALE).freeze() for v in root_edge.verts] 
    length=dist(both_cos[0],both_cos[1])
    cos=fmt_coords(both_cos)
    logger.debug (f"    Chain {chain}: Added {root_edge.index}: {cos} length {length:.4f} ")

    for dix,vert in enumerate(root_edge.verts): # go both directions
        (a,b)=vert.co.xy/SCALE
        pt=Point(a,b)
        logger.debug(f"  direction {dix} from {pt.x:.3f}, {pt.y:.3f}")
        #determine if we have hit a frog point
        mtch=False
        for fp in frog_points:
            if pt.equals_exact(fp,.001):
                mtch=True
        if mtch:
            logger.debug(f"    Stopping at frog point {pt}")
            continue

        (perp,colin,_)=typed_edges_for_vert_relative(bm,vert.index,root_edge.index,logging_level=logging_level)
        #logger.setLevel(DEBUG)# TODO remove this line
        next_edge_ix=None
        if len(colin)==2:
            ne=set(colin) - set([root_edge.index])
            next_edge_ix=ne.pop()
        else:
            if len(perp)==1:
                logger.debug(f"    Turning corner at {pt}")
                next_edge_ix=perp[0]

            if len(perp)>1: # "Unexpected multiple perpendiculars"
                pass


        if next_edge_ix is not None:
            if next_edge_ix not in edge_chain_map.keys():

                next_edge=bm.edges[next_edge_ix]
                edge_chain_map=walk_boundary_edges(bm,next_edge,chain,edge_chain_map,frog_points,other_root_edge_indexes,logging_level)
    return edge_chain_map

def heights_for_obj(obj,elevations:array)->dict:
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


def extrude_to_height(obj,heights:dict,layer:str):
    """Extrude edges of obj to the largest height given along the z dimension.
    layer is either 'level' or 'inclined' 
    """
    hv=list(set(heights.values()))
    cnt=len(hv)
    rqd={"level":1,"inclined":2}[layer]
    if rqd!=cnt:
        logger.error(f"ERROR: {layer} object {obj.name} does not have exactly {rqd} height(s): {heights.values()}")
        logger.error(f"Cannot raise panels for {obj.name}")
        # import debugpy
        # debugpy.breakpoint()
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

def links_to_edge(e):
    le=set() # indexes of all the linked edges for either vertex
    for v in e.verts:
        for vle in v.link_edges:
            le.add(vle.index)
    return le

def typed_edges_for_vert_relative(bm,vix:int,reix:int,logging_level=INFO):
    """For a vertex on the top, return lists of top edges classified relative to the given edge
    bm is a bmesh that includes the vertex whose index is vix and the edge index reix
    returns 3 lists of edge indexes: perpish, colinearish, neither
    """
    logger.setLevel(logging_level)
    relative_edge=bm.edges[reix]
    assert vix in [v.index for v in relative_edge.verts],'Oops vertex is not in the relative edge'
    colinear_ish=[]
    perp_ish=[]
    neither=[]
    edges=bm.verts[vix].link_edges
    shared_co=(bm.verts[vix].co /SCALE).freeze()
    vectors=[]
    edge_indexes=[]
    relative_flag=[]
    logger.debug("Edges")
    for edge in edges: # convert the edges to vectors
        # remove the common point all are direction from (0,0,0)
        both_cos=[(v.co / SCALE).freeze() for v in edge.verts] 
        if 0 in [c.z for c in both_cos]:
            logger.debug("skipping 0")
            continue                   
        edge_indexes.append(edge.index)
        length=dist(both_cos[0],both_cos[1])# used to scale to get unit vectors
        logger.debug(f"{edge.index}: {both_cos} length {length}")
        other_co=list(set(both_cos)-{shared_co})[0] # the coordinates of the non-common point
        vector=other_co-shared_co # center at (0,0,0)
        vector=vector /length # unit vector needed so dot product works right            
        vectors.append(vector)
        relative_flag.append(reix==edge.index)
    logger.debug("Vectors with index and (edge index) and relative flag")
    n=len(vectors)
    rvx=None # index in vectors of the relative edge
    for ix,(eix,v,rf) in enumerate(zip(edge_indexes,vectors,relative_flag)):
        logger.debug(f"{ix}. ({eix}) {v} {rf}")
        if rf:
            rvx=ix 
    assert rvx is not None,"Oops no relative vector index"
    pairs=tuple(zip(([rvx]*n),range(n))) # compare relative edge to each edge (including self)
    dps=[]
    for pair in pairs:
        dps.append(vectors[pair[0]] @ vectors[pair[1]]) # @ is python for dot product 
    
    # hack to eliminate problem seen on incline where there is an extra near colinear edge with
    s=sum([dp > .99999 for dp in dps])
    if s>1:
        sel=[(dp > .99999)and (dp<.999999) for dp in dps]
        if any(sel):
            logger.debug(f"Removed very near pair {pairs[sel.index(True)]}")
            sel=[not s for s in sel]
            pairs=list(compress(pairs,sel))
            dps=list(compress(dps,sel))


    logger.debug ("Dot products by pairs of vector indexes")
    for pr,dp in zip(pairs,dps):
        dp=round(dp,6) # to round off excess over/under 1/-1 which causes acos domain error
        dg=degrees(acos(dp))
        logger.debug (f"{pr},{dp} or {dg} degrees")
        dp=abs(dp)
        vector_ix=pr[1]
        edge_ix=edge_indexes[vector_ix]
        if dp < TOL_PERPENDICULAR: 
            perp_ish.append(edge_ix)
            logger.debug(f"Edge set item {vector_ix} with index {edge_ix} is near perpendicular.")
        else: # is it colinear or neither?
            if dp > TOL_COLINEAR:
                colinear_ish.append(edge_ix)
                logger.debug(f"Edge set item {vector_ix} with index {edge_ix} is near colinear.")
            else:
                neither.append(edge_ix)
                logger.debug(f"Edge set item {vector_ix} with index {edge_ix} is near neither perpendicular nor colinear.")

    return perp_ish, colinear_ish, neither

def bot_edges_from_face(face):
    """Returns the edges of a face that have z=0 on both vertices"""
    bot_edges=set()
    for edge in face.edges:
        nz=[0==v.co.z for v in edge.verts]
        if all(nz):
            bot_edges.add(edge)
    return bot_edges

def top_edges_from_face(face):
    """Gets the top edges from a vertical face
    """
    top_edges=set()
    for edge in face.edges:
        nz=[0<v.co.z for v in edge.verts]
        if all(nz):
            top_edges.add(edge)
    return top_edges

def top_edges_from_bm(bm):
    """Return a subset of bm.edges that do not have z=0 for either vertex"""
    top_edges=set()
    for edge in bm.edges:
        nz=[0<v.co.z for v in edge.verts]
        if all(nz):
            top_edges.add(edge)
    return top_edges

def bevel_edge_sets(tee:dict,frog_points:dict):
    """Get the bevelable edge sets for the roadbed objects.
    tee is dict with object name as key with values a list of a top end edges for every end of that object
    frog_points is a dict with object name as key and a list of shapely points where the frog vertices were created
    Returns a dict with the object name as key and a set of edge indices specific to that object.
    """
    #logger.setLevel(DEBUG)
    bm = bmesh.new()
    result={}
    for mesh_name,root_edges_indexes in tee.items():
        logger.info(f"Building bevel edge set for {mesh_name}")
        logger.info(f"  Based on {len(root_edges_indexes)} root edges:")
        dbg_level=INFO
        if mesh_name=='XTRKCAD3_curve_.003':
            dbg_level=DEBUG
        for rei in root_edges_indexes:
            logger.debug(f"    {rei}")  
        frog_pts=[] # they only exist for level meshes
        if mesh_name in frog_points:  
            frog_pts=frog_points[mesh_name]
        obj=bpy.data.objects[mesh_name]
        me = obj.data
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        chain_map={} # key is vertex index, value is chain index
        for cix,rei in enumerate(root_edges_indexes):
            co=fmt_coords([v.co/SCALE for v in bm.edges[rei].verts])
            logger.debug(f"-- Chain {cix} -- starts at {co}")
            others=root_edges_indexes.copy()
            others.remove(rei)
            chain_map=walk_boundary_edges(bm,bm.edges[rei],cix,chain_map,frog_pts,others,logging_level=dbg_level)

        all_boundary_edges=set(chain_map.keys())
        result[mesh_name]=all_boundary_edges - (set(root_edges_indexes))
        bm.clear() # ready for next object
    return result

def top_end_edges(obj_base_end_points:dict):
    """Get the edges at the top of the ends roadbed.
    obj_base_end_points is dict with object name as key and
    the endpoints of the center lines of that object as a list as the values
    Returns a dict with the object name as key and a set of edge indices specific to that object.
    """
    logger.setLevel(INFO)# TODO remove this line
    bm = bmesh.new()
    result={}
    for mesh_name,base_end_points in obj_base_end_points.items():
        logger.info(f"Looking for top edges at roadbed ends for {mesh_name}")
        logger.info(f"  Based on {len(base_end_points)} points:")
        for co in base_end_points:
            logger.debug(f"    {co}")    

        obj=bpy.data.objects[mesh_name]
        me = obj.data
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        tee=set()
        tol=.001
        points=[Point(co.xy) for co in base_end_points]
        points=set(points)
        for face in bm.faces:
            bot_edges=bot_edges_from_face(face)
            if len(bot_edges)==1:
                bot_edge=bot_edges.pop()
                (a,b)=[v.co.xy / SCALE for v in bot_edge.verts]
                line1=LineString([a,b])
                for co in list(points):
                    if not dwithin(line1,co,tol):
                        continue
                    logger.debug(f"Point {co} found within {tol} of {line1}")
                    points.remove(co)
                    edges=top_edges_from_face(face)
                    for edge in edges: # one for level, two for inclined
                        (a,b)=[v.co.xy/SCALE for v in edge.verts]
                        line2=LineString([a,b])
                        if dwithin(line2,co,tol):
                            logger.debug(f"  Edge {edge.index}: {(a,b)}")
                            tee.add(edge.index)

        # some of the inclined items have only one of the two root edges, so add the other one
        to_add=set()
        for tei in tee:
            edge=bm.edges[tei]
            for vert in edge.verts:
                _,colin,_=typed_edges_for_vert_relative(bm,vert.index,tei)
                colin.remove(tei)
                for ex in colin:
                    to_add.add(ex)
        for ex in to_add:
            tee.add(ex)
            logger.debug(f"Added edge {ex} to tee")


        result[mesh_name]=tee
        logger.info(f"  Found {len(tee)}. {len(points)} points were not consumed")
        if len(points):
            for co in points:
                logger.debug(f"  {co}")
        bm.clear() # clear out this objects bm, ready to load next one
    return result

def fix_frogs(layer_mesh_map):
    """For objects in the "level" layer, ensure frog junction the lines that pass through frog point have a vertex there.
    As it comes, the diverging line end point is near the edge of the other side, 
    but there is no vertex, so the boundary edges actually go into the solid.
    Returns dict: key=mesh_name, list of 2d shapely Points which were created.
    Call after the solidify step.
    """
    bm = bmesh.new()
    result={}
    for mesh_name in layer_mesh_map["level"]:

        logger.info(f"Finding frogs for {mesh_name}")
        obj=bpy.data.objects[mesh_name]
        me = obj.data
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        top_edges=top_edges_from_bm(bm)
        crossings=find_crossing_edges(bm,top_edges)
        net_top_edges=top_edges - crossings

        logger.debug(f"{len(top_edges)} top edges in {obj.name}")
        logger.debug(f"{len(crossings)} crossings in {obj.name}")
        logger.debug(f"{len(net_top_edges)} net top edges (after removing crossings)")

        created_points=[]
        # check each combination to see if it has an intersection
        for (e,f) in combinations(net_top_edges,2):
            # but disallow if already share a common vertex
            le=links_to_edge(e)
            lf=links_to_edge(f)

            if f.index in le:
                continue
            # pairs of vectors that define each line segment             
            (a,b)=[v.co.xy / SCALE for v in e.verts]
            (c,d)=[v.co.xy / SCALE for v in f.verts]
            line1=LineString([a,b])
            line2=LineString([c,d])
            if line1.intersects(line2):
                point=line1.intersection(line2)
                logger.debug(f"Edges {e.index}: ({a.x:.6f}, {a.y:.6f}) - ({b.x:.6f}, {b.y:.6f}), links: {le}")
                logger.debug(f"  AND {f.index}: ({c.x:.6f}, {c.y:.6f}) - ({d.x:.6f}, {d.y:.6f}), links: {lf}")
                logger.debug(f"   AT {point}")

                e_vert=e.verts[0]
                f_vert=f.verts[0]
                e_fac=dist(e_vert.co.xy/SCALE,(point.x,point.y))/dist(a,b)
                f_fac=dist(f_vert.co.xy/SCALE,(point.x,point.y))/dist(c,d)
                bmesh.utils.edge_split(e, e_vert, e_fac)
                bmesh.utils.edge_split(f, f_vert, f_fac)
                created_points.append(point)
                logger.info(f"Split two lines at {point}")
        bm.to_mesh(me)
        me.update()
        bm.clear()
        result[mesh_name]=created_points
    
    bm.free()
    return result

def bevel(obj,edge_index_set):
    """ Bevel the top outside edges
    The edge set is derived by the caller.  This just does the bevel.
    """
    logger.info(f"Beveling {obj.name}")
    bpy.ops
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

    me = obj.data
    bm=bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    edges_to_select=list(edge_index_set)
    to_bevel=[bm.edges[ix]for ix in edges_to_select]
    bmesh.ops.bevel(bm, geom=to_bevel,offset=BEVEL * SCALE,affect='EDGES')
    logger.info(f"  Bevel operation run with {len(edge_index_set)} edges")
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
    """Solidfy the roadbed sections.  Expect panels are up. 
    """
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
            bpy.context.object.modifiers["Solidify"].thickness = ROADBED_WIDTH* SCALE
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
                bpy.context.object.modifiers["Solidify"].thickness = 70 *.0254 # estimate wide enough to handle big curve
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

def create_bezier_controls(layer_mesh_map,elevations,end_point_map):
    """For the inclined layer create a bezier curve.  
    For that we need heights from the adjoining level sections.
    there should a point at the same x,y in one of the level sections
    """
    bezier_controls={} # key is the mesh name, each value is an ordered list of points
    tol=.095
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

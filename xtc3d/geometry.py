"""Collection of geometry functions
"""
from itertools import compress
from math import dist

from mathutils import Matrix, Vector
from numpy import array, concatenate, isclose
from xtc3d import logger,SCALE


def vertex_coordinates(vert,matrix_world:Matrix)->tuple:
    """Return the x,y coordinates of a vertex as a tuple of floats
    """
    scale = round(1/SCALE,6) # inches per meter
    return scale * matrix_world @ vert.co    

def get_z(coords,elevations:array,matrix_world:Matrix,context="")->float:
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

def xy_to_complex(items):
    """Take the x,y values from items and return np array of complex numbers
    """
    all_coords=array([],dtype=complex)
    c=[complex(a.x,a.y) for a in items]
    all_coords=concatenate((all_coords,c))
    return all_coords

def nearly_in(coord:Vector,search_in):
    """ Drop in replacement for 'in'. Returns True if coord is in set of search_in coordinates
    Considers only x,y (not z)"""
    all_coords=xy_to_complex(search_in)
    c=complex(coord.x,coord.y)
    sel=isclose(all_coords,c,atol=0.01)
    return any(sel)

def fmt_coords(both_cos):
    "easier to read format for coordinates"
    co_fmt=[]
    for co in both_cos:
        a=", ".join([f"{n:.4f}" for n in co])
        co_fmt.append(f"({a})")
    cos=' '.join(co_fmt)
    return cos

def height_for_vertex_set(coords,elevations:array,matrix_world:Matrix) ->dict:
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

def find_crossing_edges(bm,top_edges):
    """locate the crossing edges using dot product.
    bm is the bmesh for the object.  top_edges is a subset of bm.edges that are on the top.
    Returns the set of the edges that cross side to side.
    To do this it finds the verticies that have 3 edges and works out the angles between pairs.
    The crossing ones will be nearly perpendicular to the other two.
    """
    # make a dict with key index of vertex, values is list of edge indices
    v2e_map={} 
    for edge in top_edges:
        for vert in edge.verts:
            if vert.index in v2e_map:
                v2e_map[vert.index].append(edge.index)
            else:
                v2e_map[vert.index]=[edge.index]

    ignore_vix=[]
    crossing_edges=set()
    for vix,edge_ixs in v2e_map.items():
        if vix in ignore_vix: # way to ignore other side of deleted edges
            continue
        if len(edge_ixs)!=3:
            continue
        shared_co=(bm.verts[vix].co /SCALE).freeze()
        vectors=[]
        logger.log(5,"Edges")
        for ix in edge_ixs: # convert the edges to vectors
            edge=bm.edges[ix]
            # remove the common point all are direction from (0,0,0)
            both_cos=[(v.co / SCALE).freeze() for v in edge.verts]            
            logger.log(5,f"{ix}: {both_cos}")
            other_co=list(set(both_cos)-{shared_co})[0] # the coordinates of the non-common point
            length=dist(shared_co,other_co) # used to scale to get unit vectors
            vector=other_co-shared_co # center at (0,0,0)
            vector=vector /length # unit vector needed so dot product works right            
            vectors.append(vector)
        logger.debug("Vectors")
        for v in vectors:
            logger.log(5,v)
        pairs=(0,1),(0,2),(1,2) # the three possible pairings
        dp=[]
        for pair in pairs:
            dp.append(vectors[pair[0]] @ vectors[pair[1]]) # @ is python for dot product 
        logger.log (5,"Dot products by pair")

        # make a histogram to count the times each edge is a near perpendicular
        # the one with 2 will be the crossing edge
        hist={0:0,1:0,2:0}
        for pr,d in zip(pairs,dp):
            logger.log (5,f"{pr},{d}")
            if abs(d)<.14: # tolerance determined empirically. I think it has to do with how tight the curve is.
                for edge_set_ix in pr:
                    hist[edge_set_ix]+=1
        logger.log(5,hist)
        for edge_set_ix,cnt in hist.items():
            if cnt==2:
                crossing_edge_ix=edge_ixs[edge_set_ix]
                crossing_edges.add(bm.edges[crossing_edge_ix])
                logger.debug(f"Edge set item {edge_set_ix} with index {crossing_edge_ix} found.")

                # vertex at other side of crossing can be ignored to prevent doing it again on the other side
                edge=bm.edges[crossing_edge_ix]
                for v in edge.verts: # might as well add both sides to the ignore list
                    ignore_vix.append(v.index)

    return crossing_edges

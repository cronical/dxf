"""Collection of functions no longer needed. Kept for reference.
"""
from itertools import combinations
from logging import DEBUG
from math import dist,acos, degrees
import bmesh
import bpy
from xtc3d import logger, SCALE,TOL_PERPENDICULAR
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

def test_typed_edges_for_vert():
    obj=bpy.data.objects["XTRKCAD3_curve_.001"]
    bm=bmesh.from_edit_mesh(obj.data)
    vix=None
    for v in bm.verts:
        vix=v.index
        if v.select:
            break
    if vix:
        a,b,c= typed_edges_for_vert(bm,vix)
    print(a)
    print(b)
    print(c)

def typed_edges_for_vert(bm,vix:int):
    """For a vertex on the top, return lists of top edges: perpish, colinearish, neither
    bm is a bmesh that includes the vertex whose index is vix.
    requires SCALE, logging and logger
    """
    logger.setLevel(DEBUG)

    colinear_ish=[]
    perp_ish=[]
    neither=[]
    edges=bm.verts[vix].link_edges
    shared_co=(bm.verts[vix].co /SCALE).freeze()
    vectors=[]
    edge_indexes=[]
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
    logger.debug("Vectors with index and (edge index)")
    n=len(vectors)
    for ix,(eix,v) in enumerate(zip(edge_indexes,vectors)):
        logger.debug(f"{ix}. ({eix}) {v}")
    pairs=tuple(combinations(range(n),2)) # the possible pairings
    dps=[]
    for pair in pairs:
        dps.append(vectors[pair[0]] @ vectors[pair[1]]) # @ is python for dot product 
    
    logger.debug ("Dot products by pairs of vector indexes")

    # make a histogram to count the times each edge is a near perpendicular 
    # the ones (normally one) with 2 will be the common perpendicular to those edges
    counts=[0]*n
    hist=dict(zip(range(n),counts)) 
    for pr,dp in zip(pairs,dps):
        dp=round(dp,6) # to round off excess over/under 1/-1 which causes acos domain error
        dg=degrees(acos(dp))
        logger.debug (f"{pr},{dp} or {dg} degrees")
        dp=abs(dp)
        if dp < TOL_PERPENDICULAR: 
            for vector_ix in pr:
                hist[vector_ix]+=1
    logger.debug("Histogram")            
    logger.debug(hist)
    for vector_ix, cnt in hist.items():
        edge_ix=edge_indexes[vector_ix]
        if cnt==2:
            perp_ish.append(edge_ix)
            logger.debug(f"Edge set item {vector_ix} with index {edge_ix} is near perpendicular.")
    logger.debug(perp_ish)
    match len(perp_ish):
        case 0: # there are no edges with 2 perpendiculars.  This could be a corner
            for vector_ix in range(n):
                eix=edge_indexes[vector_ix]
                colinear_ish.append(eix)
                logger.debug(f"Edge set item {vector_ix} with index {eix} is near colinear.")
            
        case 1: # typically an edge, but it can be a cross piece with a perpendicular rail
            # examine the pairs that the perpendicuar participates in
            for pr,dp in zip(pairs,dps):
                eix= set([edge_indexes[vi]for vi in pr])
                #logger.debug(f"{eix}, {dp}")
                if perp_ish[0] not in eix:
                    continue
                eix.remove(perp_ish[0]) # the other index
                eix=eix.pop()
                #logger.debug(f"{dp} < {tol_perpendicular}")
                if dp < TOL_PERPENDICULAR:
                    colinear_ish.append(eix)
                    logger.debug(f"Edge set item {vector_ix} with index {eix} is near colinear.")
                else:
                    neither.append(eix)
                    logger.debug(f"Edge set item {vector_ix} with index {eix} is near neither perpendicular nor colinear.")
                        
            pass
        case _: # a 4 way cross should generate 4 edges that have 2 perpendiculars
            # also seen when there is a shorter and longer version of the edge both attached to the same 2 boundary edges at the same vertex.
            # in this case the two     
            co=bm.verts[vix].co / SCALE        
            logger.debug(f"Excess perpendiculars. {len(perp_ish)} found at {co}")
            for p in perp_ish:
                logger.debug(f"  {[v.co/SCALE for v in bm.edges[p].verts]}")        
    
    return perp_ish, colinear_ish, neither

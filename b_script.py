"""
Run inside of Blender

"""
from itertools import compress
import os
from math import dist
import bpy
import bmesh
from mathutils import Matrix, Vector
import numpy as np


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

def vertex_coordinates(vert:bpy.types.MeshVertex,matrix_world:Matrix)->tuple:
    """Return the x,y coordinates of a vertex as a tuple of floats
    """
    scale = 39.370079 # inches per meter
    return scale * matrix_world @ vert.co    

def get_z(vert:bpy.types.MeshVertex,elevations:np.array,matrix_world:Matrix)->float:
    """Given a vertex, vert
    the end_point array 
    the matrix world from the object holding the vertex:
    Check blender vertex against imported data to locate z value
    Return the xy,z value if found, otherwise None
    """
    coords = vertex_coordinates(vert,matrix_world)

    # trying default tolerage of 1e-8
    sel=np.isclose(elevations[:,0],coords[0])
    sel=sel & np.isclose(elevations[:,1],coords[1])
    match sum(sel):
        case 0:
            return None
        case 1: 
            z=compress(elevations[:,2],sel).__next__()
            xy=compress(elevations[:,0:2],sel).__next__()
            return xy,z
        case _:
            raise ValueError(f"{coords} found on more than 1 line in end points matrix")

def separate_sections(obj,vertex_sets):
    """Create a new mesh from the items in each vertex_set based on the coordinates.
    Use the coordinates since the index will likely change based on this activity.
    The new mesh will have the same name as the obj but with a suffix .001 etc attached.
    Return dict with new mesh name as key and the coordinates as the values"""
    bpy.ops.object.mode_set(mode='OBJECT')
    known_meshes=set([o.name for o in bpy.context.scene.objects if o.type=='MESH'])
    new_map={} 
    for vx,vertex_set in enumerate(vertex_sets): # vx for debugging
        vertex_cos=[v[1] for v in vertex_set]
        all_cos=[v.co / 0.0254 for v in obj.data.vertices]
        print (f"    {obj.name} has {len(obj.data.edges)} edges")
        for ex,edge in enumerate(obj.data.edges): # ex for debugging
            cos=[all_cos[v] for v in edge.vertices]

            if nearly_in(cos[0],vertex_cos) or nearly_in(cos[1],vertex_cos):
                edge.select = True
                print(f"      {cos} selected")
            else:
                edge.select = False
                print(f"      {cos} rejected")
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.separate() 
        new_mesh_set=set([o.name for o in bpy.context.scene.objects if o.type=='MESH'])
        new_mesh_name=list(new_mesh_set-known_meshes)[0]
        known_meshes=new_mesh_set
        print (f"    -- selected vertices moved to {new_mesh_name}")
        new_map[new_mesh_name]=vertex_cos
        bpy.ops.object.mode_set(mode='OBJECT')
    return new_map

def nearly_in(coord:Vector,search_in):
    """ Drop in replacement for 'in'. Returns True if coord is in set of search_in coordinates
    Considers only x,y (not z)"""
    all_coords=np.array([],dtype=complex)
    c=[complex(a.x,a.y) for a in search_in]
    all_coords=np.concatenate((all_coords,c))
    c=complex(coord.x,coord.y)
    sel=np.isclose(all_coords,c,atol=0.01)
    return any(sel)

def panels_up(obj,height):
    """Extrude all edges to height along z dimension"""
    bpy.ops.object.mode_set(mode='OBJECT') 
    scaled_height=height *.0254 # got to be a better way to scale
    for edge in obj.data.edges:
        edge.select = True
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.extrude_edges_move(TRANSFORM_OT_translate={"value":(0,0,scaled_height)}) 
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.object.mode_set(mode='OBJECT')

def merge_sections(sections):       
    """ Merge sections which contain close points
    close items occur where the imported curves don't quite align
    and where they align but don't share a vertex (like switches)
    example shows .01 is adequate threshhold
    treat the xy pair as a complex number and see which points are close.

    """
    all_coords=np.array([],dtype=complex)
    all_sections=[]
    coords_by_section=[]
    for ix,section in enumerate(sections):    
        sec_coords=[s[1] for s in section]
        c=[complex(a[0],a[1]) for a in sec_coords]
        all_coords=np.concatenate((all_coords,c))
        all_sections+=[ix]*len(c)
        coords_by_section.append(c)

    sibling_groups=[]
    for ix,section_coords in enumerate(coords_by_section):
        if any(ix in sg for sg in sibling_groups):
            continue
        siblings=set()
        for coord in section_coords:
            sel=np.isclose(all_coords,coord,atol=0.01)
            near=set(compress(all_sections,sel))
            siblings=siblings.union(near)
        # check to see if the sibling set has any members in prior sets
        # in which case update that set. Otherwise create a new set
        net_new=True
        for sg in sibling_groups:
            if sg.isdisjoint(siblings):
                net_new=True
                continue # no match here, try the next one
            else:
                sg.update(siblings)
                net_new=False
                break # this sibling set has been consumed (subsumed into another)
        if net_new:
            sibling_groups.append(siblings)
    grouped=[]
    for sg in sibling_groups:
        group=set()
        for ix in sg:
            group.update(sections[ix])
        grouped.append(group)
    return grouped

def cluster_vertices(obj):
    """Cluster all the vertices into several sets based on whether they are linked.
    Returns a list of sets of vertex info.
    The info is index, coordinates. 
    The sets are mutually exclusive (unlike vertex groups)
    These sets then represent the sections of track in a layer (level or inclined) 
    that are isolated from other sections in the same layer by sections of the other layer.
    This is intended to run only on an object that has not yet been extruded - i.e. 2 dimensional.
    """
    mesh = obj.data
    matrix=obj.matrix_world
    all_vertices=mesh.vertices
    bpy.ops.object.mode_set(mode='OBJECT')
    sections=[]

    # Loop through all edges in the mesh and create sections based on 
    # edges sharing the same vertex
    # then merge the sections when there is a point that is close in two sections
    for edge in mesh.edges:
        vert_ix=edge.vertices 
        assert 2==len(vert_ix)
        co=[vertex_coordinates(all_vertices[x],matrix).freeze() for x in vert_ix]
        vertex_info=list(zip(vert_ix,co))
        section_assigned=False
        for section in sections:
            sel= vertex_info[0][0]in [s[0] for s in section]# check index
            if sel:# already in a section, so the other end will go there too
                section.add(vertex_info[1])
                section_assigned=True
                break
        if section_assigned is False:
            sections.append(set(vertex_info))

    sections=merge_sections(sections)    
    pass    
    return sections

def heights_for_obj(obj:bpy.types.Object,elevations:np.array)->dict:
    """Get the defined heights for an object
    Returns a dictionary with the (a tuple of np.float64) as a key and the z as a value
    """
    mesh=obj.data
    found={}
    for vert in mesh.vertices:
        xyz=get_z(vert,elevations=elevations,matrix_world=obj.matrix_world)
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
            print (f"{obj.name} converted to mesh")

def read_track_coordinates(filepath):
    """read the xyz coordinates of the track end points
    assume its has a header row  and the rest is x,y,z data
    """
    elevations=np.loadtxt(filepath,delimiter=',',dtype=float,skiprows=1)
    print (f"Read data for {len(elevations)} end points")
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
    bpy.ops.import_scene.dxf(files=[{"name":filename}],directory=folder,dxf_scale="0.0254")

    # Use inches
    bpy.data.scenes["Scene"].unit_settings.system='IMPERIAL'
    bpy.data.scenes["Scene"].unit_settings.length_unit='INCHES'

def save_file(filepath):
    """Save the blender file under the given name"""
    #temp commented due to bug with vs code connector
    #bpy.ops.wm.save_as_mainfile(filepath= filepath)
    pass

def height_for_vertex_set(vertices:bpy.types.MeshVertices,elevations:np.array,matrix_world:Matrix) ->dict:
    """Determine the height(s) for a vertex set
    Returns dict with xy as key and z as value
    Could be 1 or two items for level or inclined.
    """
    found={}
    for vert in vertices:
        xyz=get_z(vert,elevations=elevations,matrix_world=matrix_world)
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
        raise ValueError(f"{layer} object {obj.name} does not have exactly {rqd} height(s): {heights.values()}")
    height=max(heights.values())
    panels_up(obj,height)

def polarity_changes():
    """ ** probably not needed **
    After the panels are up, some of them are facing the wrong way
    Returns a list of indicators as to whether each face changed polarity
    Works on selected object, all faces.
    First item is always False, 
    Derived frrom:
    https://blender.stackexchange.com/questions/87106/python-find-faces-with-incorrect-normals-to-flip-and-flip-them/87113#87113
    """
    bpy.ops.object.mode_set(mode="OBJECT")

    me=bpy.context.object.data
    bm = bmesh.new()
    bm.from_mesh(me)

    # Reference selected face indices
    bm.faces.ensure_lookup_table()
    face_indexes = [ f.index for f in bm.faces ]

    if len(face_indexes)==0:
        return []
    result=[False]

    prior_normal=bm.faces[face_indexes[0]].normal    
    # Compare successively using the dot product tecnique
    for i in face_indexes: 
        if i==0:
            continue
        this_normal = bm.faces[i].normal

        # Calculate the dot products between the this and prior face normal
        dot =  prior_normal.dot(this_normal) 
        result.append(dot<0) # Reversed faces have a negative dot product value
    return result

def create_bezier(control_points: list):
    """Set up a bezier curve
    control_points an ordered list of 4 x,y,z triplets in inches
    first and last are "anchor" points on the top of the level panels
    the middle two are the points where the incline stops (low,high)

    modified from: https://blender.stackexchange.com/questions/296531/script-to-import-coordinates-and-create-a-bezier-curve
    """
    
    scale=.0254 # convert to meters
    assert len(control_points)==4,'Should have 4 control points'
    cpv=[scale * Vector(cp) for cp in control_points]
    # create bezier curve and add enough control points to it
    bpy.ops.curve.primitive_bezier_curve_add(enter_editmode=True)
    curve = bpy.context.active_object
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
    pass
                
def set_trimmer():
    """Turn the bezier curves into a 3d block than can be used to trim the
    roadbed along a grade
    """
    for obj in bpy.data.objects:
        if obj.name.startswith("Bézier"):
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            print(f"Selected and active: {obj.name}")   
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.extrude_edges_move(TRANSFORM_OT_translate={"value":(0,0,.0254)}) # assume 1" is enough
            bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.modifier_add(type='SOLIDIFY')
            bpy.context.object.modifiers["Solidify"].solidify_mode="NON_MANIFOLD" # i.e. complex
            bpy.context.object.modifiers["Solidify"].thickness = 2*.0254
            bpy.context.object.modifiers["Solidify"].offset = 0
            bpy.ops.object.modifier_apply(modifier="Solidify")

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

        # to determine the bezier points
        bezier_controls=[] # each values= is an ordered list of points
        

        # set up to do level layer before inclined to help get bezier points
        names=[obj.name for obj in bpy.data.objects] 
        layers=[XTC_LAYER_TYPES[name] for name in names] 
        nl=list(zip(names,layers))
        nl=sorted(nl,key=lambda x: x[1],reverse=True)
        names=[x[0]for x in nl]

        for name in names: #level then inclined
            layer=XTC_LAYER_TYPES[name]
            print(f"Starting layer: {layer}")
            obj = bpy.data.objects[name]
            clusters=cluster_vertices(obj)
            sections=separate_sections(obj,clusters)
            
            for mesh_name,section in sections.items():
                # section is mesh_name,(x,y,z) of all points on z=0 plane for this section
                obj=bpy.data.objects[mesh_name]
                all_cos=[v.co / .0254 for v in obj.data.vertices]
                if len(all_cos)==0: # the original mesh now has 0 elements.
                    continue
                print (f"  Starting section {mesh_name}")
                section_verts=[obj.data.vertices[all_cos.index(co) ]for co in section] 
                heights=height_for_vertex_set(section_verts,elevations,obj.matrix_world)
                extrude_to_height(obj,heights,layer)
                # for inclined create a bezier curve.  For that we need heights from the 
                # adjoining level sections.
                if layer =='inclined':
                    # there should a point at the same x,y in one of the level sections
                    bz_points=[]
                    tol=.001
                    for xy,height in heights.items():
                        xyz=(xy[0],xy[1],height)
                        print(f"    looking for {xyz} in level meshes")
                        candidates=[] # for error reporting only
                        for mesh_name in meshes_of_type("level"):
                            mesh=bpy.data.objects[mesh_name]
                            print(f"      {len(mesh.data.edges)} edges in {mesh_name}")
                            bz=[]
                            for edge in mesh.data.edges:
                                vcs=[]
                                near=[]
                                for ev in edge.vertices:
                                    vc=tuple(vertex_coordinates(mesh.data.vertices[ev],mesh.matrix_world ))
                                    vcs.append(vc)
                                    near.append(dist(xyz,vc)<tol)
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
                                        print (f"        {bp}")
                                    break # done, no more edges needed for this side of the bezier curve
                            print(f"      {len(bz_points)} Bezier points")
                            if len(bz)==2: # this side of the curve is satisfied by the most recent mesh
                                break
                    if len(bz_points)!=4: # all meshes used up and we still don't have a match
                        print(f"Could not create bezier control.  Need 4 points but have {len(bz_points)}")
                        print("Here the the ones that we considered")
                        for candidate in candidates:
                            print(f"  {candidate}")
                    else:
                        bezier_controls.append(bz_points)
        
        for bz_points in bezier_controls:
            create_bezier(bz_points)

        convert_to_meshes() # the beziers this time
        set_trimmer()

        # solidify the ramps and the levels
        layers=["inclined","level"]
        for layer in layers:
            for name in meshes_of_type(layer):
                obj=bpy.data.objects[name]
                obj.select_set(True)
                print(f"Selected: {obj.name}")   
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.modifier_add(type='SOLIDIFY')
                bpy.context.object.modifiers["Solidify"].solidify_mode="NON_MANIFOLD" # i.e. complex
                bpy.context.object.modifiers["Solidify"].thickness = .9375*.0254
                bpy.context.object.modifiers["Solidify"].offset = 0
                bpy.ops.object.modifier_apply(modifier="Solidify")

        # do the trim and discard the trim tools
        trimmers=[]
        print ("Trimmers will be: ")
        for obj in bpy.data.objects:
            if obj.name.startswith("Bézier"):
                trimmers.append(obj.name)
                print (f"{obj.name}")
        
        for name in meshes_of_type("inclined"):
            obj=bpy.data.objects[name]
            for trimmer in trimmers:
                bpy.ops.object.select_all(action='DESELECT')
                obj.select_set(True)
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.modifier_add(type='BOOLEAN')
                mname=obj.modifiers[-1].name
                bpy.context.object.modifiers[mname].object=bpy.data.objects[trimmer] 
                bpy.context.object.modifiers[mname].operation = "DIFFERENCE"
                bpy.context.object.modifiers[mname].use_self=True
                bpy.ops.object.modifier_apply(modifier=mname)

        delete_objects_by_name(trimmers)             

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

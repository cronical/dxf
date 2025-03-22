"""
Run inside of Blender

"""
from os import path
from pprint import pprint

import bpy


from xtc3d.io import import_dxf_file,save_file,new_blender_file, read_track_coordinates

from xtc3d.transform import convert_to_meshes, \
    merge_near_vertices,split_xtc_layers,end_points,panels_up_for_layer,create_bezier_controls, \
    create_bezier,solidify_roadbed,solidify_trimmers,delete_objects_by_name,fix_frogs,top_end_edges,bevel_edge_sets,bevel
    
from xtc3d.diagnostics import show_edges

from xtc3d import bl_info,logger,__version__, config, DATA_FOLDER



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
        logger.info("--------------- Start of xtrackcad import -------------------")
        

        elev_file=path.join(DATA_FOLDER,config["data"]["elevations"])
        dxf_file=path.join(DATA_FOLDER,config["data"]["center_line"])
        blend_file=path.join(DATA_FOLDER,config["data"]["blend"])

        elevations=read_track_coordinates(elev_file)


        new_blender_file()

        import_dxf_file(dxf_file)

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
        # this has the side effect of creating a vertex at the center of the top end edge
        # of the inclined roadbed.
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


        #frog_points=fix_frogs(layer_mesh_map)
        #tee=top_end_edges(bevel_info)
        #bes=bevel_edge_sets(tee,frog_points)

        # diag="XTRKCAD3_curve_.003"
        # pprint(tee[diag])
        # show_edges(diag,bes[diag])

        # do the bevels
        # for name,edge_indexes in bes.items():
        #     logger.info(f"Beveling {name}")
        #     obj=bpy.data.objects[name]
        #     bevel(obj,edge_indexes)

        # all the roadbeds into one object
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.join()

        save_file(blend_file)

        
        pass


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

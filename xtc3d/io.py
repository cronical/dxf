"""Collection of I/O functions
"""
from os import path

import bpy
from numpy import loadtxt

from xtc3d import logger, SCALE

def new_blender_file():
    """open a new file and delete the default objects"""
    # temp - next line commented to allow use of debugger
    # bpy.ops.wm.read_homefile(app_template="")

    # Cleaning the scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=True)

def import_dxf_file(filepath):
    """Import the named dxf file from the data folder
    convert to inches"""
    # filepath parameter is being ignored, work around with files + directory
    # https://projects.blender.org/blender/blender/issues/127789
    folder,filename=path.split(filepath)
    bpy.ops.import_scene.dxf(files=[{"name":filename}],directory=folder,dxf_scale=format(f'{SCALE}'))

    # Use inches
    bpy.data.scenes["Scene"].unit_settings.system='IMPERIAL'
    bpy.data.scenes["Scene"].unit_settings.length_unit='INCHES'

def open_file(filepath):
    """Open the blender file under the given name"""
    bpy.ops.wm.open_mainfile(filepath= filepath)
    pass

def save_file(filepath):
    """Save the blender file under the given name"""
    #following may need to be commented if debugging via vs code, due to bug with vs code connector
    bpy.ops.wm.save_as_mainfile(filepath= filepath, check_existing=False)
    logger.info(f"Saved as {filepath}")
    pass

def read_track_coordinates(filepath):
    """read the xyz coordinates of the track end points
    assume its has a header row  and the rest is x,y,z data
    """
    elevations=loadtxt(filepath,delimiter=',',dtype=float,skiprows=1)
    logger.info (f"Read data for {len(elevations)} end points")
    return elevations


    
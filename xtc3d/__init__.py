import logging
from math import radians, cos
from os import path
from sys import exit
import tomllib

logging.basicConfig()
logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)
config_loc=path.expanduser("~/Library/Application Support/Blender/4.3/config/xtrkcad/config.toml")
try:
  with open(config_loc,'rb') as f:
    config=tomllib.load(f)
except FileNotFoundError:
  logger.error("File not found: %s"%config_loc)
  exit(-1)

logger.info("Read config from: %s" % config_loc)

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

XTC_LAYER_TYPES=dict()
for typ,idx in config["layers"].items():
  if typ in ['level','inclined']:
    XTC_LAYER_TYPES[f"XTRKCAD{idx}_curve_"]=typ
LAYER_TYPES_TO_XTC={v: k for k, v in XTC_LAYER_TYPES.items()}

DATA_FOLDER=config["data"]["folder"]

dims=config['dimensions']
SCALE=dims['scale']
BEVEL=dims['bevel']
ROADBED_WIDTH= dims['roadbed_width']
SCENE_HEIGHT = dims['scene_height']

tolerances=config['tolerances']
# allowed variation from perpendicular in degrees translated into number between 0 and 1 commensurate with dot products
TOL_PERPENDICULAR=cos(radians(tolerances['perpendicular'])) # this close to 90 degrees
logger.debug(f"perpendicular tolerance: {TOL_PERPENDICULAR}")

TOL_COLINEAR=cos(radians(tolerances['colinear'])) # this close to 0 degrees. 
logger.debug(f"colinear tolerance: {TOL_COLINEAR}")

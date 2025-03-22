#!/usr/bin/env python
"""Carve up the roadbed into work pieces.
Loop through the foam definitions and repeatedly call blender

run from folder above xtc3d:

./xtc3d/carve.py

"""
import subprocess

from xtc3d import config, logger
from xtc3d.foam import read_foam

for foam_panel in read_foam(config):
  part_no=foam_panel["part_no"]
  cmd=["./deploy_run_carve", f"{part_no}"]
  try:
      ans = subprocess.call(cmd, text=True)

  except subprocess.CalledProcessError as e:
      logger.error(f"Command failed with return code {e.returncode}")
  if ans !=0:
      logger.error(f"Command failed. {ans}")

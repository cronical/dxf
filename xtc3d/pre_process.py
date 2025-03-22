#!/usr/bin/env python
import sys
import ezdxf

from os import path
from xtc3d import config,logger

def main():
    data=config["data"]
    folder=data["folder"]

    in_file=path.join(folder,data["subset"])
    out_file=path.join(folder,data["center_line"])
    logger.info("Reading: %s"% in_file)

    try:
        doc = ezdxf.readfile(in_file)
    except IOError:
        logger.error("Not a DXF file or a generic I/O error.")
        sys.exit(1)
    except ezdxf.DXFStructureError:
        logger.error("Invalid or corrupted DXF file.")
        sys.exit(2)
    # helper function
    def process_entity(msp,e):
        """Remove the some of the lines
        """
        typ=e.dxf.linetype


        match e.dxftype():
            case 'ARC':
                logger.debug (f"{typ} {e.dxftype()} radius: {e.dxf.radius} center: {e.dxf.center} {e.dxf.start_angle} {e.dxf.end_angle}")
            case 'LINE':
                logger.debug (f"{typ} {e.dxftype()} {e.dxf.start} at {e.dxf.start.angle_deg}- {e.dxf.end} at {e.dxf.end.angle_deg}")
            case _:
                pass

        match typ:
            case "DASHED":

                pass
            case "CONTINUOUS":
                e.destroy() # this just marks for deletion until purge is called
                logger.debug ("\tdeleted")
                pass
            case _:
                pass

    # iterate over all entities in modelspace
    msp = doc.modelspace()
    logger.info (f"Starting with {len(msp)} entities")
    for ix,e in enumerate(msp):
        logger.debug(ix)
        if e.dxftype() in ("ARC","LINE"):
            process_entity(msp,e)
        else:
            print(e.dxftype())
    msp.purge()

    logger.info (f"Ended with {len(msp)} entities")

    doc.saveas(out_file)
    logger.info(f"Wrote to {out_file}")

if __name__=="__main__":
    main()
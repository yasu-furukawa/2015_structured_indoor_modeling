import sys
import getopt
from Rhino import *
from Rhino.DocObjects import *
from Rhino.Geometry import *
from Rhino.Input import *
from Rhino.Commands import *
from scriptcontext import doc
import rhinoscriptsyntax as rs

def clearRhino():
    rs.DeleteObjects(rs.AllObjects())

def addWallRhino(corners, thickness, height):
    outWallCrv = rs.AddPolyline(corners)
    inWallCrv = rs.OffsetCurve(outWallCrv, [0,1,0], thickness, [0,0,1], CurveOffsetCornerStyle.Sharp)
    objs = [outWallCrv, inWallCrv]
    btmWall = rs.AddPlanarSrf(objs)[0]
    extrudeLine = rs.AddLine(corners[0],map(sum, zip(corners[0],[0,0,height])))
    allWalls = rs.ExtrudeSurface(btmWall, extrudeLine, True)
    rs.DeleteObjects([outWallCrv,inWallCrv,btmWall,extrudeLine])
    return allWalls

def read(filename):
    with open(filename) as f:
        return [ map(float, line.split())+[0] for line in f]
def addWall (filename, thickness, height):
    corners = read(filename)
    clearRhino()
    addWallRhino(corners, thickness, height)
    
def main():
    wall_height = 50
    wall_thickness = 2
    print('==============Generate Out Walls==============')
    addWall('blueprint_poly_red.txt', wall_thickness, wall_height)
    


if __name__ == "__main__":
    main()

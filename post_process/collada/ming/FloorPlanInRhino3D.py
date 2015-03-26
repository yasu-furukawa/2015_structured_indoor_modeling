from Rhino import *
from Rhino.DocObjects import *
from Rhino.Geometry import *
from Rhino.Input import *
from Rhino.Commands import *
from scriptcontext import doc
import rhinoscriptsyntax as rs

class OneRoom:
	def __init__(self):
		#self.IDs = [[], [], []]
		self.types = {
		"wall" : 0,
		"door" : 1,
		"window" : 2
		}
		self.IDs = [ [] for x in range(1,len(self.types))]
		
	def addID(self, type, id):
		self.IDs[self.types[type]].append(id)
	def changeID(self, type, pos, newid):
		self.IDs[self.types[type]] = newid
		
		
		
class FloorPlanInRhino3D:
	def __init__(self):
		print ("Start Building 3D Floor Plan!")
		self.rooms = []

	def redraw(self):
		doc.Views.Redraw()

	def clear(self):
		objs = rs.AllObjects()
		rs.DeleteObjects(objs)
		
	def subtrackCubeFrom(self, curid, pts, height):
		btmSuf = rs.AddSrfPt(pts)
		extrudeLine = rs.AddLine(pts[0],map(sum, zip(pts[0],[0,0,height])))
		suf = rs.ExtrudeSurface(btmSuf, extrudeLine, True)
		newid = rs.BooleanDifference(curid,suf,True)
		rs.DeleteObjects([btmSuf,extrudeLine,suf])	
		return newid
		
	def addRoom(self):
		self.rooms = self.rooms + [OneRoom()]
		
	def addWall(self, corners, thickness, height):
		print ("Building Wall...")
		pts = [ Point3d(x[0],x[1],x[2]) for x in corners]
		#ptsUp = [ Point3d(x[0],x[1],x[2]+10) for x in corners]

		outerWall = PolylineCurve(pts)
		innerWall = outerWall.Offset(Plane.WorldXY, -thickness,
			doc.ModelAbsoluteTolerance, CurveOffsetCornerStyle.Sharp)[0]
		#outerWallUp = PolylineCurve(ptsUp)
		#plane = Plane.WorldXY
		#plane.Translate(Vector3d(0,0,10))
		#innerWallUp = outerWall.Offset(plane, -thickness,
			#doc.ModelAbsoluteTolerance, CurveOffsetCornerStyle.Sharp)[0]
		
		#MING: 2D floodPlan here
		#doc.Objects.AddCurve(outerWall)
		#doc.Objects.AddCurve(innerWall)
		
		#doc.Objects.AddCurve(outerWallUp)
		#doc.Objects.AddCurve(innerWallUp)		
			
		#ext = Extrusion.Create(innerWall[0], 10, True)
		#doc.Objects.AddMesh(ext.GetMesh())
		
		suf = Surface.CreateExtrusion(innerWall, Vector3d(0,0,height))
		#doc.Objects.AddSurface(suf)
		sufo = Surface.CreateExtrusion(outerWall, Vector3d(0,0,height))
		#doc.Objects.AddSurface(sufo)
		
		#brep = Brep()
		#brep.AddEdgeCurve(innerWall)
		#brep.AddEdgeCurve(outerWall)
		#Brep.
		#walls = Brep.CreateFromLoft([innerWall,outerWall],Point3d(0,0,0),Point3d(0,0,10),LoftType.Tight,False)
		btmWall = Brep.CreateFromLoft([innerWall,outerWall],Point3d.Unset,Point3d.Unset,LoftType.Tight,False)[0]
		
		upWall = btmWall
		upWall.Translate(0,0,height)
		
		inWall = Brep.CreateFromSurface(suf)
		outWall = Brep.CreateFromSurface(sufo)
		
		#allWalls = Brep.CreateBooleanUnion([btmWall,upWall,inWall,outWall],doc.ModelAbsoluteTolerance)
		
		#doc.Objects.AddBrep(allWalls)
		#print(len(allWalls))
		doc.Objects.AddBrep(btmWall)
		doc.Objects.AddBrep(upWall)
		doc.Objects.AddBrep(inWall)
		doc.Objects.AddBrep(outWall)
		
		#oWall = Brep.CreateFromLoft([outerWall],Point3d(0,0,0),Point3d(0,0,10),LoftType.Tight,False)[0]
		#doc.Objects.AddBrep(oWall)
		#crvs = rs.AllObjects()
		
		#crvids = rs.GetObjects(message="select curves to loft", filter=rs.filter.curve, minimum_count=2)
		#if not crvids: return
		#rs.AddLoftSrf(object_ids=crvids, loft_type = 3) #3 = tight
		
		#rs.AddLoftSrf(object_ids=crvs, loft_type = 3)
		self.redraw()
		
	def addWall2(self, corners, thickness, height):
		print ("Building Wall...")
		pts = [ Point3d(x[0],x[1],x[2]) for x in corners]
		outWallCrv = PolylineCurve(pts)
		inWallCrv = outWallCrv.Offset(Plane.WorldXY, -thickness,
			doc.ModelAbsoluteTolerance, CurveOffsetCornerStyle.Sharp)[0]
		doc.Objects.AddCurve(outWallCrv)
		doc.Objects.AddCurve(inWallCrv)
		
		objs = rs.GetObjects("Select planar curves to build surface", rs.filter.curve)
		#objs = [outWallCrv.Id, inWallCrv.Id]
		if objs: btmWall = rs.AddPlanarSrf(objs)[0]
		extrudeLine = rs.AddLine(corners[0],map(sum, zip(corners[0],[0,0,height])))
		allWalls = rs.ExtrudeSurface(btmWall, extrudeLine, True)
		PolylineCurve.GetObjectData()

		
	def addWall3(self, corners, thickness, height):
		outWallCrv = rs.AddPolyline(corners)
		#plane = rs.AddSrfPt([[-1,-1,0],[-1,30,0],[30,30,0],[30,-1,0]])
		#rs.OffsetCurveOnSurface(outWallCrv, plane, -thickness)
		inWallCrv = rs.OffsetCurve(outWallCrv, [0,1,0], thickness, [0,0,1], CurveOffsetCornerStyle.Sharp) 
		objs = [outWallCrv, inWallCrv]
		btmWall = rs.AddPlanarSrf(objs)[0]
		extrudeLine = rs.AddLine(corners[0],map(sum, zip(corners[0],[0,0,height])))
		allWalls = rs.ExtrudeSurface(btmWall, extrudeLine, True)
		rs.DeleteObjects([outWallCrv,inWallCrv,btmWall,extrudeLine])
		return allWalls
		

	def addDoor(self, wallid, pts, height):
		return self.subtrackCubeFrom(wallid, pts, height)
		
	def addWindow(self, wallid, pts, height):
		return self.subtrackCubeFrom(wallid, pts, height)	
		
	def addHallWall(self, roomids, corners, corners2, thickness, height):
		outWallCrv = rs.AddPolyline(corners)
		inWallCrv = rs.AddPolyline(corners2)
		#inWallCrv = rs.OffsetCurve(outWallCrv, [20,14,0], thickness, [0,0,1], CurveOffsetCornerStyle.Sharp) 
		outExtrudeLine = rs.AddLine(corners[0],map(sum, zip(corners[0],[0,0,height])))
		outBtmSuf = rs.AddPlanarSrf(outWallCrv)[0]
		outWall = rs.ExtrudeSurface(outBtmSuf, outExtrudeLine, True)
		inExtrudeLine = rs.AddLine(corners[0],map(sum, zip(corners[0],[0,0,height-thickness])))
		inBtmSuf = rs.AddPlanarSrf(inWallCrv)[0]
		inWall = rs.ExtrudeSurface(inBtmSuf, inExtrudeLine, True)
		hallway = rs.BooleanDifference(outWall,inWall,True)
		rs.DeleteObjects([outWallCrv,inWallCrv,outExtrudeLine,outBtmSuf,outWall,inExtrudeLine,inBtmSuf,inWall])
		newroomids = list(roomids)
		for roomid in roomids:
			newhallway = rs.BooleanDifference(hallway, roomid,False)
			newroomid =  rs.BooleanDifference(roomid, hallway,False)
			newroomids.append(newroomid)
			rs.DeleteObject(hallway)
			rs.DeleteObject(roomid)
			hallway = newhallway
		return hallway
		
		
	def testCase(self):
		self.clear()
		
		#------------add 1st room------------
		self.addRoom()
		#add the wall
		corners = [
		[0,0,0]
		,[10,0,0]
		,[10,5,0]
		,[15,5,0]
		,[15,20,0]
		,[11,20,0]
		,[11,18,0]
		,[4,18,0]
		,[4,20,0]
		,[0,20,0]
		,[0,0,0]
		]
		thickness = 0.8
		height = 10
		id1 = self.addWall3(corners, thickness, height)
		#self.rooms[0].addID("wall", wallid) 
		#add door
		doorBtmPts = [
		[1,0,0]
		,[3.5,0,0]
		,[3.5,thickness,0]
		,[1,thickness,0]
		]
		doorHeight = 7
		id1 = self.addDoor(id1, doorBtmPts, doorHeight)
		
		#add door
		
		hallThickness = 0.5
		hallBtmPts1 = [
		[15,10,0]
		,[25,10,0]
		,[25,14,0]
		,[15,14,0]
		,[15,10,0]
		]
		hallBtmPts2 = [
		[15,10+hallThickness,0]
		,[25,10+hallThickness,0]
		,[25,14-hallThickness,0]
		,[15,14-hallThickness,0]
		,[15,10+hallThickness,0]
		]
		hallHeight = 7
		
		doorBtmPts2 = [
		[15-thickness,10+hallThickness,0]
		,[15,10+hallThickness,0]
		,[15,14-hallThickness,0]
		,[15-thickness,14-hallThickness,0]
		]
		doorHeight2 = hallHeight - hallThickness
		id1 = self.addDoor(id1, doorBtmPts2, doorHeight2)
		
		#add window
		winBtmPts = [
		[0,3,4]
		,[thickness,3,4]
		,[thickness,7,4]
		,[0,7,4]
		]
		doorHeight = 4
		id1 = self.addWindow(id1, winBtmPts, doorHeight)
		
		#add 2nd room
		#id2 = rs.CopyObjects(id1, [30,0,0])
		id2 = rs.MirrorObject(id1, [20,0,0], [20,20,0], True)
		
		#add hallway
		
		self.addHallWall([id1,id2], hallBtmPts1, hallBtmPts2, hallThickness, hallHeight)
		
		
		
		
		
		
		

if __name__ == '__main__':
	fp = FloorPlanInRhino3D()
	fp.testCase()
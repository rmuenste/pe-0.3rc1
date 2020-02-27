#./salome -t import_unv1.py
from salome.gui import helper
from salome.smesh import smeshBuilder
import SMESH
from SMESH_mechanic import *
import tempfile
import os

smesh = smeshBuilder.New()
#smesh.SetEnablePublish( False ) # Set to False to avoid publish in study if not needed or in some particular situations:
                                 # multiples meshes built in parallel, complex and numerous mesh edition (performance)

Cube_Hexa_unv = smesh.CreateMeshesFromUNV(r'/home/raphael/code/GitHub/FC-2019/FullC0ntact/unit_tests/boundarylayer_generation/myout.unv')

myMesh = Cube_Hexa_unv
boundaryGroup = myMesh.MakeBoundaryElements(SMESH.BND_2DFROM3D, "bndry2")

theNewFaceList = myMesh.GetElementsByType(SMESH.FACE)

lastIdx = len(theNewFaceList) - 1

rangeFilter = smesh.GetFilter(SMESH.FACE, SMESH.FT_RangeOfIds, Threshold="%i-%i" %(theNewFaceList[lastIdx-1], theNewFaceList[lastIdx]))
ids = myMesh.GetIdsFromFilter(rangeFilter)
rangeGroup = myMesh.GroupOnFilter( SMESH.FACE, "FaceOnRotor", rangeFilter)


cr = myMesh.GetGroupByName("CyclicR")
cl = myMesh.GetGroupByName("CyclicL")
cr[0].SetName("oldCyclicR")
cl[0].SetName("oldCyclicL")

# Add final cyclicr group
cr[0].GetName()
cr[0].GetListOfID()
cr[0].GetListOfID()[0]
faceID = cr[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
CyclicR = myMesh.GroupOnFilter( SMESH.FACE, "CyclicR", filter)

# Add final cyclicl group
cl[0].GetName()
cl[0].GetListOfID()
cl[0].GetListOfID()[0]
faceID = cl[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
CyclicL = myMesh.GroupOnFilter( SMESH.FACE, "CyclicL", filter)

# Add final rotor group
faceID = rangeGroup.GetListOfID()[0]
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
RotorI = myMesh.GroupOnFilter( SMESH.FACE, "RotorI", filter)


# Add final stator group
statorGroup = myMesh.GetGroupByName("FaceOnStator")
faceID = statorGroup[0].GetListOfID()[0]
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
RotorI = myMesh.GroupOnFilter( SMESH.FACE, "StatorI", filter)

allGroups = myMesh.GetGroups()
for group in allGroups:
    if group.GetName() not in ("CyclicR", "CyclicL", "StatorI", "RotorI"):
        myMesh.RemoveGroup(group)

try:
  myMesh.ExportUNV(r'/home/raphael/code/GitHub/FC-2019/FullC0ntact/unit_tests/boundarylayer_generation/newmesh.unv')
  pass
except:
  print('ExportUNV() failed. Invalid file name?')


#meshRef = helper.getSObjectSelected()[0].GetObject()
#smesh = smeshBuilder.New(salome.myStudy)
#myMesh = smesh.Mesh(meshRef)

#myMesh = Cube_Hexa_unv
#
#rotorGroup = myMesh.GetGroupByName("Stator")
#rotorGroup[0].GetName()
#rotorGroup[0].GetListOfID()
#rotorGroup[0].GetListOfID()[0]
#faceID = rotorGroup[0].GetListOfID()[0]
#faceID
#filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
#ids = myMesh.GetIdsFromFilter(filter)
#print (len(ids))
#myGroup = myMesh.GroupOnFilter( SMESH.FACE, "group on filter", filter)
##myGroup.Size()
#myMesh.Compute()
#aPath = "/home/raphael/code/GitHub/FC-2019/FullC0ntact/unit_tests/boundarylayer_generation/StatorI.dat"
#datFile = tempfile.NamedTemporaryFile(suffix=".dat").name
#myMesh.ExportDAT(datFile, meshPart=myGroup)
#os.rename(datFile, aPath)
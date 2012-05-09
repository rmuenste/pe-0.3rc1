// extern "C" void initsimulation();
// extern "C" void initterminalvelbench();
// extern "C" void initdkt();
// extern "C" void initbasf();
// 
// 
// extern "C" void startcollisionpipeline();
// extern "C" void clearcollisionpipeline();
// extern "C" void logdistance();
// extern "C" void logposition();
// extern "C" void logvelocity();
// extern "C" void logangularvelocity();
// 
// extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin);
// extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin);
// 
// extern "C" void setdensity(double *ddens);
// extern "C" void setposition(double *dx,double *dy,double *dz);
// extern "C" void setrotation(double *dx,double *dy,double *dz);
// extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
// extern "C" void settimestep(double *dTime);
// extern "C" void settime(double *dTime);
// 
// extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime);
// extern "C" void writepvtu(int *iNodes,int *iTime);
// extern "C" void writeparticles(int *iout);
// extern "C" void writepolygon(int *iout);
// 
// extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist);
// extern "C" void getnumparticles(int *nParts);
// extern "C" void getradius(double *drad, int *iid);
// extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid);
// extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
// extern "C" void getpos(double *dx,double *dy,double *dz,int *iID);
// extern "C" void getvel(double *dx,double *dy,double *dz,int *iID);
// extern "C" void getdensity(double *ddens, int *iid);
//

#include <cppinterface.h>
#include <string>
#include <aabb3.h>
#include <iostream>
#include <genericloader.h>
#include <unstructuredgrid.h>
#include <distops3.h>
#include <triangulator.h>
#include <iomanip>
#include <sstream>
#include <intersectorray3tri3.h>
#include <vtkwriter.h>
#include <world.h>
#include <particlefactory.h>
#include <collisionpipeline.h>
//#include <mymath.h> asdf
#include <distancetriangle.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <log.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <reader.h>
#include <deformparameters.h>
#include <objloader.h>
#include <hspatialhash.h>
#include <broadphasestrategy.h>
#include <distancemeshpoint.h>
#include <distancetriangle.h>
#include <intersector2aabb.h>
#include <perftimer.h>
#include <motionintegratorsi.h>

using namespace i3d;

struct funcx
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.x < elem2.vVec.x;
  }
};

struct funcy
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.y < elem2.vVec.y;
  }
};  

struct funcz
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.z < elem2.vVec.z;
  }
};  


Real a = CMath<Real>::MAXREAL;
CUnstrGrid myGrid;
CWorld myWorld;
CCollisionPipeline myPipeline;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
CTimeControl myTimeControl;
CWorldParameters myParameters;
CDeformParameters myDeformParameters;
CPerfTimer myTimer;
CRigidBodyMotion *myMotion;
CDistanceMeshPointResult<Real> resMaxM1;
CDistanceMeshPointResult<Real> resMax0;
CDistanceMeshPointResult<Real> *resCurrent;

#ifdef FEATFLOWLIB
extern "C" void communicateforce_(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz);
#endif

double xmin=0;
double ymin=0;
double zmin=0;
double xmax=0.1;
double ymax=0.1;
double zmax=0.16;
Real radius = Real(0.075);
int iReadGridFromFile = 0;

C3DModel Model;
CLog mylog;
CLog myPositionlog;
CLog myVelocitylog;
CLog myAngVelocitylog;
CLog myCollisionlog;
CAABB3r boxDomain;

#ifdef FEATFLOWLIB
extern "C" void velocityupdate()
{

  double *ForceX = new double[myWorld.m_vRigidBodies.size()];
  double *ForceY = new double[myWorld.m_vRigidBodies.size()];;
  double *ForceZ = new double[myWorld.m_vRigidBodies.size()];;
  double *TorqueX = new double[myWorld.m_vRigidBodies.size()];;
  double *TorqueY = new double[myWorld.m_vRigidBodies.size()];;
  double *TorqueZ = new double[myWorld.m_vRigidBodies.size()];
  
  //get the forces from the cfd-solver
  communicateforce_(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
  
  std::vector<VECTOR3> vForce;
  std::vector<VECTOR3> vTorque;  
  
  std::vector<CRigidBody*>::iterator vIter;  
  int count = 0;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++,count++)
  {
    CRigidBody *body    = *vIter;
    vForce.push_back(VECTOR3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(VECTOR3(TorqueX[count],TorqueY[count],TorqueZ[count]));
  }

  //calculate the forces in the current timestep by a semi-implicit scheme
  myPipeline.m_pIntegrator->UpdateForces(vForce,vTorque);

  delete[] ForceX;
  delete[] ForceY;
  delete[] ForceZ;
  delete[] TorqueX;
  delete[] TorqueY;
  delete[] TorqueZ;      
  
}
#endif

//-------------------------------------------------------------------------------------------------------

extern "C" void intersecthexbody(double dMinMax[][3], int *iid, int *intersection)
{
 
 Real minx = Real(dMinMax[0][0]);
 Real miny = Real(dMinMax[0][1]);
 Real minz = Real(dMinMax[0][2]);
 Real maxx = Real(dMinMax[1][0]);
 Real maxy = Real(dMinMax[1][1]);
 Real maxz = Real(dMinMax[1][2]);

 CAABB3r box(VECTOR3(minx,miny,minz),VECTOR3(maxx,maxy,maxz)); 
 int i = *iid;
 CRigidBody *pBody  = myWorld.m_vRigidBodies[i];
 CShaper *pShape    = pBody->GetWorldTransformedShape();
 CAABB3r boxBody    = pShape->GetAABB();
 CIntersector2AABB<Real> intersector(box,boxBody);
 bool bIntersect =  intersector.Intersection();
 delete pShape;
 if(bIntersect)
   *intersection=1;
 else
   *intersection=0;


}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettiming(double *time)
{
  double dtime=0.0;
  dtime = myTimer.GetTime();
  *time=dtime;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void starttiming()
{
  myTimer.Start();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void updateelementsprev(int *ibody)
{
  int i  = *ibody; 
  int e1 = myWorld.m_vRigidBodies[i]->m_iElements.size();
  int e2 = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
  int itotal = e1+e2;
  myWorld.m_vRigidBodies[i]->m_iElementsPrev = itotal;

}

//-------------------------------------------------------------------------------------------------------

extern "C" void setdomainbox(double vmin[3], double vmax[3])
{
  boxDomain = CAABB3r(VECTOR3(vmin[0],vmin[1],vmin[2]),VECTOR3(vmax[0],vmax[1],vmax[2]));
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setmyid(int *myid)
{
  int id = *myid;
  myWorld.m_myParInfo.SetID(id);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void intersectdomainbody(int *ibody,int *domain,int *intersection)
{
  // call the intersector for the bounding box of the body and 
  // and the domain bounding box
  int i = *ibody;
  CRigidBody *pBody  = myWorld.m_vRigidBodies[i];
  CAABB3r boxBody    = pBody->m_pShape->GetAABB();
  CIntersector2AABB<Real> intersector(boxDomain,boxBody);
  bool bIntersect =  intersector.Intersection();
  if(bIntersect)
    *intersection=1;
  else
    *intersection=0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void vertexorderxyz(int *invt,int iorder[], double dcorvg[][3])
{
  int nvt = *invt;
  std::vector<sPerm> permArray;
  
  for(int i=0;i<nvt;i++)
  {
    sPerm perm;
    Real x = dcorvg[0][i];
    Real y = dcorvg[1][i];
    Real z = dcorvg[2][i];
    CVector3<Real> vec(x,y,z);        
    perm.vVec =  vec;
    perm.index  = i;
    permArray.push_back(perm);
  }
  
  std::stable_sort(permArray.begin(),permArray.end(),funcz());
  std::stable_sort(permArray.begin(),permArray.end(),funcy());
  std::stable_sort(permArray.begin(),permArray.end(),funcx());
  
  for(int i=0;i<nvt;i++)
  {
    iorder[i]=permArray[i].index;
  }  
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void updateMax0(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  
  // compute distance point triangle
  int k = resMaxM1.iTriangleID;
  CTriangle3<Real> &tri3 = resMaxM1.pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();  
  
  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent           = &resMax0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setMaxM1(double *dx,double *dy,double *dz,double *dist)
{ 
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);  
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  
  resMaxM1.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMaxM1.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMaxM1.pNode       = distMeshPoint.m_Res.pNode;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setstartbb(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);  
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  
  
  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent           = &resMax0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistancebbid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  
  // compute the distance to the triangle found for the reference point
  int k = resCurrent->iTriangleID;
  CTriangle3<Real> &tri3 = resCurrent->pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();    
  
  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void clearelementlists(int *ibody)
{
  int i = *ibody;
  myWorld.m_vRigidBodies[i]->m_iElements.clear();
  myWorld.m_vRigidBodies[i]->m_iBoundaryElements.clear();
}

//-------------------------------------------------------------------------------------------------------

void addelement2list(int *iel, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  myWorld.m_vRigidBodies[i]->m_iElements.push_back(ielc);
}

//-------------------------------------------------------------------------------------------------------

void addelement2bndlist(int *iel, int *idofs, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  int idofsc = *idofs;
  myWorld.m_vRigidBodies[i]->m_iBoundaryElements.push_back(std::pair<int,int>(ielc,idofsc));
}

//-------------------------------------------------------------------------------------------------------

void getelementarray(int* elements, int *idofselement, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.begin();
  for(int j=0;iter!=myWorld.m_vRigidBodies[i]->m_iBoundaryElements.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
    idofselement[j]= mypair.second;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettotalelements(int* nel, int* ibody)
{
  int i = *ibody;
  int e1 = myWorld.m_vRigidBodies[i]->m_iElements.size();
  int e2 = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
  int itotal = e1+e2;
  *nel = itotal;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsprev(int* nel, int* ibody)
{
  int i = *ibody;
  *nel  = myWorld.m_vRigidBodies[i]->m_iElementsPrev;
}

//-------------------------------------------------------------------------------------------------------

void getallelements(int* elements, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.begin();
  std::list<int>::iterator iter2 = myWorld.m_vRigidBodies[i]->m_iElements.begin();
  int j;
  for(j=0;iter!=myWorld.m_vRigidBodies[i]->m_iBoundaryElements.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
  }

  for(;iter2!=myWorld.m_vRigidBodies[i]->m_iElements.end();iter2++,j++)
  {
    int iel = *iter2;
    elements[j]= iel;
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsinside(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.m_vRigidBodies[i]->m_iElements.size();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsbndry(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
}

//-------------------------------------------------------------------------------------------------------

void getforce(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID;
  CVector3<double> force(myWorld.m_vRigidBodies[i]->m_vForce.x,
                         myWorld.m_vRigidBodies[i]->m_vForce.y,
                         myWorld.m_vRigidBodies[i]->m_vForce.z);
  *dx=force.x;
  *dy=force.y;
  *dz=force.z;
}

//-------------------------------------------------------------------------------------------------------

void gettorque(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID; 
  CVector3<double> torque(myWorld.m_vRigidBodies[i]->m_vTorque.x,
                          myWorld.m_vRigidBodies[i]->m_vTorque.y,
                          myWorld.m_vRigidBodies[i]->m_vTorque.z);
  *dx=torque.x;
  *dy=torque.y;
  *dz=torque.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumpworld()
{
  std::cout<<myWorld<<std::endl;
  mylog.Write(myWorld.ToString().c_str());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumprigidbodies()
{
  CRigidBodyIO writer;
  writer.Write(myWorld,"particles.i3d");
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logposition()
{
  std::vector<CRigidBody*>::iterator vIter;
  //Check every pair
  mylog.Write("Simulation time: %f",myTimeControl.GetTime());
  int count = 0;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body =*vIter;
    mylog.Write("Position of body %i: %f %f %f:",count,body->m_vCOM.x,body->m_vCOM.y,body->m_vCOM.z);
    count++;
  } 
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logvelocity()
{
  i3d::Real rad = myWorld.m_vParticles[0].m_dRadius + myWorld.m_vParticles[1].m_dRadius;
  i3d::Real dist = fabs((myWorld.m_vParticles[1].m_vMeshes[0].m_vOrigin-myWorld.m_vParticles[0].m_vMeshes[0].m_vOrigin).mag()-rad);
  mylog.Write("%f %f",dist,myTimeControl.GetTime());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logangularvelocity()
{
  i3d::Real rad = myWorld.m_vParticles[0].m_dRadius + myWorld.m_vParticles[1].m_dRadius;
  i3d::Real dist = fabs((myWorld.m_vParticles[1].m_vMeshes[0].m_vOrigin-myWorld.m_vParticles[0].m_vMeshes[0].m_vOrigin).mag()-rad);
  mylog.Write("%f %f",dist,myTimeControl.GetTime());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logdistance()
{
  i3d::Real rad = myWorld.m_vParticles[0].m_dRadius + myWorld.m_vParticles[1].m_dRadius;
  i3d::Real dist = fabs((myWorld.m_vParticles[1].m_vMeshes[0].m_vOrigin-myWorld.m_vParticles[0].m_vMeshes[0].m_vOrigin).mag()-rad);
  mylog.Write("%f %f",dist,myTimeControl.GetTime());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logcollision()
{
  
  std::list<CCollisionInfo>::iterator Iter;
  int nContacts=0;
  //loop to determine the total number of contact points
  for(Iter=myPipeline.m_CollInfo.begin();Iter!=myPipeline.m_CollInfo.end();Iter++)
  {
    CCollisionInfo &info = *Iter;
    if(!info.m_vContacts.empty())
    {
      CContact &contact = info.m_vContacts.front();
      nContacts++;
      myCollisionlog.Write("Contact(%i,%i)",contact.id0,contact.id1);
    }
  }
  myCollisionlog.Write("Simulation time: %f ,Total number of collisions: %i",myTimeControl.GetTime(),nContacts);
}


//-------------------------------------------------------------------------------------------------------

extern "C" void gettype(int *itype, int *iid)
{
  int i = *iid;
  *itype = myWorld.m_vRigidBodies[i]->m_iShape;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void startcollisionpipeline()
{
  //start the collision pipeline
  myPipeline.StartPipeline();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void clearcollisionpipeline()
{
  //erase the old info, if there is any
  //myResponses.clear();
  //make a copy of the responses for postprocessing
  //myResponses=myPipeline.m_Response->m_Responses;
  myPipeline.m_CollInfo.clear();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writeparticles(int *iout)
{
  int iTimestep=*iout;
  std::ostringstream sName,sNameParticles;
  std::string sModel("_vtk/model.vtk");
  std::string sParticle("solution/particles.i3d");
  CVtkWriter writer;
  sName<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sNameParticles<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sModel.append(sName.str());
  sParticle.append(sNameParticles.str());
  
  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());

  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
  rbwriter.Write(myWorld,sParticle.c_str(),false);
  
/*  std::ostringstream sNameHGrid;  
  std::string sHGrid("_gmv/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->ConvertToUnstructuredGrid(hgrid);

  writer.WriteUnstr(hgrid,sHGrid.c_str());  */
  
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writepvtu(int *iNodes,int *iTime)
{
  int nodes=*iNodes;
  int time =*iTime;
  CVtkWriter writer;
  writer.WritePVTU(nodes,time);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writetri(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode)
{
  int NEL=*iNEL;
  int inode=*iNode;
  
  if(inode==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;

    std::string strEnding(".tri");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  using namespace std;

	std::ostringstream sName;
	sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;
	
	string strEnding(".tri");
	string strFileName("_gmv/res_");
	strFileName.append(sName.str());
	strFileName.append(strEnding);
	
	//open file for writing
	FILE * myfile = fopen(strFileName.c_str(),"w");

  //check
	fprintf(myfile,"Coarse mesh exported by stdQ2P1 \n");
	fprintf(myfile,"Version 0.1a \n");

	fprintf(myfile,"    %i %i 0 8 12 6     NEL,NVT,NBCT,NVE,NEE,NAE\n",NEL,8*NEL);
  //write the points data array to the file
  fprintf(myfile,"DCORVG\n");
  for(int i=0;i<8*NEL;i++)
  {
    fprintf(myfile,"%f %f %f\n",dcorvg[i][0],dcorvg[i][1],dcorvg[i][2]);
  }//end for

  fprintf(myfile,"KVERT\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"%i %i %i %i %i %i %i %i\n",iKVERT[i][0],iKVERT[i][1],iKVERT[i][2],iKVERT[i][3],iKVERT[i][4],iKVERT[i][5],iKVERT[i][6],iKVERT[i][7]);
  }

  fprintf(myfile,"KNPR\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"0\n");
  }

	fclose( myfile );
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime)
{
  int NEL=*iNEL;
  int NVT=*iNVT;
  int node=*iNode;
  int time=*iTime;
  
  if(node==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<node<<"."<<std::setfill('0')<<std::setw(4)<<time;

    std::string strEnding(".vtu");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  CVtkWriter writer;
  writer.WritePUXML(NEL,NVT,iKVERT,dcorvg,vu,vv,vw,vp,dist,node,time);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

void creategrid()
{
  i3d::Real drad = myWorld.m_vParticles[0].m_dRadius;
  i3d::Real d    = 2.0 * drad;
  i3d::Real distbetween = 0.25 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, myGrid.m_vMax.z-d);
  int row=0;
  for(int i=0;i<myWorld.m_vParticles.size();i++)
  {
    if((i+1)%(perrow+1) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.z -= d+distbetween;
    }
    myWorld.m_vParticles[i].m_vMeshes[0].MoveToPosition(pos);
    pos.x+=d+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

void createcolltest()
{
  i3d::Real drad = myWorld.m_vParticles[0].m_dRadius;
  i3d::Real d    = 2.0 * drad;
/*  i3d::Real distbetween = 0.25 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, myGrid.m_vMax.z-d);
  int row=0;
  for(int i=0;i<myWorld.m_vParticles.size();i++)
  {
    if((i+1)%(perrow+1) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.z -= d+distbetween;
    }
    myWorld.m_vParticles[i].m_vMeshes[0].MoveToPosition(pos);
    pos.x+=d+distbetween;
  }*/
  
  myWorld.m_vParticles[0].m_vMeshes[0].MoveToPosition(VECTOR3(1,1,3));
  myWorld.m_vParticles[1].m_vMeshes[0].MoveToPosition(VECTOR3(1+d+0.05,1,3));
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist)
{

  CDistOps3 op;
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  CVector3<Real> vec(x,y,z);
  double dist=op.BruteForceDistance(Model, vec);
  *ddist=dist;

}//end getcenter

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistanceid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  ddist = distMeshPoint.ComputeDistance();
  *dist=ddist;
}//end getdistance

//-------------------------------------------------------------------------------------------------------

void intersecbodyelement(int *ibody,int *iel,double vertices[][3])
{

/*    if(body->m_iShape == CRigidBody::BOUNDARYBOX)
    {
      return;
    }*/
    int i = *ibody;
    i--;
    CRigidBody *body = myWorld.m_vRigidBodies[i];
    VECTOR3 verts[8];
    for(int i=0;i<8;i++)
    {
      verts[i]=VECTOR3(Real(vertices[i][0]),Real(vertices[i][1]),Real(vertices[i][2]));
    }
    int in = body->NDofsHexa(verts);
    
    if(in > 0)body->m_iElement = *iel;
    
}

//-------------------------------------------------------------------------------------------------------

void isboundarybody(int* isboundary, int* ibodyc)
{
  int i = *ibodyc;
  if(myWorld.m_vRigidBodies[i]->m_iShape==CRigidBody::BOUNDARYBOX)
  {
    *isboundary = 1;
  }
  else
  {
    *isboundary = 0;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin)
{
  CDistOps3 op;
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[0];
  CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  C3DModel &model = object->m_Model;
  int in=op.BruteForceInnerPointsStatic(model,vec);
  *isin=in;
}//end isinelement

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelementperf(double *dx,double *dy,double *dz,int *isin)
{

  CDistOps3 op;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  CVector3<Real> point(x,y,z);
  int in=0;

  //locate the cell in that the point is
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());    
  
  for(int level=0;level<=pHash->GetMaxLevel();level++)
  {
    if(pHash->IsBoundaryLevel(level))
      continue;

    //get the cell on the current level
    CCellCoords cell = pHash->GetCell(point,level);

    //get the entries of the cell
    std::vector<CSpatialHashEntry> *vEntries = pHash->GetCellEntries(cell);
    
    //test for all entries if the point is inside
    //loop through the entries of the hash bucket
    std::vector<CSpatialHashEntry>::iterator viter = vEntries->begin();
    
    //check cell 
    for(;viter!=vEntries->end();viter++)
    {
      //get the rigid body
      CRigidBody *pBody = viter->m_pBody;
      
      //check if inside, if so then leave the function
      if(pBody->IsInBody(point))
      {
        in=1;
        *isin=pBody->m_iID;
        return;
      }

    }//end for viter
        
  }//end for level
  
}//end isinelementperf

//-------------------------------------------------------------------------------------------------------

extern "C" void isinobstacle(double *dx,double *dy,double *dz,int *isin)
{
  int inside = 0;
  CDistOps3 op;
  int in[3];
  CVector3<float> vec(*dx,*dy,*dz);
  CRigidBody *pBody0 = myWorld.m_vRigidBodies[0];
  CMeshObject<i3d::Real> *object0 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody0->m_pShape);
  C3DModel &model0 = object0->m_Model;

  CRigidBody *pBody1 = myWorld.m_vRigidBodies[1];
  CMeshObject<i3d::Real> *object1 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody1->m_pShape);
  C3DModel &model1 = object1->m_Model;

  CRigidBody *pBody2 = myWorld.m_vRigidBodies[2];
  CMeshObject<i3d::Real> *object2 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody2->m_pShape);
  C3DModel &model2 = object2->m_Model;

  VECTOR3 orig(vec.x,vec.y,vec.z);
  CRay3r ray0(orig,VECTOR3(0,0,1));
  CRay3r ray1(orig,VECTOR3(0,0,-1));
  CRay3r ray2(orig,VECTOR3(0,1,0));

  in[0]=op.BruteForcefbm(model0, orig, ray0);
  in[1]=op.BruteForcefbm(model1, orig, ray1);
  in[2]=op.BruteForcefbm(model2, orig, ray2);
  

  for(int i=0;i<3;i++)
  {
    if(in[i]==1)
    {
      inside=1;
      break;
    }
  }
  *isin=inside;
}//end isinelement

//-------------------------------------------------------------------------------------------------------

extern "C" void setposition(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  CVector3<Real> vNewPos(x,y,z);

  Model.m_vMeshes[0].MoveToPosition(vNewPos);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setrotation(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  

  CVector3<Real> vNewRot(x,y,z);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void writepolygon(int *iout)
{
  std::ostringstream sName;
  std::string sModel("model1.vtk");
  int iTimestep=*iout;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sModel.append(sName.str());
  C3DModel model_out(myWorld.m_vParticles[0]);
  model_out.m_vMeshes[0].TransformModelWorld();
  CVtkWriter writer;
  //Write the grid to a file and measure the time
  std::cout<<"Writing vtk file:... "<<std::endl;
  writer.WriteModel(model_out,sModel.c_str());
  
  std::ostringstream sName2;
  std::string sModel2("model2.vtk");
  sName2<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sModel2.append(sName2.str());
  C3DModel model_out2(myWorld.m_vParticles[1]);
  model_out2.m_vMeshes[0].TransformModelWorld();
  //Write the grid to a file and measure the time
  std::cout<<"Writing vtk file:... "<<std::endl;
  writer.WriteModel(model_out2,sModel2.c_str());
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin)
{

  CDistOps3 op;
  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  CVector3<Real> vec(x,y,z);
  int in=0;
  if(myWorld.m_vRigidBodies[id]->IsInBody(vec))
  {
    in=1;
  }
  *isin=in;

}//end isinelement

//-------------------------------------------------------------------------------------------------------

void setelement(int* iel, int* iID)
{
  int i = *iID;
  myWorld.m_vRigidBodies[i]->m_iElement = *iel;
  myWorld.m_vRigidBodies[i]->m_iElementsPrev = 1;
}

//-------------------------------------------------------------------------------------------------------

void getelement(int* iel, int* iID)
{
  int i = *iID;
  *iel = myWorld.m_vRigidBodies[i]->m_iElement;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  int id = *iID;
  CVector3<Real> vNewPos(x,y,z);
  myWorld.m_vRigidBodies[id]->TranslateTo(vNewPos);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;
  CVector3<Real> vNewRot(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vAngle = vNewRot;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;  
  CVector3<Real> vNewVel(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vVelocity = vNewVel;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  i3d::Real x = *dangvelx;
  i3d::Real y = *dangvely;
  i3d::Real z = *dangvelz;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewAngVel(x,y,z);
  myWorld.m_vRigidBodies[id]->GetAngVel() = vNewAngVel;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setforce(double *dforcex,double *dforcey,double *dforcez,int *iid)
{
  i3d::Real x = *dforcex;
  i3d::Real y = *dforcey;
  i3d::Real z = *dforcez;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewForce(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vForce = vNewForce;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settorque(double *dtorquex,double *dtorquey,double *dtorquez,int *iid)
{
  i3d::Real x = *dtorquex;
  i3d::Real y = *dtorquey;
  i3d::Real z = *dtorquez;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewTorque(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vTorque = vNewTorque;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getresponse(double *dux,double *duy,double *duz,
                            double *dcorrx,double *dcorry,double *dcorrz,
                            double *domx,double *domy,double *domz,int *iID)
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getpos(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID;
  CVector3<double> pos(myWorld.m_vRigidBodies[i]->m_vCOM.x,myWorld.m_vRigidBodies[i]->m_vCOM.y,myWorld.m_vRigidBodies[i]->m_vCOM.z);
  *dx=pos.x;
  *dy=pos.y;
  *dz=pos.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getvel(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID; 
  CVector3<double> vel(myWorld.m_vRigidBodies[i]->m_vVelocity.x,myWorld.m_vRigidBodies[i]->m_vVelocity.y,myWorld.m_vRigidBodies[i]->m_vVelocity.z);
  *dx=vel.x;
  *dy=vel.y;
  *dz=vel.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getnumparticles(int *nParts)
{
  
  *nParts=myWorld.m_vRigidBodies.size();
}

extern "C" void getradius(double *drad, int *iid)
{
  int i = *iid;
  CAABB3r box = myWorld.m_vRigidBodies[i]->m_pShape->GetAABB();
  double rad  = box.m_Extends[0];
  *drad = rad;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid)
{
  int i = *iid; 
  CVector3<double> angle(myWorld.m_vRigidBodies[i]->m_vAngle.x,myWorld.m_vRigidBodies[i]->m_vAngle.y,myWorld.m_vRigidBodies[i]->m_vAngle.z);
  *dangx=angle.x;
  *dangy=angle.y;
  *dangz=angle.z;
}

extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  int i = *iid;
  CVector3<double> angvel(myWorld.m_vRigidBodies[i]->GetAngVel().x,myWorld.m_vRigidBodies[i]->GetAngVel().y,myWorld.m_vRigidBodies[i]->GetAngVel().z);
  *dangvelx=angvel.x;
  *dangvely=angvel.y;
  *dangvelz=angvel.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdensity(double *ddens, int *iid)
{
  int i = *iid;
  *ddens=myWorld.m_vRigidBodies[i]->m_dDensity;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settimestep(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetDeltaT(dT);
  myTimeControl.SetPreferredTimeStep(dT);
  //myPipeline.m_pTimeControl->SetDeltaT(dT);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settime(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetTime(dT);

}

//-------------------------------------------------------------------------------------------------------

void addboundary()
{
  //initialize the box shaped boundary
  myWorld.m_vRigidBodies.push_back(new CRigidBody());
  CRigidBody *body = myWorld.m_vRigidBodies.back();
  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::BOUNDARYBOX;
  CBoundaryBoxr *box = new CBoundaryBoxr();
  box->rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  box->CalcValues();
  body->m_vCOM      = box->rBox.GetCenter();
  body->m_pShape      = box;
  body->m_InvInertiaTensor.SetZero();
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
}

//-------------------------------------------------------------------------------------------------------

void cleanup()
{
  std::vector<CRigidBody*>::iterator vIter;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    delete body;
  }
}

//-------------------------------------------------------------------------------------------------------

void initphysicalparameters()
{

  std::vector<CRigidBody*>::iterator vIter;

  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    body->m_dDensity    = myParameters.m_dDefaultDensity;
    body->m_dVolume     = body->m_pShape->Volume();
    Real dmass          = body->m_dDensity * body->m_dVolume;
    body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
    body->m_vAngle      = VECTOR3(0,0,0);
    body->SetAngVel(VECTOR3(0,0,0));
    body->m_vVelocity   = VECTOR3(0,0,0);
    body->m_vCOM        = VECTOR3(0,0,0);
    body->m_vForce      = VECTOR3(0,0,0);
    body->m_vTorque     = VECTOR3(0,0,0);
    body->m_Restitution = 0.0;
    body->SetOrientation(body->m_vAngle);
    body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());

    //calculate the inertia tensor
    //Get the inertia tensor
    body->GenerateInvInertiaTensor();
  }

}

//-------------------------------------------------------------------------------------------------------

void cupdynamics()
{
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceMesh("meshes/cup_small_high2.obj");
  Real extentBox[3]={0.25, 0.25, 0.025};
  myFactory.AddBoxes(myWorld.m_vRigidBodies,1,extentBox);
  myFactory.AddSpheres(myWorld.m_vRigidBodies,20,myParameters.m_dDefaultRadius);

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.m_vRigidBodies[0]->TranslateTo(VECTOR3(0.49,0.25,0.378));
  myWorld.m_vRigidBodies[1]->TranslateTo(VECTOR3(0.75, 0.25, 0.28));
  myWorld.m_vRigidBodies[1]->m_bAffectedByGravity=false;
  myWorld.m_vRigidBodies[1]->m_dInvMass=0;
  myWorld.m_vRigidBodies[1]->m_InvInertiaTensor.SetZero();
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.m_vRigidBodies[0]->m_pShape);

  C3DModel model_out(pMeshObject->m_Model);
  model_out.m_vMeshes[0].m_matTransform =myWorld.m_vRigidBodies[0]->GetTransformationMatrix();
  model_out.m_vMeshes[0].m_vOrigin =myWorld.m_vRigidBodies[0]->m_vCOM;
  model_out.m_vMeshes[0].TransformModelWorld();
  model_out.GenerateBoundingBox();
  model_out.m_vMeshes[0].GenerateBoundingBox();
  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();
  
  int offset = 2;
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = 7;//myGrid.m_vMax.x/(distbetween+d);
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  Real xstart=myGrid.m_vMin.x + (myGrid.m_vMax.x/2.5) - (drad+distbetween);
  Real ystart=myGrid.m_vMin.y+drad+distbetween+myGrid.m_vMax.y/3.0;  
  VECTOR3 pos(xstart , ystart, (myGrid.m_vMax.z/1.75)-d);
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
  myWorld.m_vRigidBodies[offset]->TranslateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.0015;
  int count=0;
  for(int i=offset+1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = xstart;
      pos.y += d+distbetween;
      if(even)
      {
        ynoise = -ynoise;
      }
      if(++count==6)
      {
        pos.z -= d+distbetween;
        pos.y=ystart;
        count=0;
      }
    }
    VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
    pos.x+=d+distbetween;
    ynoise = -ynoise;
  }  
  
}

//-------------------------------------------------------------------------------------------------------

void createlineuptest()
{
  Real drad = radius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = 1.0/(distbetween+d);
  VECTOR3 pos(-0.5+drad+distbetween, 0.0, 3.0);
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  distbetween = 0.5 * drad;
  pos.x+=d+distbetween;
  distbetween = 0.1 * drad;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    std::cout<<"position: "<<pos<<std::endl;    
    if((i+1)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = -0.5+drad+distbetween;
      pos.z -= d+distbetween;
    }
    myWorld.m_vRigidBodies[i]->TranslateTo(pos);
    pos.x+=d+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

void createstackingtest()
{
  
}

//-------------------------------------------------------------------------------------------------------

void pyramidtest()
{
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceBoxes(myParameters.m_iBodies, extends);
  
  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  Real drad = extends[0];
  Real d    = 2.0 * drad;
  Real distbetween = drad * 0.05;
  Real delta = d+distbetween;

  Real ystart = 0.25;
  VECTOR3 pos(0.75, ystart, (1.0/3.0));
  int index = 0;
  for(int i=0;i<4;i++)
  {
    pos.y=ystart+Real(i)* (drad+distbetween/2.0);
    for(int j=i;j<4;j++)
    {
      myWorld.m_vRigidBodies[index]->TranslateTo(pos);
      pos.y+=delta;
      index++;
    }
    pos.z+=delta;
  }

  myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(1.15,pos.y-delta,pos.z-2.5*delta));
  //myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,0.75,0);
  myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void createrestingtest()
{
  
  CParticleFactory myFactory;
  
  myWorld = myFactory.ProduceSpheres(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.5 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMin.z)+drad);
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  pos.z+=d;//+distbetween;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.z += d;
    }
    myWorld.m_vRigidBodies[i]->TranslateTo(pos);
    pos.z+=d;//+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void createbasf()
{

  CParticleFactory myFactory;
       myWorld = myFactory.ProduceTubes("meshes/myAllClumps.obj");
}

//-------------------------------------------------------------------------------------------------------

void addmesh()
{
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceMesh("meshes/cup_small_high2.obj");

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.m_vRigidBodies[0]->TranslateTo(VECTOR3(0.25,0.25,0.8));
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.m_vRigidBodies[0]->m_pShape);

  C3DModel model_out(pMeshObject->m_Model);
  model_out.m_vMeshes[0].m_matTransform =myWorld.m_vRigidBodies[0]->GetTransformationMatrix();
  model_out.m_vMeshes[0].m_vOrigin =myWorld.m_vRigidBodies[0]->m_vCOM;
  model_out.m_vMeshes[0].TransformModelWorld();
  model_out.GenerateBoundingBox();
  model_out.m_vMeshes[0].GenerateBoundingBox();
  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);

  //myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,0.75,0);
  //myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void addsphere_dt(int *itime)
{
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  //VECTOR3 pos(myGrid.m_vMin.x+1.0*d, myGrid.m_vMax.y/2.0, myGrid.m_vMax.z/2.0);
  std::vector<VECTOR3> vPos;
  VECTOR3 pos(0.0,0.0,7.75);
  vPos.push_back(pos);
  pos = VECTOR3(0.17,0.0,7.75);
  vPos.push_back(pos);
  pos = VECTOR3(-0.17,0.0,7.75);
  vPos.push_back(pos);  
  pos = VECTOR3(0.,0.17,7.75);
  vPos.push_back(pos);  
  pos = VECTOR3(0.,-0.17,7.75);    
  vPos.push_back(pos);  

  
  int iadd = 5;
  int iStart = *itime;
  int iSeed = 1;
  
//   if(iStart == 1)
//     pos.y = -0.026 + drad + distbetween;
//   else if(iStart == 2)
//     pos.y = 0.026 - (drad + distbetween);
  
  
  Real noise = 0.0005;
  
  if(myWorld.m_vRigidBodies.size() < 1000)
  {
    CParticleFactory myFactory;

    int offset = myWorld.m_vRigidBodies.size();

    Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};

    myFactory.AddSpheres(myWorld.m_vRigidBodies,iadd,myParameters.m_dDefaultRadius);
    
    for(int i=0;i<iadd;i++)
    {
      CRigidBody *body    = myWorld.m_vRigidBodies[offset+i];
      body->m_dDensity    = myParameters.m_dDefaultDensity;
      body->m_dVolume     = body->m_pShape->Volume();
      Real dmass          = body->m_dDensity * body->m_dVolume;
      body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
      body->m_vAngle      = VECTOR3(0,0,0);
      body->SetAngVel(VECTOR3(0,0,0));
      body->m_vVelocity   = VECTOR3(0,0,-1.05);
      body->m_vCOM        = VECTOR3(0,0,0);
      body->m_vForce      = VECTOR3(0,0,0);
      body->m_vTorque     = VECTOR3(0,0,0);
      body->m_Restitution = 0.0;
      body->SetOrientation(body->m_vAngle);
      body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
      
      //calculate the inertia tensor
      //Get the inertia tensor
      body->GenerateInvInertiaTensor();
      pos = vPos[i];
      body->TranslateTo(pos);      
    }
  }//end if

  myPipeline.m_pGraph->Clear();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
    myWorld.m_vRigidBodies[j]->m_iID = j;

  std::cout<<"Added body, number of particles: "<<myWorld.m_vRigidBodies.size()<<std::endl;

}

//-------------------------------------------------------------------------------------------------------

void addobstacle()
{

  CObjLoader Loader;

  CRigidBody *body = new CRigidBody();
  CMeshObject<Real> *pMeshObject= new CMeshObject<Real>();

  Loader.ReadMultiMeshFromFile(&pMeshObject->m_Model,"meshes/fritten_final_mili.obj");

  pMeshObject->m_Model.GenerateBoundingBox();

  pMeshObject->SetFileName("meshes/fritten_final_mili.obj");

  body->m_pShape = pMeshObject;
  body->m_iShape = CRigidBody::MESH;
  myWorld.m_vRigidBodies.push_back(body);

  //initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(3.14,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::MESH;

  body->m_vCOM      = VECTOR3(0,0,0);

  body->m_InvInertiaTensor.SetZero();

  body->m_Restitution = 0.0;

  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  body->TranslateTo(VECTOR3(0.13,0.2125,0.0155));

  C3DModel model_out(pMeshObject->m_Model);
  model_out.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    model_out.m_vMeshes[i].m_matTransform =body->GetTransformationMatrix();
    model_out.m_vMeshes[i].m_vOrigin =body->m_vCOM;
    model_out.m_vMeshes[i].TransformModelWorld();
    model_out.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();

}

//-------------------------------------------------------------------------------------------------------

void reactor()
{
  CParticleFactory myFactory;
  int offset = myWorld.m_vRigidBodies.size();
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};

  myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
  
  CRigidBody *body    = myWorld.m_vRigidBodies.back();
  body->m_dDensity    = myParameters.m_dDefaultDensity;
  body->m_dVolume     = body->m_pShape->Volume();
  Real dmass          = body->m_dDensity * body->m_dVolume;
  body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
  body->m_vAngle      = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity   = VECTOR3(0,0,0);
  body->m_vCOM        = VECTOR3(0,0,0);
  body->m_vForce      = VECTOR3(0,0,0);
  body->m_vTorque     = VECTOR3(0,0,0);
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  //VECTOR3 pos(0.0+distbetween+drad, 0, 0);
  
  //VECTOR3 pos(VECTOR3(0.125,0.0,0.0));  
  VECTOR3 pos(0.25,0.3333,0.75);  
  
  body->TranslateTo(pos);
  
  body->m_vVelocity=VECTOR3(0.0,0.0,0);  
  
  //addobstacle();
}

//-------------------------------------------------------------------------------------------------------

void spherestack()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.25 * drad;
  Real distbetweenz = 0.5 * drad;
  int perrowx = myGrid.m_vMax.x/(distbetween+d);
  int perrowy = myGrid.m_vMax.y/(distbetween+d);  
  
  int numPerLayer = perrowx * perrowy;
  int layers =1;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();
  
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+0.0025, (myGrid.m_vMax.z-drad));
  
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
  Real ynoise = 0.0025;
  int count=0;
    
  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myGrid.m_vMin.x+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z-=d;
    pos.y=myGrid.m_vMin.y+drad+distbetween+0.0025;        
  }

}

//-------------------------------------------------------------------------------------------------------

void drivcav()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real myxmin = 2.0;  
  Real myymin = 0.0;  
  Real myzmin = 0.0;  

  Real myxmax = 6.0;  
  Real myymax = 2.0;  
  Real myzmax = 1.0;  


  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 1.0 * drad;
  Real distbetweenz = 0.5 * drad;

  Real extendX = myxmax - myxmin;  
  Real extendY = myymax - myymin;  
  Real extendZ = myzmax - myzmin;  

  int perrowx = extendX/(distbetween+d);
  int perrowy = extendY/(distbetween+d);  
  
  int numPerLayer = perrowx * perrowy;
  int layers = 4;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();
  
  VECTOR3 pos(myxmin+drad+distbetween , myymin+drad+distbetween+0.0025, (myzmin+drad));
  
  Real ynoise = 0.0025;
  int count=0;
    
  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myxmin+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myymin+drad+distbetween+0.0025;        
  }

}

//-------------------------------------------------------------------------------------------------------

void sphericalstack()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.1 * drad;
  int perrowx = 1.0/(distbetween+d);
  int perrowy = perrowx-1;
  Real z=7.7;
  Real x=-0.5+drad+distbetween;
  Real y=-0.5+drad+distbetween;
  int layers = 50;
  std::vector<VECTOR3> vPos;

  for(int layer=0;layer<layers;layer++)
  {
    double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand()/(double)RAND_MAX);
    // make an x-row and rotate
    for(int i=0;i<perrowx;i++)
    {
      VECTOR3 pos(x,0,0);
      MATRIX3X3 rotmat;
      rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
      pos = rotmat * pos;
      pos.z=z;
      //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
      vPos.push_back(pos);
      x+=d+distbetween;
    }
    //yrow
    for(int i=0;i<perrowx-1;i++)
    {
      VECTOR3 pos(0,y,0);
      MATRIX3X3 rotmat;
      rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
      pos = rotmat * pos;
      pos.z=z;
      //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
      vPos.push_back(pos);
      if(i==3)
        y=(d+distbetween);
      else
        y+=d+distbetween;
    }
    y=-0.5+drad+distbetween;
    x=-0.5+drad+distbetween;
    z-=d+1.5*distbetween;
  }

  int numPerLayer = 2*perrowx - 1;
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();

  std::vector<CRigidBody*>::iterator vIter;
  std::vector<VECTOR3>::iterator i;

  for(vIter=myWorld.m_vRigidBodies.begin(),i=vPos.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++,i++)
  {
    CRigidBody *body    = *vIter;
    VECTOR3 pos         = *i;
    body->TranslateTo(pos);
  }

}

//-------------------------------------------------------------------------------------------------------

void initrigidbodies()
{
  CParticleFactory myFactory;

  if(myParameters.m_iBodyInit == 0)
  {
    myWorld = myFactory.ProduceFromParameters(myParameters);
  }
  else if(myParameters.m_iBodyInit == 1)
  {
    myWorld = myFactory.ProduceFromFile(myParameters.m_sBodyFile.c_str(),myTimeControl);
  }
  else
  {
    if(myParameters.m_iBodyInit == 2)
    {
      createstackingtest();
    }

    if(myParameters.m_iBodyInit == 3)
    {
      createlineuptest();
    }

    if(myParameters.m_iBodyInit == 4)
    {
      reactor();
    }
    if(myParameters.m_iBodyInit == 5)
    {
      createbasf();
    }
    
    if(myParameters.m_iBodyInit == 6)
    {
      spherestack();
    }
    if(myParameters.m_iBodyInit == 7)
    {
      drivcav();
    }
  }

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();

  //add the boundary as a rigid body
  addboundary();
  
}

void initsimulation()
{

  //first of all initialize the rigid bodies
  initrigidbodies();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
  {
    myWorld.m_vRigidBodies[j]->m_iID = j;
    myWorld.m_vRigidBodies[j]->m_iElementsPrev = 0;
  }

  //set the timestep
  myTimeControl.SetDeltaT(myParameters.m_dTimeStep);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

  //link the boundary to the world
  myWorld.SetBoundary(&myBoundary);

  //set the time control
  myWorld.SetTimeControl(&myTimeControl);

  //set the gravity
  myWorld.SetGravity(myParameters.m_vGrav);

  //Set the collision epsilon
  myPipeline.SetEPS(0.02);

  //initialize the collision pipeline 
  myPipeline.Init(&myWorld,myParameters.m_iSolverType,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  if(myParameters.m_iSolverType==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CMotionIntegratorSI(&myWorld);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CRigidBodyMotion(&myWorld);
  }

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

}

void continuesimulation()
{
  
  CParticleFactory myFactory;

  //Produces a domain
  //it is a bit unsafe, because the domain at this point is
  //not fully useable, because of non initialized values in it
  //string = ssolution
  myWorld = myFactory.ProduceFromFile(myParameters.m_sSolution.c_str(),myTimeControl);

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();
  
  //set the timestep
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myParameters.m_iTotalTimesteps+=myTimeControl.GetTimeStep();

  //link the boundary to the world
  myWorld.SetBoundary(&myBoundary);

  //set the time control
  myWorld.SetTimeControl(&myTimeControl);

  //set the gravity
  myWorld.SetGravity(myParameters.m_vGrav);

  //Set the collision epsilon
  myPipeline.SetEPS(0.02);

  //initialize the collision pipeline 
  myPipeline.Init(&myWorld,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion = new CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

  CRigidBody *body    = myWorld.m_vRigidBodies[4];
  //body->m_InvInertiaTensor.SetZero();
  body->SetAngVel(VECTOR3(0,0,0));

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
  {
    myWorld.m_vRigidBodies[j]->m_iID = j;
    myWorld.m_vRigidBodies[j]->m_iElementsPrev = 0;
  }

}

void writetimestep(int iout)
{
  std::ostringstream sName,sNameParticles,sContacts;
  std::string sModel("output/model.vtk");
  std::string sParticle("solution/particles.i3d");
  CVtkWriter writer;
  int iTimestep=iout;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sNameParticles<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sModel.append(sName.str());
  sParticle.append(sNameParticles.str());
  sContacts<<"output/contacts.vtk."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());
  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
  std::vector<int> indices;
  //indices.push_back(8);
  //indices.push_back(10);
  //indices.push_back(11);
  //rbwriter.Write(myWorld,indices,sParticle.c_str());
  //rbwriter.Write(myWorld,sParticle.c_str());
  //writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

  // if(iout==0)
  // {
  //   std::ostringstream sNameGrid;
  //   std::string sGrid("output/grid.vtk");
  //   sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  //   sGrid.append(sNameGrid.str());
  //   writer.WriteUnstr(myGrid,sGrid.c_str());
  // }
}

extern "C" void fallingparticles()
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  CReader reader;
  std::string meshFile;

  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.InitMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.InitCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.m_iStartType == 0)
  {
    initsimulation();
  }
  else
  {
    continuesimulation();
  }
  
}

extern "C" void initdeform()
{

  CReader reader;  
  CParticleFactory factory;

  //read the user defined configuration file
  reader.ReadParametersDeform(std::string("start/data.TXT"),myDeformParameters);  
  
  myWorld = factory.ProduceFromDeformParameters(myDeformParameters);  
  
}

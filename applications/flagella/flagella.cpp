#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>
#include <mymath.h>

namespace i3d {

  template<>
  class SoftBody4<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
  {
    public:

    int istep_;

    Real A_;

    Real q_;

    Real N_;

    Real f_;

    Real m_;

    Real k_tail_;

    Real kd_;

    Real ks_;

    Real a0_;

    Real l0_;

    Real dt_;

    Real rho_;

    Real massAll_;

    Real volume_;

    int n_mid_;

    int n_head_;

    int offset_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    SoftBody4() : A_(3.2), N_(100),  f_(1.0/60.0), a0_(0.5), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0,0,0));
      geom_.center_ = Vec3(0,0,0);

      geom_.vertices_.reserve(N_);
      u_.reserve(N_);
      force_.reserve(N_);

      velocity_ = Vec3(0,0,0);
    };

    virtual ~SoftBody4(){};

    void calcCOM()
    {
      com_ = Vec3(0,0,0);
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        com_ += geom_.vertices_[i]; 
      }
      com_ *= (1.0/geom_.vertices_.size());
    }

    Vec3 getCOM()
    {
      calcCOM();
      Vec3 transCom = com_ + transform_.getOrigin();
      return transCom; 
    }

    void setOrigin(const Vec3 &origin)
    {
      transform_.setOrigin(origin);
    }

    void calcVolume() 
    {
      rho_ = 1.1;  
      Real vc = 4.0/3.0 * CMath<Real>::SYS_PI * std::pow(0.5,3.0);
      volume_ = 0.5 * geom_.vertices_.size() * vc;
      Real vc2 = 4.0/3.0 * CMath<Real>::SYS_PI * std::pow(5.0,3.0);
      volume_ += vc2;
      massAll_ = volume_ * rho_;
    }

    bool isInBody(const Vec3 &vQuery) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin(); 
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        if((geom_.vertices_[i] - q).mag() < 0.5)
        {
//          std::cout << "q: " << q << "v: " << geom_.vertices_[i] << "v-q: " << geom_.vertices_[i] - q << std::endl;
          return true;
        }
      }
      return false;
    }

    int isInBodyID(const Vec3 &vQuery) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin(); 
      int imin = 0;
      Real dmin = (geom_.vertices_[0] - q).mag(); 
      for (int i(1); i < geom_.vertices_.size(); ++i)
      {
        Real mmin = (geom_.vertices_[i] - q).mag();
        if(mmin < dmin)
        {
          dmin = mmin; 
          imin = i;
        }
      }
      if((geom_.vertices_[imin] - q).mag() < 0.5)
      {
        return imin+1;
      }
      int last = geom_.vertices_.size();
      if((geom_.vertices_[last-1] - q).mag() < 5.0)
        return (last-1);

      return 0;
    }

    Vec3 getVelocity(const Vec3 &vQuery, Real t)
    {
      Vec3 q = vQuery - transform_.getOrigin(); 
      int imin = 0;
      Real dmin = (geom_.vertices_[0] - q).mag(); 
      for (int i(1); i < geom_.vertices_.size(); ++i)
      {
        Real mmin = (geom_.vertices_[i] - q).mag();
        if(mmin < dmin)
        {
          dmin = mmin; 
          imin = i;
        }
      }

      Vec3 velocity = (1.0/dt_) * (geom_.vertices_[imin] - geom_.verticesOld_[imin]);
      Real vx = velocity_.x;
      return Vec3(vx,velocity.y,0);
    }

    void storeVertices()
    {
      for(int i(0); i < geom_.vertices_.size(); ++i)
      {
        geom_.verticesOld_[i] = geom_.vertices_[i];
      }
    }

    void internalForce(Real t)
    {

      //int ilast = geom_.vertices_.size()-1;
      //force_[ilast].z = 1.0;


      Real A = 1.0;
      Real f = 1.0 / 240.0;
      Real phi = 0 * CMath<Real>::SYS_PI;
      Real sign = 1.0;

      if (t < 6.0)
      {
        for (int j(0); j < geom_.vertices_.size(); ++j)
        {
          Real t = geom_.vertices_[j].x;
          Real fz = A * std::sin(2.0 * CMath<Real>::SYS_PI * f * t + phi);
          force_[j].z = sign * fz;
        }
      }

      if (t > 6.0)
      {
        sign *= -1.0;
        for (int j(0); j < geom_.vertices_.size(); ++j)
        {
          phi = 0.0 * CMath<Real>::SYS_PI;
          Real t = geom_.vertices_[j].x;
          Real fz = A * std::sin(2.0 * CMath<Real>::SYS_PI * f * t + phi);
          force_[j].z = sign * fz;
        }
      }


      // evaluate force for the first 14 segments
      for(unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

        force_[spring.i0_] += f;
        force_[spring.i1_] -= f;

//        if(i==98)
//        {
//          std::cout << "ks: " << spring.ks_ << std::endl;
//          std::cout << "kd: " << spring.kd_ << std::endl;
//          std::cout << "x0: " << *spring.x0_ << std::endl;
//          std::cout << "x1: " << *spring.x1_ << std::endl;
//        }
//        if(i==60)
//        {
//          std::cout << "60: " << std::endl;
//          std::cout << "ks: " << spring.ks_ << std::endl;
//          std::cout << "kd: " << spring.kd_ << std::endl;
//          std::cout << "x0: " << *spring.x0_ << std::endl;
//          std::cout << "x1: " << *spring.x1_ << std::endl;
//          std::cout << "v0: " << *spring.v0_ << std::endl;
//          std::cout << "v1: " << *spring.v1_ << std::endl;
//        }
      }

    }; 

    void applyForce(Real dt)
    {

    }; 

    void step(Real t, Real dt)
    {
      dt_ = dt;
      internalForce(t); 
      
      std::cout << "Time: " << t << "|-----------------------|" << dt_ << "|l0: " << l0_<< std::endl;
      integrate();
    }

    void init()
    {


      a0_ = 0.5;
      Real xx = 0 * l0_;
      Real yy = 0;

      ks_ = 8.0;
      kd_ = -0.2;

      geom_.vertices_.push_back(Vector3<Real>(xx,
                                              yy,
                                              0));
      u_.push_back(Vec3(0,0,0));

      force_.push_back(Vec3(0,0,0));

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(k) * l0_;

        Real y = 0;

        geom_.vertices_.push_back(Vector3<Real>(x,y,0));

        geom_.faces_.push_back(std::pair<int,int>(k-1,k));

        geom_.segments_.push_back(Segment3<Real>(geom_.vertices_[k-1], 
                                                 geom_.vertices_[k]
                                                ));

        u_.push_back(Vec3(0,0,0));
        force_.push_back(Vec3(0,0,0));

        springs_.push_back(SimpleSpringConstraint<Real>(ks_, kd_, l0_,k-1,k,
                                                       &geom_.vertices_[k-1],
                                                       &geom_.vertices_[k],
                                                       &u_[k-1],
                                                       &u_[k]
                                                       ));

      }

      // evaluate force for the first 14 segments
      for(unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];
      }

      for(auto &v: geom_.vertices_)
      {
        geom_.verticesOld_.push_back(v);
      }

    }; 

    void integrate()
    {

      std::vector<Vector3<Real>> &u0 = u_; 
      std::vector<Vector3<Real>> &f0 = force_; 

      // Integrate the mid-piece vertices
      for(int i(4); i < N_; ++i)
      {
        Vec3 &vel = u0[i];

        Vec3 &force = f0[i];

        //force -= 0.0125 * vel;

        Real m = 1.0;

        Vec3 &pos = geom_.vertices_[i];
//        if(i == 99 || i == 61 || i == 60)
//        {
//          std::cout << "force_" << i-1 << " " << f0[i-1] << std::endl;
//          std::cout << "force_" << i << " " << f0[i] << std::endl;
//        }
      
        vel = vel + dt_ * force * (1.0/m);

//        if(i == 99 || i == 61 || i == 60)
//        {
//          std::cout << "vel_" << i-1 << " "  << u0[i-1] << std::endl;
//          std::cout << "vel_" << i << " " << u0[i] << std::endl;
//        }

        pos = pos + dt_ * vel;

        force = Vec3(0,0,0);

//        if(i==0)
//          vel = 0.98 * vel;
      }

    }; 

  };

  class Flagella : public Application {

  public:

    SoftBody4<Real, ParamLine<Real>> flagella_;

    int steps_;

    Real dt_;
    Real time_;
    int step_;

    Flagella() : Application() {

    }

    void init(std::string fileName) {

      using namespace std;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(fileName.c_str());
      }
      else
      {
        if (dataFileParams_.hasExtents_)
        {
          grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
            dataFileParams_.extents_[4], dataFileParams_.extents_[1],
            dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
        }
        else
          grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
      }
      
    }

    void run()
    {
      flagella_.init();
      flagella_.istep_ = 0;
      steps_ = 4000;

      dt_ = 0.01;
      time_ = 0.0;
      step_ = 0;
      //grid_.calcVol();
      //std::cout << "> Grid volume: " << grid_.vol_ << std::endl;

      for(int istep(0); istep <= steps_; ++istep)
      {
        flagella_.step(time_,dt_);
        CVtkWriter writer;
        std::ostringstream name;
        name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        writer.WriteParamLine(flagella_.geom_, name.str().c_str());
        time_ += dt_;
        step_++;
        flagella_.istep_ = step_;
      }
    }
  };
}

using namespace i3d;

int main()
{
  Flagella myApp;
  myApp.init("start/sampleRigidBody.xml");
  myApp.run();
  
  return 0;
}

// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file ArmsauvHydrodynamicModel.hh
/// \brief This file contains the definition for different classes of
/// hydrodynamic models for submerged objects

#ifndef __UUV_GAZEBO_HYDRO_MODEL_HH__
#define __UUV_GAZEBO_HYDRO_MODEL_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <string>
#include <vector>
#include <map>

#include <uuv_gazebo_plugins/Def.hh>
#include <uuv_gazebo_plugins/BuoyantObject.hh>


namespace gazebo
{
class ArmsauvHydrodynamicModel : public BuoyantObject
{
  /// \brief Protected constructor: Use the factory for object creation.
  protected: ArmsauvHydrodynamicModel(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Returns type of model
  public: virtual std::string GetType() = 0;

  /// \brief Computation of the hydrodynamic forces
  public: virtual void ApplyHydrodynamicForces(
    double time, const ignition::math::Vector3d &_flowVelWorld,
    double rudderb, double rudders, double rudderr, double rpm) = 0;

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
    std::string _message = std::string()) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    std::vector<double>& _output) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    double& _output) = 0;

  /// \brief Set a scalar parameters
  public: virtual bool SetParam(std::string _tag, double _input) = 0;

  /*
  /// \brief Set front rudder angle                                                            
  public: virtual void SetFrontRudderAngle(double rudderb) = 0;
  
  /// \brief Set back rudder angle
  public: virtual void SetBackRudderAngle(double rudders) = 0;;
  
  /// \brief Set vertical rudder angle
  public: virtual void SetVertRudderAngle(double rudderr) = 0;

  /// \brief Set rotor speed
  public: virtual void SetRotorSpeed(double rpm) = 0;
  */

  /// \brief Filter acceleration (fix due to the update structure of Gazebo)
  protected: void ComputeAcc(Eigen::Vector6d _velRel,
                            double _time,
                            double _alpha = 0.3);

  /// \brief Filter acceleration (fix due to the update structure of Gazebo)
  protected: void ComputeAccNoFilter(Eigen::Vector6d _velRel,
                            double _time);
 
  /// \brief Returns true if all parameters are available from the SDF element
  protected: bool CheckParams(sdf::ElementPtr _sdf);

  /// \brief Convert vector to comply with the NED reference frame
  protected: ignition::math::Vector3d ToNED(ignition::math::Vector3d _vec);

  /// \brief Convert vector to comply with the NED reference frame
  protected: ignition::math::Vector3d FromNED(ignition::math::Vector3d _vec);

  /// \brief Filtered linear & angular acceleration vector in link frame.
  /// This is used to prevent the model to become unstable given that Gazebo
  /// only calls the update function at the beginning or at the end of a
  /// iteration of the physics engine
  protected: Eigen::Vector6d filteredAcc;

  /// \brief Last timestamp (in seconds) at which ApplyHydrodynamicForces was
  /// called
  protected: double lastTime;

  /// \brief Last body-fixed relative velocity (nu_R in Fossen's equations)
  protected: Eigen::Vector6d lastVelRel;

  /// \brief List of parameters needed from the SDF element
  protected: std::vector<std::string> params;

  /// \brief Reynolds number (not used by all models)
  protected: double Re;

  /// \brief Temperature (not used by all models)
  protected: double temperature;
};

/// \brief Pointer to model
typedef boost::shared_ptr<ArmsauvHydrodynamicModel> ArmsauvHydrodynamicModelPtr;

/// \brief Function pointer to create a certain a model
typedef ArmsauvHydrodynamicModel* (*ArmsauvHydrodynamicModelCreator)(sdf::ElementPtr, \
                                                       physics::LinkPtr);

/// \brief Factory singleton class that creates a ArmsauvHydrodynamicModel from sdf.
class ArmsauvHydrodynamicModelFactory
{
  /// \brief Create ArmsauvHydrodynamicModel object according to its sdf Description.
  public: ArmsauvHydrodynamicModel* CreateArmsauvHydrodynamicModel(sdf::ElementPtr _sdf,
                                                     physics::LinkPtr _link);

  /// \brief Returns the singleton instance of this factory.
  public: static ArmsauvHydrodynamicModelFactory& GetInstance();

  /// \brief Register a class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               ArmsauvHydrodynamicModelCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: ArmsauvHydrodynamicModelFactory() {}

  /// \brief Map of each registered identifier to its corresponding creator.
  private: std::map<std::string, ArmsauvHydrodynamicModelCreator> creators_;
};

/// Use the following macro within a ArmsauvHydrodynamicModel declaration:
#define REGISTER_ArmsauvHydrodynamicModel(type) static const bool registeredWithFactory

/// Use the following macro before a ArmsauvHydrodynamicModel's definition:
#define REGISTER_ArmsauvHydrodynamicModel_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  ArmsauvHydrodynamicModelFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containting the methods and attributes
/// for a Fossen robot-like hydrodynamic model. The restoring
/// forces are applied by the BuoyantObject class methods. Using the
/// plugin for UUV models will use both this and the buoyant object
/// class definitions, therefore the restoring forces were not
/// inherited here.
/// References:
///     - Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
///       Control", 2011
class ArmsauvFossen : public ArmsauvHydrodynamicModel
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static ArmsauvHydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
                                std::vector<double>& _output);

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Set scalar parameter
  public: virtual bool SetParam(std::string _tag, double _input);

  /// \brief Set front rudder angle
  // public: void SetFrontRudderAngle(double rudderb);

  /// \brief Set back rudder angle
  // public: void SetBackRudderAngle(double rudders);

  /// \brief Set vertical rudder angle
  // public: void SetVertRudderAngle(double rudderr);

  /// \brief Set rotor speed
  // public: void SetRotorSpeed(double rpm);
  
  /// \brief Register this model with the factory.
  protected: REGISTER_ArmsauvHydrodynamicModel(ArmsauvFossen);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: ArmsauvFossen(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Computation of the hydrodynamic forces
  public: virtual void ApplyHydrodynamicForces(double time,
                            const ignition::math::Vector3d &_flowVelWorld,
                            double _rouderb, double _rouders, double _rouderr, double _rpm);

  /// \brief Computes the added-mass Coriolis matrix Ca.
  protected: void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                             const Eigen::Matrix6d& _Ma,
                                             Eigen::Matrix6d &_Ca) const;

  /// \brief Updates the damping matrix for the current velocity
  protected: void ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                       Eigen::Matrix6d &_D) const;

  /// \brief Updates the quadratic damping matrix for the current velocity
  protected: void ComputeQuadDampingMatrix(const Eigen::Vector6d& _vel,
                                       Eigen::Matrix6d &_D) const;
  
  /// \brief Updates the quadratic damping matrix for the current velocity
  protected: void ComputeLinForwardSpeedDampingMatrix(const Eigen::Vector6d& _vel,
                                       Eigen::Matrix6d &_D) const;

  protected: void ComputeDamping(const Eigen::Vector6d& _vel,
                                 Eigen::Vector6d &_damping) const; // discarded

  /// \brief Returns the added-mass matrix with the scaling and offset
  protected: Eigen::Matrix6d GetAddedMass() const;

  /// \brief mass of uuv
  protected: double mass;
  
  /// \brief Buoyancy of uuv
  protected: double buoy;
  
  /// \brief inertia of uuv
  protected: std::vector<double> inertia;

  /// \brief rigid body transform matrix
  protected: Eigen::Matrix6d Mrb;

  /// \brief Coriolis 
  protected: Eigen::Matrix6d Crb;

  /// \brief Added-mass matrix
  protected: Eigen::Matrix6d Ma;

  /// \brief Scaling of the added-mass matrix
  protected: double scalingAddedMass;

  /// \brief Offset for the added-mass matrix
  protected: double offsetAddedMass;

  /// \brief Added-mass associated Coriolis matrix
  protected: Eigen::Matrix6d Ca;

  /// \brief Damping matrix
  protected: Eigen::Matrix6d D;

  /// \brief Scaling of the damping matrix
  protected: double scalingDamping;

  /// \brief Offset for the linear damping matrix
  protected: double offsetLinearDamping;

  /// \brief Offset for the linear damping matrix
  protected: double offsetLinForwardSpeedDamping;

  /// \brief Offset for the linear damping matrix
  protected: double offsetNonLinDamping;

  /// \brief Linear damping matrix
  protected: Eigen::Matrix6d DLin;

  /// \brief Linear damping matrix proportional only to the forward speed
  /// (useful for modeling AUVs). From [1], according to Newman (1977), there
  /// is a damping force component that linearly increases with the presence
  /// of forward speed, particularly so for slender bodies.
  ///
  /// References:
  /// [1] Refsnes - 2007 - Nonlinear model-based control of slender body AUVs
  protected: Eigen::Matrix6d DLinForwardSpeed;

  /// \brief Nonlinear damping coefficients
  protected: Eigen::Matrix6d DNonLin;

  /// \brief Linear damping coefficients
  protected: std::vector<double> linearDampCoef;

  /// \brief Quadratic damping coefficients
  protected: std::vector<double> quadDampCoef;

  /// \brief front rudder angle
  // protected: double frontRudderAng;
  
  /// \brief back rudder angle
  // protected: double backRudderAng;
  
  /// \brief vertical rudder angle
  // protected: double vertRudderAng;

  /// \brief front rudder factors
  protected: std::vector<double> frontRudderFactor;
  
  /// \brief back rudder factors
  protected: std::vector<double> backRudderFactor;

  /// \brief vertical rudder factors
  protected: std::vector<double> vertRudderFactor;

  /// \brief speed of rotor
  // protected: double rpm;

  /// \brief left death zone
  protected: double deltaLeft;
  
  /// \brief right death zone
  protected: double deltaRight;
  
  /// \brief rotor constant
  protected: double rotorConstantR;
  
  /// \brief rotor constant
  protected: double rotorConstantL;
};
}

#endif  // __UUV_GAZEBO_HYDRO_MODEL_HH__

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

#include <gazebo/gazebo.hh>
#include <uuv_gazebo_plugins/ArmsauvHydrodynamicModel.hh>

namespace gazebo
{
/////////////////////////////////////////////////
ArmsauvHydrodynamicModel::ArmsauvHydrodynamicModel(sdf::ElementPtr _sdf,
    physics::LinkPtr _link) : BuoyantObject(_link)
{
  GZ_ASSERT(_link != NULL, "Invalid link pointer");

  // Initialize filtered acceleration & last velocity
  this->filteredAcc.setZero();
  this->lastVelRel.setZero();

  // Set volume
  if (_sdf->HasElement("volume"))
    this->volume = _sdf->Get<double>("volume");


  // Reading the information for the metacentric width and length in the case
  // that the model is a surface vessel or floating object
  if (_sdf->HasElement("metacentric_width") &&
      _sdf->HasElement("metacentric_length") &&
      _sdf->HasElement("submerged_height"))
  {
    this->metacentricWidth = _sdf->Get<double>("metacentric_width");
    this->metacentricLength = _sdf->Get<double>("metacentric_length");
    this->submergedHeight = _sdf->Get<double>("submerged_height");
    this->isSurfaceVessel = true;

    gzmsg << "Surface vessel parameters" << std::endl;
    gzmsg << "\tMetacentric width [m]=" << this->metacentricWidth << std::endl;
    gzmsg << "\tMetacentric length [m]=" << this->metacentricLength << std::endl;
    gzmsg << "\tSubmerged height [m]=" << this->submergedHeight << std::endl;
  }
  else
  {
    this->metacentricWidth = 0.0;
    this->metacentricLength = 0.0;
    this->waterLevelPlaneArea = 0.0;
    this->isSurfaceVessel = false;
  }

  // Get the center of buoyancy
  std::vector<double> cob = {0, 0, 0};
  if (_sdf->HasElement("center_of_buoyancy"))
  {
    cob = Str2Vector(_sdf->Get<std::string>("center_of_buoyancy"));
    this->SetCoB(ignition::math::Vector3d(cob[0], cob[1], cob[2]));
  }
  // FIXME(mam0box) This is a work around the problem of the invalid bounding
  // box returned by Gazebo
  if (_sdf->HasElement("box"))
  {
    sdf::ElementPtr sdfModel = _sdf->GetElement("box");
    if (sdfModel->HasElement("width") && sdfModel->HasElement("length") &&
        sdfModel->HasElement("height"))
    {
      double width = sdfModel->Get<double>("width");
      double length = sdfModel->Get<double>("length");
      double height = sdfModel->Get<double>("height");
      ignition::math::Box boundingBox = ignition::math::Box(
        ignition::math::Vector3d(-width / 2, -length / 2, -height / 2),
        ignition::math::Vector3d(width / 2, length / 2, height / 2));
      // Setting the the bounding box from the given dimensions
      this->SetBoundingBox(boundingBox);
    }
  }

  // If neutrally buoyant is given, then calculate restoring
  // force to cancel out the gravitational force
  if (_sdf->HasElement("neutrally_buoyant"))
  {
    if (_sdf->Get<bool>("neutrally_buoyant"))
      this->SetNeutrallyBuoyant();
    else
      gzmsg << "not neutrally buoyant" <<std::endl;
  }

  // Initialize Reynolds number with zero (will not always be used)
  this->Re = 0;

  // Initialize temperature (not used by all models)
  this->temperature = 0;
}

/////////////////////////////////////////////////
void ArmsauvHydrodynamicModel::ComputeAcc(Eigen::Vector6d _velRel, double _time,
                                  double _alpha)
{
  // Compute Fossen's nu-dot numerically. We have to do this for now since
  // Gazebo reports angular accelerations that are off by orders of magnitude.
  double dt = _time - lastTime;

  if (dt <= 0.0)  // Extra caution to prevent division by zero
    return;

  Eigen::Vector6d acc = (_velRel - this->lastVelRel) / dt;

  // TODO  We only have access to the acceleration of the previous simulation
  //       step. The added mass will induce a strong force/torque counteracting
  //       it in the current simulation step. This can lead to an oscillating
  //       system.
  //       The most accurate solution would probably be to first compute the
  //       latest acceleration without added mass and then use this to compute
  //       added mass effects. This is not how gazebo works, though.
  this->filteredAcc = (1.0 - _alpha) * this->filteredAcc + _alpha * acc;

  lastTime = _time;
  this->lastVelRel = _velRel;
}

/////////////////////////////////////////////////
void ArmsauvHydrodynamicModel::ComputeAccNoFilter(Eigen::Vector6d _velRel, double _time)
{
  // Compute Fossen's nu-dot numerically. We have to do this for now since
  // Gazebo reports angular accelerations that are off by orders of magnitude.
  double dt = _time - lastTime;

  if (dt <= 0.0)  // Extra caution to prevent division by zero
    return;

  this->filteredAcc = (_velRel - this->lastVelRel) / dt; 

  lastTime = _time;
  this->lastVelRel = _velRel;
}

/////////////////////////////////////////////////
ignition::math::Vector3d ArmsauvHydrodynamicModel::ToNED(ignition::math::Vector3d _vec)
{
  ignition::math::Vector3d output = _vec;
  output.Y() = -1 * output.Y();
  output.Z() = -1 * output.Z();
  return output;
}

/////////////////////////////////////////////////
ignition::math::Vector3d ArmsauvHydrodynamicModel::FromNED(ignition::math::Vector3d _vec)
{
  return this->ToNED(_vec);
}

/////////////////////////////////////////////////
bool ArmsauvHydrodynamicModel::CheckParams(sdf::ElementPtr _sdf)
{
  if (this->params.empty()) return true;

  for (auto tag : this->params)
  {
    if (!_sdf->HasElement(tag))
      {
        gzerr << "Hydrodynamic model: Expected element " <<
           tag << std::endl;
        return false;
      }
  }

  return true;
}

/////////////////////////////////////////////////
ArmsauvHydrodynamicModel * ArmsauvHydrodynamicModelFactory::CreateArmsauvHydrodynamicModel(
    sdf::ElementPtr _sdf, physics::LinkPtr _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");
  sdf::ElementPtr sdfModel = _sdf->GetElement("hydrodynamic_model");
  if (!sdfModel->HasElement("type"))
  {
    std::cerr << "Model has no type" << std::endl;
    return NULL;
  }

  std::string identifier = sdfModel->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create ArmsauvHydrodynamicModel with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf, _link);
}

/////////////////////////////////////////////////
ArmsauvHydrodynamicModelFactory& ArmsauvHydrodynamicModelFactory::GetInstance()
{
  static ArmsauvHydrodynamicModelFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool ArmsauvHydrodynamicModelFactory::RegisterCreator(const std::string& _identifier,
                               ArmsauvHydrodynamicModelCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering ArmsauvHydrodynamicModel with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered ArmsauvHydrodynamicModel type " << _identifier << std::endl;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Fossen's robot-like equations of motion for underwater vehicles
//////////////////////////////////////////////////////////////////////////

const std::string ArmsauvFossen::IDENTIFIER = "fossen";
REGISTER_ArmsauvHydrodynamicModel_CREATOR(ArmsauvFossen,
                                   &ArmsauvFossen::create);

/////////////////////////////////////////////////
ArmsauvHydrodynamicModel* ArmsauvFossen::create(sdf::ElementPtr _sdf,
                                    physics::LinkPtr _link)
{
  gzmsg << "Create armsauv fossen model" << std::endl;
  return new ArmsauvFossen(_sdf, _link);
}

/////////////////////////////////////////////////
ArmsauvFossen::ArmsauvFossen(sdf::ElementPtr _sdf,
                   physics::LinkPtr _link)
                  : ArmsauvHydrodynamicModel(_sdf, _link)
{  
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  gzmsg << "Parse hydrodynamic parameters" << std::endl;

  // Get mass 
  if (modelParams->HasElement("body_mass"))
  {
      this->mass = modelParams->Get<double>("body_mass");
      gzmsg << "Get body mass: " << this->mass << std::endl;
  }
  this->params.push_back("body_mass");

  // Get buoyancy
  if (modelParams->HasElement("body_buoyancy"))
  {
      this->buoy = modelParams->Get<double>("body_buoyancy");
      gzmsg << "Get body buoyancy: " << this->buoy << std::endl;
  }
  this->params.push_back("body_buoyancy");

  // Get centre of gravity
  if(modelParams->HasElement("center_of_gravity"))
  {
      this->cog = Str2Vector(modelParams->Get<std::string>("center_of_gravity"));
      GZ_ASSERT(this->cog.size() == 3, "Dimension should be 3");
      gzmsg << "Get cog: " << this->cog[0] << " " << this->cog[1] << " " << this->cog[2]
          << std::endl;
  }
  this->params.push_back("center_of_gravity");

  // Get inertia
  if(modelParams->HasElement("inertia"))
  {
      this->inertia = Str2Vector(modelParams->Get<std::string>("inertia"));
      GZ_ASSERT(this->inertia.size() == 3, "Dimension should be 3");
      gzmsg << "Get inertia: " << this->inertia[0] << " " << this->inertia[1] << " " << this->inertia[2]
          << std::endl;
  }
  this->params.push_back("inertia");

  // Get death zone of thruster
  if(modelParams->HasElement("delta_left"))
  {
      this->deltaLeft = modelParams->Get<double>("delta_left");
      gzmsg << "Get delta left: " << this->deltaLeft << std::endl;
  }
  this->params.push_back("delta_left");
  
  // Get death zone of thruster
  if(modelParams->HasElement("delta_right"))
  {
      this->deltaRight = modelParams->Get<double>("delta_right");
      gzmsg << "Get delta right: " << this->deltaRight << std::endl;
  }
  this->params.push_back("delta_right");
  
  // Get rotor constant
  if(modelParams->HasElement("rotor_constant_right"))
  {
      this->rotorConstantR = modelParams->Get<double>("rotor_constant_right");
      gzmsg << "Get right rotor constant: " << this->rotorConstantR << std::endl;
  }
  this->params.push_back("rotor_constant_right");
  
  // Get rotor constant
  if(modelParams->HasElement("rotor_constant_left"))
  {
      this->rotorConstantL = modelParams->Get<double>("rotor_constant_left");
      gzmsg << "Get left rotor constant: " << this->rotorConstantL << std::endl;
  }
  this->params.push_back("rotor_constant_left");

  // Get rotor speed threshold
  if(modelParams->HasElement("rotor_speed_threshold"))
  {
      this->rotorConstantR = modelParams->Get<double>("rotor_speed_threshold");
      gzmsg << "Rotor speed threshold: " << this->rotorSpeedThreshold << std::endl;
  }
  this->params.push_back("rotor_constant_right");

  Eigen::Matrix6d mrb;
  // Get rigid body transform matrix
  mrb << this->mass, 0.0, 0.0, 0.0, this->mass*this->cog[2], -this->mass*this->cog[1],
       0.0, this->mass, 0.0, -this->mass*this->cog[2], 0.0, this->mass*this->cog[0],
       0.0, 0.0, this->mass, this->mass*this->cog[1], -this->mass*this->cog[0], 0.0,
       0.0, -this->mass*this->cog[2], this->mass*this->cog[1], this->inertia[0], 0.0, 0.0,
       this->mass*this->cog[2], 0.0, -this->mass*this->cog[0], 0.0, this->inertia[1], 0.0,
       -this->mass*this->cog[1], this->mass*this->cog[0], 0.0, 0.0, 0.0, this->inertia[2];

  this->Mrb = mrb;

  // Get front rudder
  if(modelParams->HasElement("front_rudder_factor"))
  {
      this->frontRudderFactor = Str2Vector(modelParams->Get<std::string>("front_rudder_factor"));
      GZ_ASSERT(this->frontRudderFactor.size() == 6, "Dimension should be 6");
      gzmsg << "Get front rudder factor: " << this->frontRudderFactor[0] << " " << this->frontRudderFactor[1] << " " << this->frontRudderFactor[2]
          << " " << this->frontRudderFactor[3] << " " << this->frontRudderFactor[4] << " " << this->frontRudderFactor[5] << std::endl;
  }
  this->params.push_back("front_rudder_factor");
  
  // Get back rudder
  if(modelParams->HasElement("back_rudder_factor"))
  {
      this->backRudderFactor = Str2Vector(modelParams->Get<std::string>("back_rudder_factor"));
      GZ_ASSERT(this->backRudderFactor.size() == 6, "Dimension should be 6");
      gzmsg << "Get back rudder factor: " << this->backRudderFactor[0] << " " << this->backRudderFactor[1] << " " << this->backRudderFactor[2]
          << " " << this->backRudderFactor[3] << " " << this->backRudderFactor[4] << " " << this->backRudderFactor[5] << std::endl;
  }
  this->params.push_back("back_rudder_factor");
  
  // Get vert rudder
  if(modelParams->HasElement("vertical_rudder_factor"))
  {
      this->vertRudderFactor = Str2Vector(modelParams->Get<std::string>("vertical_rudder_factor"));
      GZ_ASSERT(this->vertRudderFactor.size() == 6, "Dimension should be 6");
      gzmsg << "Get vertical rudder factor: " << this->vertRudderFactor[0] << " " << this->vertRudderFactor[1] << " " << this->vertRudderFactor[2]
          << " " << this->vertRudderFactor[3] << " " << this->vertRudderFactor[4] << " " << this->vertRudderFactor[5] << std::endl;
  }
  this->params.push_back("vertical_rudder_factor");

  std::vector<double> addedMass(36, 0.0);
  std::vector<double> linDampCoef(6, 0.0);
  std::vector<double> linDampForward(6, 0.0);
  std::vector<double> quadDampCoef(6, 0.0);

  // GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
  //           "Hydrodynamic model is missing");

  // sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");
  
  // Load added-mass coefficients, if provided. Otherwise, the added-mass
  // matrix is set to zero
  if (modelParams->HasElement("added_mass"))
    addedMass = Str2Vector(modelParams->Get<std::string>("added_mass"));
  else
    gzmsg << "ArmsauvFossen: Using added mass NULL" << std::endl;

  this->params.push_back("added_mass");

  // Load linear damping coefficients, if provided. Otherwise, the linear
  // damping matrix is set to zero
  if (modelParams->HasElement("linear_damping"))
    linDampCoef = Str2Vector(modelParams->Get<std::string>("linear_damping"));
  else
    gzmsg << "ArmsauvFossen: Using linear damping NULL" << std::endl;

  // Add added mass' scaling factor to the parameter list
  this->params.push_back("scaling_added_mass");
  // Set default value for the added mass's scaling vector
  this->scalingAddedMass = 1.0;
  // Add added mass' scaling factor to the parameter list
  this->params.push_back("offset_added_mass");
  // Set default value for the added mass identity offset
  this->offsetAddedMass = 0.0;

  // Add linear damping to the parameter list
  this->params.push_back("linear_damping");

  // Load linear damping coefficients that described the damping forces
  // proportional to the forward speed only, if provided. Otherwise, the linear
  // damping matrix is set to zero
  if (modelParams->HasElement("linear_damping_forward_speed"))
    linDampForward = Str2Vector(
      modelParams->Get<std::string>("linear_damping_forward_speed"));
  else
    gzmsg << "ArmsauvFossen: Using linear damping for forward speed NULL"
      << std::endl;
  // Add the matrix for linear damping proportional to forward speed to the
  // parameter list
  this->params.push_back("linear_damping_forward_speed");

  // Load nonlinear quadratic damping coefficients, if provided. Otherwise,
  // the nonlinear quadratic damping matrix is set to zero
  if (modelParams->HasElement("quadratic_damping"))
    quadDampCoef = Str2Vector(
        modelParams->Get<std::string>("quadratic_damping"));
  else
    gzmsg << "ArmsauvFossen: Using quad damping NULL" << std::endl;

  // Add quadratic damping coefficients to the parameter list
  this->params.push_back("quadratic_damping");
  // Add damping's scaling factor to the parameter list
  this->params.push_back("scaling_damping");
  // Setting the damping scaling default value
  this->scalingDamping = 1.0;

  // Add the offset for the linear damping coefficients to the parameter list
  this->params.push_back("offset_linear_damping");
  // Set the offset of the linear damping coefficients to default value
  this->offsetLinearDamping = 0.0;

  // Add the offset for the linear damping coefficients to the parameter list
  this->params.push_back("offset_lin_forward_speed_damping");
  // Set the offset of the linear damping coefficients to default value
  this->offsetLinForwardSpeedDamping = 0.0;

  // Add the offset for the linear damping coefficients to the parameter list
  this->params.push_back("offset_nonlin_damping");
  // Set the offset of the linear damping coefficients to default value
  this->offsetNonLinDamping = 0.0;

  // Adding the volume to the parameter list
  this->params.push_back("volume");
  // Add volume's scaling factor to the parameter list
  this->params.push_back("scaling_volume");

  GZ_ASSERT(addedMass.size() == 36,
            "Added-mass coefficients vector must have 36 elements");
  GZ_ASSERT(linDampCoef.size() == 6 || linDampCoef.size() == 36,
            "Linear damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");
  GZ_ASSERT(linDampForward.size() == 6 || linDampForward.size() == 36,
            "Linear damping coefficients proportional to the forward speed "
            "vector must have 6 elements for a diagonal matrix or 36 elements"
            " for a full matrix");
  GZ_ASSERT(quadDampCoef.size() == 6 || quadDampCoef.size() == 36,
            "Quadratic damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");

  this->DLin.setZero();
  this->DNonLin.setZero();
  this->DLinForwardSpeed.setZero();

  for (int row = 0; row < 6; row++)
    for (int col = 0; col < 6; col++)
    {
      // Set added-mass coefficient
      this->Ma(row, col) = addedMass[6*row+col];
      // Set the linear damping matrix if a full matrix was provided
      if (linDampCoef.size() == 36)
        this->DLin(row, col) = linDampCoef[6*row+col];
      if (quadDampCoef.size() == 36)
        this->DNonLin(row, col) = quadDampCoef[6*row+col];
      if (linDampForward.size() == 36)
        this->DLinForwardSpeed(row, col) = linDampForward[6*row+col];
    }

  // In the case the linear damping matrix was set as a diagonal matrix
  for (int i = 0; i < 6; i++)
  {
    if (linDampCoef.size() == 6)
      this->DLin(i, i) = linDampCoef[i];
    if (quadDampCoef.size() == 6)
      this->DNonLin(i, i) = quadDampCoef[i];
    if (linDampForward.size() == 6)
      this->DLinForwardSpeed(i, i) = linDampForward[i];
  }

  // Store damping coefficients
  this->linearDampCoef = linDampCoef;
  this->quadDampCoef = quadDampCoef;
}

/////////////////////////////////////////////////
void ArmsauvFossen::ApplyHydrodynamicForces(
  double _time, const ignition::math::Vector3d &_flowVelWorld,
  double _rouderb, double _rouders, double _rouderr, double _rpm)
{
  // Link's pose
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linVel, angVel;

#if GAZEBO_MAJOR_VERSION >= 8
  pose = this->link->WorldPose();
  linVel = this->link->RelativeLinearVel();
  angVel = this->link->RelativeAngularVel();
#else
  pose = this->link->GetWorldPose().Ign();

  gazebo::math::Vector3 linVelG, angVelG;
  linVelG = this->link->GetRelativeLinearVel();
  angVelG = this->link->GetRelativeAngularVel();
  linVel = ignition::math::Vector3d(
    linVelG.x, linVelG.y, linVelG.z);
  angVel = ignition::math::Vector3d(
    angVelG.x, angVelG.y, angVelG.z);
#endif

  // get gravity acc
  double g_acc = this->GetGravity();
  gzmsg << "Gravity acc: " << g_acc << std::endl;

  // Transform the flow velocity to the BODY frame
  ignition::math::Vector3d flowVel = pose.Rot().RotateVectorReverse(
    _flowVelWorld);

  Eigen::Vector6d velRel;
  // Compute the relative velocity
  velRel = EigenStack(
    this->ToNED(linVel - flowVel),
    this->ToNED(angVel));

  gzmsg << "Relative velocity: " << velRel[0] << " " << velRel[1] << " " << velRel[2] 
      << " " << velRel[3] << " " << velRel[4] << " " << velRel[5] << std::endl;

  // get pose in euler
  ignition::math::Quaternion<double> quad = pose.Rot();
  ignition::math::Vector3d euler = quad.Euler();

  gzmsg << "Pose: " << euler.X() << " " << euler.Y() << " " << euler.Z() << std::endl;
  
  /* Calculate restoring force & torque */
  ignition::math::Vector3d restoring_force;
  restoring_force.X() = -(this->mass*g_acc - this->buoy)*std::sin(euler.Y());
  restoring_force.Y() = (this->mass*g_acc - this->buoy)*std::cos(euler.Y())*std::sin(euler.X());
  restoring_force.Z() = (this->mass*g_acc - this->buoy)*std::cos(euler.Y())*std::cos(euler.X());

  ignition::math::Vector3d restoring_torque;
  std::vector<double> cog = this->cog;
  ignition::math::Vector3d cob = this->GetCoB();
  double xg = cog[0]; double yg = cog[1]; double zg = cog[2];
  double xb = cob.X(); double yb = cob.Y(); double zb = cob.Z();
  restoring_torque.X() = (yg*this->mass*g_acc - yb*this->buoy)*std::cos(euler.Y())*std::cos(euler.X())-
      (zg*this->mass*g_acc - zb*this->buoy)*std::cos(euler.Y())*std::sin(euler.X());
  restoring_torque.Y() = -(xg*this->mass*g_acc - xb*this->buoy)*std::cos(euler.Y())*std::cos(euler.X())-
      (zg*this->mass*g_acc - zb*this->buoy)*std::sin(euler.Y());
  restoring_torque.Z() = (xg*this->mass*g_acc - xb*this->buoy)*std::cos(euler.Y())*std::sin(euler.X())-
      (yg*this->mass*g_acc - yb*this->buoy)*std::sin(euler.Y());

  gzmsg << "Restoring wrench: " << restoring_force.X() << " " << restoring_force.Y() << " " << restoring_torque.Z()
      << " " << restoring_torque.X() << " " << restoring_torque.Y() << " " << restoring_torque.Z() << std::endl;

  Eigen::Vector6d tau_bg = EigenStack(restoring_force, restoring_torque);

  /* Calculate thruster force */
  ignition::math::Vector3d thruster_force, thruster_torque;
  // double rpm = std::floor(this->rpm);
  double rpm = std::floor(_rpm);
  if(rpm > this->rotorSpeedThreshold){
      thruster_force.X() = this->rotorConstantR * (rpm * fabs(rpm) - this->deltaRight);
  }
  else if(rpm < -this->rotorSpeedThreshold){
      thruster_force.X() = this->rotorConstantL * (rpm * fabs(rpm) - this->deltaLeft);
  }
  else{
      thruster_force.X() = 0.0;
  }

  thruster_force.Y() = 0.0;
  thruster_force.Z() = 0.0;
  thruster_torque.X() = 0.0;
  thruster_torque.Y() = 0.0;
  thruster_torque.Z() = 0.0;
  
  Eigen::Vector6d tau_c = EigenStack(thruster_force, restoring_torque);

  gzmsg << "Thruster wrench: " << tau_c[0] << " " << tau_c[1] << " " << tau_c[2]
      << " " << tau_c[3] << " " << tau_c[4] << " " << tau_c[5] << std::endl;

  /* Calculate rudder lift & drag */
  ignition::math::Vector3d rudder_force, rudder_torque;
  // rudder_force.X() = 0.0;
  // rudder_force.Y() = this->vertRudderFactor[1]*this->vertRudderAng*velRel[0]*velRel[0];
  // rudder_force.Z() = this->frontRudderFactor[2]*this->frontRudderAng*velRel[0]*velRel[0]+ this->backRudderFactor[2]*this->backRudderAng*velRel[0]*velRel[0];
  // rudder_torque.X() = 0.0;
  // rudder_torque.Y() = this->vertRudderFactor[4]*this->vertRudderAng*velRel[0]*velRel[0];
  // rudder_torque.Z() = this->frontRudderFactor[5]*this->frontRudderAng*velRel[0]*velRel[0] + this->backRudderFactor[5]*this->backRudderAng*velRel[0]*velRel[0];
  
  rudder_force.X() = 0.0;
  rudder_force.Y() = this->vertRudderFactor[1]*_rouderr*velRel[0]*velRel[0];
  rudder_force.Z() = this->frontRudderFactor[2]*_rouderb*velRel[0]*velRel[0]+ this->backRudderFactor[2]*_rouders*velRel[0]*velRel[0];
  rudder_torque.X() = 0.0;
  rudder_torque.Y() = this->vertRudderFactor[4]*_rouderr*velRel[0]*velRel[0];
  rudder_torque.Z() = this->frontRudderFactor[5]*_rouderb*velRel[0]*velRel[0] + this->backRudderFactor[5]*_rouders*velRel[0]*velRel[0];
  Eigen::Vector6d tau_r = EigenStack(rudder_force, rudder_torque);

  /* Update solid Coriolis matrix */
  Eigen::Matrix6d C_rb;
  C_rb << 0.0, 0.0, 0.0, this->mass*(yg*angVel.Y()+zg*angVel.Z()), -this->mass*(xg*angVel.Y()-linVel.Z()), -this->mass*(xg*angVel.Z()+linVel.Y()),
       0.0, 0.0, 0.0, -this->mass*(yg*angVel.X()+linVel.Z()), this->mass*(zg*angVel.Z()+xg*angVel.X()), -this->mass*(yg*angVel.Z()-linVel.X()),
       0.0, 0.0, 0.0, -this->mass*(zg*angVel.X()-linVel.Y()), -this->mass*(zg*angVel.Y()+linVel.X()), this->mass*(xg*angVel.X() + yg*angVel.Y()),
       -this->mass*(yg*angVel.Y()+zg*angVel.Z()), this->mass*(yg*angVel.X()+linVel.Z()), this->mass*(zg*angVel.X()-linVel.Y()), 0.0, this->inertia[2]*angVel.Z(), -this->inertia[1]*angVel.Y(),
       this->mass*(xg*angVel.Y()-linVel.Z()), -this->mass*(zg*angVel.Z()+xg*angVel.X()), this->mass*(zg*angVel.Y()+linVel.X()), -this->inertia[2]*angVel.Z(), 0.0, this->inertia[0]*angVel.X(),
       this->mass*(xg*angVel.Z()+linVel.Y()), this->mass*(yg*angVel.Z()-linVel.X()), -this->mass*(xg*angVel.X()+yg*angVel.Y()), this->inertia[1]*angVel.Y(), -this->inertia[0]*angVel.X(), 0.0;
  this->Crb = C_rb;

  // Update added Coriolis matrix
  this->ComputeAddedCoriolisMatrix(velRel, this->Ma, this->Ca);

  // Update damping matrix
  this->ComputeDampingMatrix(velRel, this->D);
  // this->ComputeLinForwardSpeedDampingMatrix(velRel, this->D);

  // Filter acceleration (see issue explanation above)
  this->ComputeAcc(velRel, _time, 0.3);
  // this->ComputeAccNoFilter(velRel, _time);

  // We can now compute the additional forces/torques due to thisdynamic
  // Damping forces and torques
  Eigen::Vector6d damping = -this->D * velRel;

  // Estimate acc
  Eigen::Vector6d acc = (this->Mrb + this->Ma).inverse()*(-(this->Crb + this->Ca + this->D)*velRel + tau_bg + tau_r + tau_c);
  gzmsg << "Acceleration: " << acc << std::endl;

  // Added-mass forces and torques
  // Eigen::Vector6d added = -this->GetAddedMass() * this->filteredAcc;
  Eigen::Vector6d added = -this->GetAddedMass() * acc;

  // Added Coriolis term
  Eigen::Vector6d cor = -this->Ca * velRel;

  // All additional (compared to standard rigid body) Fossen terms combined.
  Eigen::Vector6d tau = damping + added + cor;

  // std::string msg = this->link->GetModel()->GetName() + ": Hydrodynamic forces vector is nan";
  // GZ_ASSERT(!std::isnan(tau.norm()), msg.c_str());

  /* Test interface */
  /*
  printf("Force of %s: %f, %f, %f\n", this->link->GetName().c_str(), tau.head(0), tau.head(1), tau.head(2));
  printf("Torque of %s: %f, %f, %f\n", this->link->GetName().c_str(), tau.head(3), tau.head(4), tau.head(5));

  if(!std::isnan(tau.norm())){
      printf("force norm of %s: %f\n", this->link->GetName().c_str(), tau.norm());
  }*/
  
  // GZ_ASSERT(!std::isnan(tau.norm()), ": Hydrodynamic forces vector is nan");

  if (!std::isnan(tau.norm()))
  {
    // Convert the forces and moments back to Gazebo's reference frame
    ignition::math::Vector3d hydForce =
      this->FromNED(Vec3dToGazebo(tau.head<3>()));
    ignition::math::Vector3d hydTorque =
      this->FromNED(Vec3dToGazebo(tau.tail<3>()));

    // Forces and torques are also wrt link frame
    this->link->AddRelativeForce(hydForce);
    this->link->AddRelativeTorque(hydTorque);
  }

  this->ApplyBuoyancyForce();

  if ( this->debugFlag )
  {
    // Store intermediate results for debugging purposes
    this->StoreVector(UUV_DAMPING_FORCE, Vec3dToGazebo(damping.head<3>()));
    this->StoreVector(UUV_DAMPING_TORQUE, Vec3dToGazebo(damping.tail<3>()));

    this->StoreVector(UUV_ADDED_MASS_FORCE, Vec3dToGazebo(added.head<3>()));
    this->StoreVector(UUV_ADDED_MASS_TORQUE, Vec3dToGazebo(added.tail<3>()));

    this->StoreVector(UUV_ADDED_CORIOLIS_FORCE, Vec3dToGazebo(cor.head<3>()));
    this->StoreVector(UUV_ADDED_CORIOLIS_TORQUE, Vec3dToGazebo(cor.tail<3>()));
  }
}

/////////////////////////////////////////////////
void ArmsauvFossen::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                          const Eigen::Matrix6d& _Ma,
                                          Eigen::Matrix6d &_Ca) const
{
  // This corresponds to eq. 6.43 on p. 120 in
  // Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
  // Control", 2011
  Eigen::Vector6d ab = this->GetAddedMass() * _vel;
  Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
  _Ca << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1 * CrossProductOperator(ab.tail<3>());
}

/////////////////////////////////////////////////
void ArmsauvFossen::ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  // From Antonelli 2014: the viscosity of the fluid causes
  // the presence of dissipative drag and lift forces on the
  // body. A common simplification is to consider only linear
  // and quadratic damping terms and group these terms in a
  // matrix Drb

  _D.setZero();

  _D = -1 *
    (this->DLin + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
    _vel[0] * (this->DLinForwardSpeed +
      this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

  // Nonlinear damping matrix is considered as a diagonal matrix
  // for (int i = 0; i < 6; i++)
  // {
  //   _D(i, i) += -1 *
  //     (this->DNonLin(i, i) + this->offsetNonLinDamping) *
  //     std::fabs(_vel[i]);
  // }

  for (int i = 0; i < 6; i++){
      for(int j = 0; j < 6; j++){
          _D(i, j) += -1 * (this->DNonLin(i, j) + this->offsetNonLinDamping) * std::fabs(_vel[j]);
      }
  }

  _D *= this->scalingDamping;
}

/////////////////////////////////////////////////
void ArmsauvFossen::ComputeQuadDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  _D.setZero();

  /*
  _D = -1 *
    (this->DLin + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
    _vel[0] * (this->DLinForwardSpeed +
      this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());
  */
  _D = -1 * this->DLin;

  // Nonlinear damping matrix is considered as a diagonal matrix
  for (int i = 0; i < 6; i++)
      for(int j = 0; j < 6; j++)
  {
    _D(i, j) += -1 *
      (this->DNonLin(i, j) + this->offsetNonLinDamping) *
      _vel[0];
  }
  _D *= this->scalingDamping;
}

/////////////////////////////////////////////////
void ArmsauvFossen::ComputeLinForwardSpeedDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  _D.setZero();
  _D = -1 * _vel[0] * this->DLinForwardSpeed;
  _D *= this->scalingDamping;
}

/////////////////////////////////////////////////
void ArmsauvFossen::ComputeDamping(const Eigen::Vector6d& _vel,
                              Eigen::Vector6d &_damping) const
{
  double Xuu = -117.573, 
         Yuv = -3695.169, Yup = 2156.455, Yur = 7963.276,
         Zuw = -2500.771, Zuq = -2095.934,
         Kuv = 1293.236, Kup = -3941.599, Kur= -1571.203, 
         Muw = 382.237,  Muq = -18675.024,
         Nuv = -1592.655, Nup = 233.778, Nur = -20306.030;

  double u = _vel[0],
         v = _vel[1],
         w = _vel[2],
         p = _vel[3],
         q = _vel[4],
         r = _vel[5];

  double X = 0, 
         Y = 0, 
         Z = 0, 
         K = 0, 
         M = 0, 
         N =0;

  X = Xuu * u * u;
  Y = Yuv * u * v + 
      Yup * u * p + 
      Yur * u * r;
  Z = Zuw * u * w +
      Zuq * u * q;
  K = Kuv * u * v + 
      Kup * u * p + 
      Kur * u * r;
  M = Muw * u * w +
      Muq * u * q;
  N = Nuv * u * v + 
      Nup * u * p + 
      Nur * u * r;

  static double time = 0.0;

  Z += 20 * std::sin(0.25 * 3.1415926 * time) + 100*(rand() / double(RAND_MAX)-0.5);
  M += 20 * std::cos(0.3 * 3.1415926 * time) + 100*(rand() / double(RAND_MAX)-0.5);
  // N += 2.5 * std::cos(0.25 * 3.1415926 * time);
  // Z += 20 * std::sin(0.25 * 3.1415926 * time);
  // M += 20 * std::cos(0.3 * 3.1415926 * time);

  time += 0.002;

  _damping[0] = X;
  _damping[1] = Y;
  _damping[2] = Z;
  _damping[3] = K;
  _damping[4] = M;
  _damping[5] = N;
  // _damping[0] = 0;
  // _damping[1] = 0;
  // _damping[2] = 0;
  // _damping[3] = 0;
  // _damping[4] = 0;
  // _damping[5] = 0;
}

/////////////////////////////////////////////////
Eigen::Matrix6d ArmsauvFossen::GetAddedMass() const
{
  return this->scalingAddedMass *
    (this->Ma + this->offsetAddedMass * Eigen::Matrix6d::Identity());
}

/////////////////////////////////////////////////
bool ArmsauvFossen::GetParam(std::string _tag, std::vector<double>& _output)
{
  _output = std::vector<double>();
  if(!_tag.compare("center_of_gravity"))
  {
      for(int i = 0; i < this->cog.size(); ++i)
      {
          _output.push_back(this->cog[i]);
      }
  }
  if(!_tag.compare("inertia"))
  {
      for(int i = 0; i < this->inertia.size(); ++i)
      {
          _output.push_back(this->inertia[i]);
      }
  }
  if(!_tag.compare("front_rudder_factor"))
  {
      for(int i = 0; i < this->frontRudderFactor.size(); ++i)
      {
          _output.push_back(this->frontRudderFactor[i]);
      }
  }
  if(!_tag.compare("back_rudder_factor"))
  {
      for(int i = 0; i < this->backRudderFactor.size(); ++i)
      {
          _output.push_back(this->backRudderFactor[i]);
      }
  }
  if(!_tag.compare("vertical_rudder_factor"))
  {
      for(int i = 0; i < this->vertRudderFactor.size(); ++i)
      {
          _output.push_back(this->vertRudderFactor[i]);
      }
  }
  if (!_tag.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->Ma(i, j));
  }
  else if (!_tag.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLin(i, j));
  }
  else if (!_tag.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLinForwardSpeed(i, j));
  }
  else if (!_tag.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DNonLin(i, j));
  }
  else if (!_tag.compare("center_of_buoyancy"))
  {
    _output.push_back(this->centerOfBuoyancy.X());
    _output.push_back(this->centerOfBuoyancy.Y());
    _output.push_back(this->centerOfBuoyancy.Z());
  }
  else
    return false;
  gzmsg << "ArmsauvHydrodynamicModel::GetParam <" << _tag << ">=" << std::endl;
  for (auto elem : _output)
    std::cout << elem << " ";
  std::cout << std::endl;
  return true;
}

/////////////////////////////////////////////////
bool ArmsauvFossen::GetParam(std::string _tag, double& _output)
{
  _output = -1.0;
  if (!_tag.compare("volume"))
    _output = this->volume;
  else if (!_tag.compare("scaling_volume"))
    _output = this->scalingVolume;
  else if (!_tag.compare("scaling_added_mass"))
    _output = this->scalingAddedMass;
  else if (!_tag.compare("scaling_damping"))
    _output = this->scalingDamping;
  else if (!_tag.compare("fluid_density"))
    _output = this->fluidDensity;
  else if (!_tag.compare("bbox_height"))
    _output = this->boundingBox.ZLength();
  else if (!_tag.compare("bbox_width"))
    _output = this->boundingBox.YLength();
  else if (!_tag.compare("bbox_length"))
    _output = this->boundingBox.XLength();
  else if (!_tag.compare("offset_volume"))
    _output = this->offsetVolume;
  else if (!_tag.compare("offset_added_mass"))
    _output = this->offsetAddedMass;
  else if (!_tag.compare("offset_linear_damping"))
    _output = this->offsetLinearDamping;
  else if (!_tag.compare("offset_lin_forward_speed_damping"))
    _output = this->offsetLinForwardSpeedDamping;
  else if (!_tag.compare("offset_nonlin_damping"))
    _output = this->offsetNonLinDamping;
  else if(!_tag.compare("mass"))
    _output = this->mass;
  else if(!_tag.compare("delta_left"))
    _output = this->deltaLeft;
  else if(!_tag.compare("delta_right"))
    _output = this->deltaRight;
  else if(!_tag.compare("rotor_constant_left"))
    _output = this->rotorConstantL;
  else if(!_tag.compare("rotor_constant_right"))
    _output = this->rotorConstantR;
  else if(!_tag.compare("rotor_speed_threshold"))
    _output = this->rotorSpeedThreshold;
  else
  {
    _output = -1.0;
    return false;
  }

  gzmsg << "ArmsauvHydrodynamicModel::GetParam <" << _tag << ">=" << _output <<
    std::endl;
  return true;
}

/////////////////////////////////////////////////
bool ArmsauvFossen::SetParam(std::string _tag, double _input)
{
  if (!_tag.compare("scaling_volume"))
  {
    if (_input < 0)
      return false;
    this->scalingVolume = _input;
  }
  else if (!_tag.compare("scaling_added_mass"))
  {
    if (_input < 0)
      return false;
    this->scalingAddedMass = _input;
  }
  else if (!_tag.compare("scaling_damping"))
  {
    if (_input < 0)
      return false;
    this->scalingDamping = _input;
  }
  else if (!_tag.compare("fluid_density"))
  {
    if (_input < 0)
      return false;
    this->fluidDensity = _input;
  }
  else if (!_tag.compare("offset_volume"))
    this->offsetVolume = _input;
  else if (!_tag.compare("offset_added_mass"))
    this->offsetAddedMass = _input;
  else if (!_tag.compare("offset_linear_damping"))
    this->offsetLinearDamping = _input;
  else if (!_tag.compare("offset_lin_forward_speed_damping"))
    this->offsetLinForwardSpeedDamping = _input;
  else if (!_tag.compare("offset_nonlin_damping"))
    this->offsetNonLinDamping = _input;
  else if(!_tag.compare("body_mass"))
      this->mass = _input;
  else if(!_tag.compare("body_buoyancy"))
      this->buoy = _input;
  else if(!_tag.compare("delta_left"))
      this->deltaLeft = _input;
  else if(!_tag.compare("delta_right"))
      this->deltaRight = _input;
  else if(!_tag.compare("rotor_constant_left"))
      this->rotorConstantL = _input;
  else if(!_tag.compare("rotor_constant_right"))
      this->rotorConstantR = _input;
  else if(!_tag.compare("rotor_speed_threshold"))
      this->rotorSpeedThreshold = _input;
  else
    return false;
  gzmsg << "ArmsauvHydrodynamicModel::SetParam <" << _tag << ">=" << _input <<
    std::endl;
  return true;
}

/*
/////////////////////////////////////////////////
void ArmsauvFossen::SetFrontRudderAngle(double rudderb)
{
    this->frontRudderAng = rudderb;
}

/////////////////////////////////////////////////
void ArmsauvFossen::SetBackRudderAngle(double rudders)
{
    this->backRudderAng = rudders;
}

/////////////////////////////////////////////////
void ArmsauvFossen::SetVertRudderAngle(double rudderr)
{
    this->vertRudderAng = rudderr;
}

/////////////////////////////////////////////////
void ArmsauvFossen::SetRotorSpeed(double rpm)
{
    this->rpm = rpm;
}
*/

/////////////////////////////////////////////////
void ArmsauvFossen::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("all"))
  {
    for (auto tag : this->params)
      this->Print(tag);
    return;
  }
  if (!_message.empty())
    std::cout << _message << std::endl;
  else
    std::cout << this->link->GetModel()->GetName() << "::"
      << this->link->GetName() << "::" << _paramName
      << std::endl;
  if (!_paramName.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->Ma(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLinForwardSpeed(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DNonLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("volume"))
  {
    std::cout << std::setw(12) << this->volume << " m^3" << std::endl;
  }
  else if(!_paramName.compare("body_mass"))
  {
    std::cout << std::setw(12) << this->mass << " kg" << std::endl;
  }
  else if(!_paramName.compare("body_buoyancy"))
  {
    std::cout << std::setw(12) << this->buoy << " N" << std::endl;
  }
  else if(!_paramName.compare("center_of_gravity"))
  {
    for (int i = 0; i < 3; i++)
      std::cout << std::setw(12) << this->cog[i];
    std::cout << std::endl;
  }
  else if(!_paramName.compare("inertia"))
  {
    for (int i = 0; i < 3; i++)
      std::cout << std::setw(12) << this->inertia[i];
    std::cout << std::endl;
  }
  else if(!_paramName.compare("delta_left"))
  {
      std::cout << std::setw(12) << this->deltaLeft << std::endl;
  }
  else if(!_paramName.compare("delta_right"))
  {
      std::cout << std::setw(12) << this->deltaRight << std::endl;
  }
  else if(!_paramName.compare("rotor_constant_right"))
  {
      std::cout << std::setw(12) << this->rotorConstantR << std::endl;
  }
  else if(!_paramName.compare("rotor_constant_left"))
  {
      std::cout << std::setw(12) << this->rotorConstantL << std::endl;
  }
  else if(!_paramName.compare("rotor_speed_threshold"))
  {
      std::cout << std::setw(12) << this->rotorSpeedThreshold << std::endl;
  }
  else if(!_paramName.compare("front_rudder_factor"))
  {
    for (int i = 0; i < 6; i++)
      std::cout << std::setw(12) << this->frontRudderFactor[i];
    std::cout << std::endl;
  }
  else if(!_paramName.compare("back_rudder_factor"))
  {
    for (int i = 0; i < 6; i++)
      std::cout << std::setw(12) << this->backRudderFactor[i];
    std::cout << std::endl;
  }
  else if(!_paramName.compare("vertical_rudder_factor"))
  {
    for (int i = 0; i < 6; i++)
      std::cout << std::setw(12) << this->vertRudderFactor[i];
    std::cout << std::endl;
  }
}
}; // ns

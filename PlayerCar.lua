
require "Tire"

--[[ PlayerCar
  Class defining the functionality of the player-controlled car.
--]]
PlayerCar = Object:extend()

--[[ PlayerCar:new
  PlayerCar constructor.
  
  x: X coordinate to initialize the car at.
  y: Y coordinate to initialize the car at.
--]]
function PlayerCar:new(x, y)
  
  -- Define image
  self.image = Sprite("images/CorvetteC5.png")
  
  -- Define attributes
  self.length = self.image.width / pxPerMtr
  self.width = self.image.height / pxPerMtr
  self.maxSteeringAngle = 30 * math.pi/180
  self.wheelbase = 2.65
  self.mass = 1500
  
  self.idleRpm = 1200
  self.redlineRpm = 6000
  local torqueValues = {297.09, 304.16, 310.24, 316.21, 321.50, 325.36, 328.32, 331.21, 332.89, 334.79,
                       337.73, 340.97, 346.18, 349.93, 352.09, 353.66, 352.89, 353.30, 351.66, 346.61,
                       338.96, 329.55, 311.85, 294.41, 274.84}
  
  self.numTorqueSteps = #torqueValues - 1
  self.torqueStep = (self.redlineRpm - self.idleRpm) / self.numTorqueSteps
  local ftLbToNm = 1.3558
  
  -- Build map of RPM to torque values
  self.torqueCurve = {}
  for i = 0, self.numTorqueSteps do
    self.torqueCurve[i] = {self.idleRpm + (i * self.torqueStep), ftLbToNm * torqueValues[i + 1]}
  end
  
  self.gearRatios = {-2.90, 2.66, 1.78, 1.30, 1.00, 0.74, 0.50}
  self.finalDrive = 3.42
  self.wheelRadius = 0.34
  self.driveEfficiency = 0.85
  self.gearShiftTime = 0.7
  
  self.tireMu = 1.17
  self.frontWheelDrive = false
  self.rearWheelDrive = true
  
  -- Compute angular inertia of drivetrain + wheels
  local wheelMass = 10
  local twoWheelsAngInertia = 2*wheelMass * self.wheelRadius^2
  local drivelineAngInertia = 1 -- VERY rough estimate
  self.frontAngInertia = twoWheelsAngInertia
  self.rearAngInertia = twoWheelsAngInertia
  if self.frontWheelDrive then
    self.frontAngInertia = self.frontAngInertia + drivelineAngInertia
  end
  if self.rearWheelDrive then
    self.rearAngInertia = self.rearAngInertia + drivelineAngInertia
  end
  
  self.brakeTorque = 6000
  self.dragCoeff = 0.42
  self.rollingRes = 0.015 * gravity * self.mass
  
  self.speedZeroThreshold = 0.02
  
  -- Initialize state variables
  self.throttle = 0
  self.brake = 0
  self.steering = 0
  
  self.frontWheelAngV = 0
  self.rearWheelAngV = 0
  
  self.rpm = self.idleRpm
  self.gear = 1
  self.gearShiftDelay = 0
  
  -- Set up physics
  self.body = love.physics.newBody(world, x, y, "dynamic")
  self.shape = love.physics.newRectangleShape(self.length, self.width)
  local density = (love.physics.getMeter()^2 * self.mass) / (self.length * self.width)
  self.fixture = love.physics.newFixture(self.body, self.shape, density)
  self.fixture:setRestitution(0.05)
  
  -- Traction and cornering force constants
  self.idealSlipRatio = 0.054
  self.tractionForceFalloff = -0.009
  self.idealSideSlipAngle = 0.0413
  self.corneringForceFalloff = -0.415
  
  
  
  -- Set up tires
  self.frontTire = Tire(self.body:getX() + self.wheelbase/2, self.body:getY(), 20, 0.34, 0.6, 1.17, 0.5, 1.6, 0.97, 20, 1.8, 0.97)
  self.rearTire = Tire(self.body:getX() - self.wheelbase/2, self.body:getY(), 20, 0.34, 0.6, 1.17, 0.5, 1.6, 0.97, 20, 1.8, 0.97)
  self.frontTireJoint = love.physics.newRevoluteJoint(self.body, self.frontTire.body, self.body:getX() + self.wheelbase/2, self.body:getY(), false)
  self.rearTireJoint = love.physics.newRevoluteJoint(self.body, self.rearTire.body, self.body:getX() - self.wheelbase/2, self.body:getY(), false)
  
end


--[[ PlayerCar:update
  Updates state of car for current program cycle.
  
  dt: Time in seconds since last program cycle.
--]]
function PlayerCar:update(dt)
  
  self:processInputs(dt)
  
  local ux, uy = PhysicsHelper.getUnitFacingVector(self.body)
  local vx, vy = self.body:getLinearVelocity()
  local forwardSpeed, lateralSpeed = PhysicsHelper.getRelativeSpeed(self.body)
  
  -- Prevent zero crossing errors
  if math.abs(PhysicsHelper.getSpeed(self.body)) < self.speedZeroThreshold and self.throttle == 0 then
    self.body:setLinearVelocity(0, 0)
    self.body:setAngularVelocity(0)
    self.frontWheelAngV = 0
    self.rearWheelAngV = 0
    return
  end
  
  
  -- Acceleration
  local engineTorque = 0
  
  if self.gearShiftDelay <= 0 then
    
    local driveWheelSpeed = 0
    if self.frontWheelDrive and self.rearWheelDrive then
      driveWheelSpeed = (self.frontWheelAngV + self.rearWheelAngV) / 2
    elseif self.frontWheelDrive then
      driveWheelSpeed = self.frontWheelAngV
    elseif self.rearWheelDrive then
      driveWheelSpeed = self.rearWheelAngV
    end
    
    local clutchOutputRpm = self.finalDrive * self.gearRatios[self.gear + 1] * driveWheelSpeed * (30/math.pi)
    self.rpm = math.max(math.min(clutchOutputRpm, self.redlineRpm), self.idleRpm)  
    engineTorque = self.throttle * self.driveEfficiency * self:torqueCurveLookup(clutchOutputRpm)
    
  end
  
  local accelTorque = engineTorque * self.finalDrive * self.gearRatios[self.gear + 1]
  
  local frontAccelTorque = 0
  local rearAccelTorque = 0
  if self.frontWheelDrive and self.rearWheelDrive then
    frontAccelTorque = accelTorque / 2
    rearAccelTorque = accelTorque / 2
  elseif self.frontWheelDrive then
    frontAccelTorque = accelTorque
  elseif self.rearWheelDrive then
    rearAccelTorque = accelTorque
  end
  
  
  -- Braking
  local brakeTorque = -self.brake * self.brakeTorque
  local frontBrakeTorque = brakeTorque / 2
  local rearBrakeTorque = brakeTorque / 2
  if self.frontWheelAngV < 0 then frontBrakeTorque = -frontBrakeTorque end
  if self.rearWheelAngV < 0 then rearBrakeTorque = -rearBrakeTorque end
  
  
  -- Traction force
  local frontSlipRatio = 0
  local rearSlipRatio = 0
  
  if forwardSpeed ~= 0 or self.frontWheelAngV ~= 0 then
    frontSlipRatio = (self.wheelRadius * self.frontWheelAngV - forwardSpeed) / math.abs(forwardSpeed)
  end
  if forwardSpeed ~= 0 or self.rearWheelAngV ~= 0 then
    rearSlipRatio = (self.wheelRadius * self.rearWheelAngV - forwardSpeed) / math.abs(forwardSpeed)
  end
  
  local frontWheelLoad = gravity * self.mass / 2
  local rearWheelLoad = gravity * self.mass / 2
  
  local frontTractionForce = self:computeTractionForce(frontSlipRatio, frontWheelLoad, self.tireMu)
  local rearTractionForce = self:computeTractionForce(rearSlipRatio, rearWheelLoad, self.tireMu)
  
  local frontTractionTorque = -frontTractionForce * self.wheelRadius
  local rearTractionTorque = -rearTractionForce * self.wheelRadius
  
  local tractionForce = frontTractionForce + rearTractionForce
  local tractionForceX = tractionForce * ux
  local tractionForceY = tractionForce * uy
  
  
  -- Update wheel angular velocity
  local frontWheelTorque = frontAccelTorque + frontBrakeTorque + frontTractionTorque
  local rearWheelTorque = rearAccelTorque + rearBrakeTorque + rearTractionTorque
  
  self.frontWheelAngV = self.frontWheelAngV + (frontWheelTorque * dt / self.frontAngInertia)
  self.rearWheelAngV = self.rearWheelAngV + (rearWheelTorque * dt / self.rearAngInertia)
  
  
  -- Drag and rolling resistance
  local dragForceX = -self.dragCoeff * vx * PhysicsHelper.getSpeed(self.body)
  local dragForceY = -self.dragCoeff * vy * PhysicsHelper.getSpeed(self.body)
  
  local rollResForce = 0
  if forwardSpeed > 0 then
    rollResForce = -self.rollingRes
  elseif forwardSpeed < 0 then
    rollResForce = self.rollingRes
  end
  local rollResForceX = rollResForce * ux
  local rollResForceY = rollResForce * uy
  
  
  -- Apply net forces
  local netForceX = tractionForceX + rollResForceX + dragForceX
  local netForceY = tractionForceY + rollResForceY + dragForceY
  
  self.body:applyForce(netForceX, netForceY)
  
  
  -- Cornering
  local wheelsCenterDist = self.wheelbase / 2
  local wheelRelativeSpeed = self.body:getAngularVelocity() * wheelsCenterDist
  
  local frontWheelLatSpeed = lateralSpeed + wheelRelativeSpeed
  local rearWheelLatSpeed = lateralSpeed - wheelRelativeSpeed
  
  local frontSideSlip = math.atan2(frontWheelLatSpeed, math.abs(forwardSpeed))
  local rearSideSlip = math.atan2(rearWheelLatSpeed, math.abs(forwardSpeed))
  
  -- Scale steering such that peak cornering force can be reached
  local scaledMaxSteeringAngle = math.abs(math.atan2(frontWheelLatSpeed, math.abs(forwardSpeed))) + self.idealSideSlipAngle
  if scaledMaxSteeringAngle > self.maxSteeringAngle then scaledMaxSteeringAngle = self.maxSteeringAngle end
  
  local steeringAngle = self.steering * scaledMaxSteeringAngle
  
  if forwardSpeed > 0 then
    frontSideSlip = frontSideSlip - steeringAngle
  elseif forwardSpeed < 0 then
    frontSideSlip = frontSideSlip + steeringAngle
  end
  
  local frontCorneringForce = self:computeCorneringForce(frontSideSlip, frontWheelLoad, self.tireMu) * math.cos(steeringAngle)
  local rearCorneringForce = self:computeCorneringForce(rearSideSlip, rearWheelLoad, self.tireMu)
  
  
  
  -- Apply cornering force to tires until tires properly implemented
  self.frontTire.body:applyForce(frontCorneringForce*uy, frontCorneringForce*-ux)
  self.rearTire.body:applyForce(rearCorneringForce*uy, rearCorneringForce*-ux)
  
  -- Tires
  self.frontTire.body:setAngle(self.body:getAngle() + steeringAngle)
  self.rearTire.body:setAngle(self.body:getAngle())
  
  self.frontTire:update(frontWheelLoad, frontAccelTorque, frontBrakeTorque, dt)
  self.rearTire:update(rearWheelLoad, rearAccelTorque, rearBrakeTorque, dt)
  
end


--[[ PlayerCar:draw
  Draws the car in its current position.
--]]
function PlayerCar:draw()
  
  love.graphics.setColor(1, 1, 1)
  self.image:draw(self.body:getX()*pxPerMtr, self.body:getY()*pxPerMtr, self.body:getAngle())
  self.frontTire:draw()
  self.rearTire:draw()
  
end


--[[ PlayerCar:torqueCurveLookup
  Returns the torque produced by the car's engine at a given RPM.
  
  rpm: RPM for which a torque value should be returned.
--]]
function PlayerCar:torqueCurveLookup(rpm)
  
  if rpm < self.idleRpm then
    
    -- Below idle RPM, scale torque down to simulate clutch slip
    local slipFactor = (0.5 * rpm / self.idleRpm) + 0.5
    return slipFactor * self:torqueCurveLookup(self.idleRpm)
    
  elseif rpm >= self.redlineRpm then
    
    -- Fuel cutoff
    return 0
    
  else
    
    -- Linear interpolation between nearest values in torque curve map
    local lowerRpmRank = math.floor((rpm - self.idleRpm) / self.torqueStep)
    
    local x0 = self.torqueCurve[lowerRpmRank][1]
    local y0 = self.torqueCurve[lowerRpmRank][2]
    local x1 = self.torqueCurve[lowerRpmRank + 1][1]
    local y1 = self.torqueCurve[lowerRpmRank + 1][2]
    
    return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0)
    
  end
  
end


--[[ PlayerCar:computeTractionForce
  Returns the traction force for a given slip ratio and wheel load.
  Traction force increases rapidly with magnitude of slip ratio up to a point,
  then drops off gradually.
  
  slipRatio: Slip ratio of the tire.
  tireLoad: Load in Newtons on the tire.
  tireMu: Tire friction coefficient.
--]]
function PlayerCar:computeTractionForce(slipRatio, tireLoad, tireMu)
  
  local loadFactor = 0
  
  if slipRatio < -self.idealSlipRatio then
    loadFactor = (self.tractionForceFalloff * slipRatio) - 1
    if loadFactor > -0.5 then loadFactor = -0.5 end
  elseif slipRatio <= self.idealSlipRatio then
     loadFactor = slipRatio / self.idealSlipRatio
  else
    loadFactor = (self.tractionForceFalloff * slipRatio) + 1
    if loadFactor < 0.5 then loadFactor = 0.5 end
  end
  
  return loadFactor * tireLoad * tireMu
  
end


--[[ PlayerCar:computeCorneringForce
  Returns the cornering force for a given sideslip angle and wheel load.
  Cornering force increases rapidly with magnitude of sideslip angle up to a point,
  then drops off gradually.
  
  sideSlipAngle: Sideslip angle of the tire in radians.
  tireLoad: Load in Newtons on the tire.
  tireMu: Tire friction coefficient.
--]]
function PlayerCar:computeCorneringForce(sideSlipAngle, tireLoad, tireMu)
  
  local loadFactor = 0
  
  if sideSlipAngle < -self.idealSideSlipAngle then
    loadFactor = (self.corneringForceFalloff * sideSlipAngle) - 1
    if loadFactor > -0.5 then loadFactor = -0.5 end
  elseif sideSlipAngle <= self.idealSideSlipAngle then
     loadFactor = sideSlipAngle / self.idealSideSlipAngle
  else
    loadFactor = (self.corneringForceFalloff * sideSlipAngle) + 1
    if loadFactor < 0.5 then loadFactor = 0.5 end
  end
  
  return loadFactor * tireLoad * tireMu
  
end


--[[ PlayerCar:processInputs
  Handles processing of player inputs.
  
  dt: Time in seconds since last program cycle.
--]]
function PlayerCar:processInputs(dt)
  
  -- Accelerator
  if love.keyboard.isDown("up") then
    self.throttle = self.throttle + 2*dt
  else
    self.throttle = self.throttle - 2*dt
  end
  
  if self.throttle < 0 then self.throttle = 0 end
  if self.throttle > 1 then self.throttle = 1 end
  
  -- Brake
  if love.keyboard.isDown("down") then
    self.brake = self.brake + 4*dt
  else
    self.brake = self.brake - 4*dt
  end
  
  if self.brake < 0 then self.brake = 0 end
  if self.brake > 1 then self.brake = 1 end
  
  -- Steering
  local noSteerInput = true
  if love.keyboard.isDown("left") then
    noSteerInput = false
    self.steering = self.steering - 3*dt
  end
  if love.keyboard.isDown("right") then
    noSteerInput = false
    self.steering = self.steering + 3*dt
  end
  
  if self.steering < -1 then self.steering = -1 end
  if self.steering > 1 then self.steering = 1 end
  
  if noSteerInput == true then
    if self.steering > 0.05 then
      self.steering = self.steering - 3*dt
    elseif self.steering < -0.05 then
      self.steering = self.steering + 3*dt
    else
      self.steering = 0
    end
  end
  
  -- Gears
  if self.gearShiftDelay > 0 then
    self.gearShiftDelay = self.gearShiftDelay - dt
  elseif love.keyboard.isDown("x") and self.gear < #self.gearRatios - 1 then
    self.gear = self.gear + 1
    self.gearShiftDelay = self.gearShiftTime
  elseif love.keyboard.isDown("z") and self.gear > 0 then
    self.gear = self.gear - 1
    self.gearShiftDelay = self.gearShiftTime
  end
  
  -- Reset
  if love.keyboard.isDown("r") then self:reset() end
  
end


--[[ PlayerCar:reset
  Resets the car back to default position and state.
--]]
function PlayerCar:reset()
  
  self.throttle = 0
  self.brake = 0
  self.steering = 0
  
  self.rpm = self.idleRpm
  self.gear = 1
  self.gearShiftDelay = 0
  
  self.frontWheelAngV = 0
  self.rearWheelAngV = 0
  
  self.body:setAngle(0)
  self.body:setAngularVelocity(0)
  self.body:setPosition(0, 0)
  self.body:setLinearVelocity(0, 0)
  
end

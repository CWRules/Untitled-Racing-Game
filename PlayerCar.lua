
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
  self.speedZeroThreshold = 0.02
  
  -- Set up physics
  self.body = love.physics.newBody(world, x, y, "dynamic")
  self.shape = love.physics.newRectangleShape(self.length, self.width)
  local density = (love.physics.getMeter()^2 * self.mass) / (self.length * self.width)
  self.fixture = love.physics.newFixture(self.body, self.shape, density)
  self.fixture:setRestitution(0.05)
  
  -- Engine and drivetrain
  self.idleRpm = 1200
  self.redlineRpm = 6000
  local torqueValues = {297.09, 304.16, 310.24, 316.21, 321.50, 325.36, 328.32, 331.21, 332.89, 334.79,
                       337.73, 340.97, 346.18, 349.93, 352.09, 353.66, 352.89, 353.30, 351.66, 346.61,
                       338.96, 329.55, 311.85, 294.41, 274.84}
  
  self.numTorqueSteps = #torqueValues - 1
  self.torqueStep = (self.redlineRpm - self.idleRpm) / self.numTorqueSteps
  local ftLbToNm = 1.3558
  
  self.torqueCurve = {}
  for i = 0, self.numTorqueSteps do
    self.torqueCurve[i] = {self.idleRpm + (i * self.torqueStep), ftLbToNm * torqueValues[i + 1]}
  end
  
  self.gearRatios = {-2.90, 2.66, 1.78, 1.30, 1.00, 0.74, 0.50}
  self.finalDrive = 3.42
  self.driveEfficiency = 0.85
  self.gearShiftTime = 0.7
  
  self.frontWheelDrive = false
  self.rearWheelDrive = true
  
  -- Compute angular inertia of drivetrain + wheels
  local wheelMass = 10
  local wheelRadius = 0.34
  local drivelineEffectiveMass = 5 -- VERY rough estimate
  local frontEffectiveWheelMass = 2*wheelMass
  local rearEffectiveWheelMass = 2*wheelMass
  
  if self.frontWheelDrive then
    frontEffectiveWheelMass = frontEffectiveWheelMass + drivelineEffectiveMass
  end
  if self.rearWheelDrive then
    rearEffectiveWheelMass = rearEffectiveWheelMass + drivelineEffectiveMass
  end
  
  -- Tires
  self.frontTire = Tire(self.body:getX() + self.wheelbase/2, self.body:getY(), frontEffectiveWheelMass, wheelRadius, 0.6, 1.17, 0.5, 1.6, 0.97, 20, 1.8, 0.97)
  self.rearTire = Tire(self.body:getX() - self.wheelbase/2, self.body:getY(), rearEffectiveWheelMass, wheelRadius, 0.6, 1.17, 0.5, 1.6, 0.97, 20, 1.8, 0.97)
  self.frontTireJoint = love.physics.newRevoluteJoint(self.body, self.frontTire.body, self.body:getX() + self.wheelbase/2, self.body:getY(), false)
  self.rearTireJoint = love.physics.newRevoluteJoint(self.body, self.rearTire.body, self.body:getX() - self.wheelbase/2, self.body:getY(), false)
  
  self.brakeTorque = 6000
  self.dragCoeff = 0.42
  self.rollingRes = 0.015 * gravity * self.mass
  
  -- Initialize state variables
  self.throttle = 0
  self.brake = 0
  self.steering = 0
  self.rpm = self.idleRpm
  self.gear = 1
  self.gearShiftDelay = 0
  
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
    self.frontTire.angVelocity = 0
    self.frontTire.angVelocity = 0
    return
  end
  
  -- Acceleration
  local engineTorque = 0
  
  if self.gearShiftDelay <= 0 then
    
    local driveWheelSpeed = 0
    if self.frontWheelDrive and self.rearWheelDrive then
      driveWheelSpeed = (self.frontTire.angVelocity + self.rearTire.angVelocity) / 2
    elseif self.frontWheelDrive then
      driveWheelSpeed = self.frontTire.angVelocity
    elseif self.rearWheelDrive then
      driveWheelSpeed = self.rearTire.angVelocity
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
  if self.frontTire.angVelocity < 0 then frontBrakeTorque = -frontBrakeTorque end
  if self.rearTire.angVelocity < 0 then rearBrakeTorque = -rearBrakeTorque end
  
  -- Steering
  -- Scale based on peak cornering force
  local maxForceAngle = 0
  if self.steering > 0 then
    maxForceAngle = PhysicsHelper.getMoveAngle(self.frontTire.body) + self.frontTire.idealSlipLat
  elseif self.steering < 0 then
    maxForceAngle = PhysicsHelper.getMoveAngle(self.frontTire.body) - self.frontTire.idealSlipLat
  end
  
  local carAngle = math.atan2(uy, ux)  
  local scaledMaxSteeringAngle = maxForceAngle - carAngle
  
  if scaledMaxSteeringAngle > math.pi then
    scaledMaxSteeringAngle = scaledMaxSteeringAngle - 2*math.pi
  elseif scaledMaxSteeringAngle < -math.pi then
    scaledMaxSteeringAngle = scaledMaxSteeringAngle + 2*math.pi
  end
  
  scaledMaxSteeringAngle = math.abs(scaledMaxSteeringAngle)
  if scaledMaxSteeringAngle > self.maxSteeringAngle then
    scaledMaxSteeringAngle = self.maxSteeringAngle
  end
  
  local steeringAngle = self.steering * scaledMaxSteeringAngle
  self.frontTire.body:setAngle(self.body:getAngle() + steeringAngle)
  self.rearTire.body:setAngle(self.body:getAngle())
  
  -- Traction forces
  local frontWheelLoad = gravity * self.mass / 2
  local rearWheelLoad = gravity * self.mass / 2
  self.frontTire:update(frontWheelLoad, frontAccelTorque, frontBrakeTorque, dt)
  self.rearTire:update(rearWheelLoad, rearAccelTorque, rearBrakeTorque, dt)
  
  
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
  
  self.body:applyForce(rollResForceX + dragForceX, rollResForceY + dragForceY)
  
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
    self.steering = self.steering - 2*dt
  end
  if love.keyboard.isDown("right") then
    noSteerInput = false
    self.steering = self.steering + 2*dt
  end
  
  if self.steering < -1 then
    self.steering = -1
  elseif self.steering > 1 then
    self.steering = 1
  end
  
  if noSteerInput == true then
    if self.steering > 0.05 then
      self.steering = self.steering - 2*dt
    elseif self.steering < -0.05 then
      self.steering = self.steering + 2*dt
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

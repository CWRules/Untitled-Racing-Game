
PlayerCar = Object:extend()

function PlayerCar:new(x, y)
  
  -- Define image
  self.image = love.graphics.newImage("images/CorvetteC5.png")
  self.origin_x = self.image:getWidth() / 2
  self.origin_y = self.image:getHeight() / 2
  
  -- Define attributes
  self.length = self.image:getWidth()
  self.width = self.image:getHeight()
  self.maxSteeringAngle = 30 * math.pi/180
  self.wheelbase = 2.65 * love.physics.getMeter()
  self.mass = 1500
  
  self.idleRpm = 1200
  self.redlineRpm = 6000
  self.torqueValues = {297.09, 304.16, 310.24, 316.21, 321.50, 325.36, 328.32, 331.21, 332.89, 334.79,
                       337.73, 340.97, 346.18, 349.93, 352.09, 353.66, 352.89, 353.30, 351.66, 346.61,
                       338.96, 329.55, 311.85, 294.41, 274.84}
  
  self.numTorqueSteps = #self.torqueValues - 1
  self.torqueStep = (self.redlineRpm - self.idleRpm) / self.numTorqueSteps
  local ftLbToNm = 1.3558
  
  self.torqueCurve = {}
  for i = 0, self.numTorqueSteps do
    self.torqueCurve[i] = {self.idleRpm + (i * self.torqueStep), ftLbToNm * self.torqueValues[i + 1]}
  end
  
  self.gearRatios = {-2.90, 2.66, 1.78, 1.30, 1.00, 0.74, 0.50}
  self.finalDrive = 3.42
  self.wheelRadius = 0.34
  self.driveEfficiency = 0.85
  self.gearShiftTime = 0.7
  
  self.brakeTorque = 5000
  
  self.cDrag = 0.42
  self.rollingRes = 0.015 * gravity * self.mass
  
  self.speedZeroThreshold = 0.2
  
  -- Initialize state variables
  self.throttle = 0
  self.brake = 0
  self.steering = 0
  
  self.rpm = self.idleRpm
  self.gear = 1
  self.gearShiftDelay = 0
  
  self.rearWheelAngV = 0
  
  -- Set up physics
  self.body = love.physics.newBody(world, x, y, "dynamic", 1)
  self.shape = love.physics.newRectangleShape(self.length, self.width)
  local density = (love.physics.getMeter()^2 * self.mass) / (self.length * self.width)
  self.fixture = love.physics.newFixture(self.body, self.shape, density)
  self.fixture:setRestitution(0.05)
  
end


function PlayerCar:update(dt)
  
  self:processInputs(dt)
  
  -- Frequently accessed values
  local ux = math.cos(self.body:getAngle())
  local uy = math.sin(self.body:getAngle())
  local vx, vy = self.body:getLinearVelocity()
  local speed = self:getSpeed()
  local forwardSpeed = self:getForwardSpeed()
  
  -- Acceleration
  local engineTorque = 0
  if self.gearShiftDelay <= 0 then
    local clutchOutputRpm = self.finalDrive * math.abs(self.gearRatios[self.gear + 1]) * speed * (30/math.pi) / self.wheelRadius
    self.rpm = math.max(math.min(clutchOutputRpm, self.redlineRpm), self.idleRpm)  
    engineTorque = self.throttle * self.driveEfficiency * self:torqueCurveLookup(clutchOutputRpm)
  end
  
  local accelTorque = engineTorque * self.finalDrive * self.gearRatios[self.gear + 1]
  local accelForce = accelTorque / self.wheelRadius
  
  -- Braking and rolling resistance
  local brakeTorque = -self.brake * self.brakeTorque
  local brakeForce = brakeTorque / self.wheelRadius
  local rollResForce = -self.rollingRes
  
  -- Flip braking and rolling resistance forces if necessary
  if forwardSpeed < 0 then
    brakeForce = -brakeForce
    rollResForce = -rollResForce
  end
  
  -- Calculate total linear forces
  local dragForceX = 0
  local dragForceY = 0
  local tractionForce = 0
  
  -- Calculate traction force from slip ratio TODO
  local slipRatio = (self.wheelRadius * self.rearWheelAngV - forwardSpeed) / math.abs(forwardSpeed)
  
  if math.abs(forwardSpeed) < self.speedZeroThreshold and accelForce == 0 then
    -- Prevent zero crossing
    tractionForce = 0
    self.body:setLinearVelocity(0, 0)
  else
    tractionForce = accelForce + brakeForce + rollResForce
    dragForceX = -self.cDrag * vx * speed
    dragForceY = -self.cDrag * vy * speed
  end
  
  local tractionForceX = tractionForce * ux
  local tractionForceY = tractionForce * uy
  
  -- Cornering
  -- Scale steering angle based on speed
  local steeringAngle = self.steering * self.maxSteeringAngle / (0.05 * math.abs(forwardSpeed) + 1)
  local steeringRadius = self.wheelbase / math.sin(steeringAngle)
  
  local steeringForceX = 0
  local steeringForceY = 0
  
  -- Compute steering force
  if steeringAngle ~= 0 and math.abs(forwardSpeed) > self.speedZeroThreshold then
    local steeringForce = self.mass * forwardSpeed^2 / math.abs(steeringRadius)
    local steeringForceAngle = 0
    
    -- Rotate force to be at 90 degrees to the facing of the front wheels
    if forwardSpeed > 0 then
      if self.steering < 0 then
        steeringForceAngle = steeringAngle - math.pi/2
      else
        steeringForceAngle = steeringAngle + math.pi/2
      end
    else
      if self.steering < 0 then
        steeringForceAngle = -steeringAngle - math.pi/2
      else
        steeringForceAngle = -steeringAngle + math.pi/2
      end
    end
    
    steeringForceX = math.cos(steeringForceAngle)*steeringForce*ux - math.sin(steeringForceAngle)*steeringForce*uy
    steeringForceY = math.sin(steeringForceAngle)*steeringForce*ux + math.cos(steeringForceAngle)*steeringForce*uy
    
    -- Set angle to match direction of travel (or directly opposite if closer)
    local angleDiff = math.acos((ux*vx + uy*vy) / speed)
    
    if math.abs(angleDiff) < math.pi/2 then
      self.body:setAngle(math.atan2(vy, vx))
    else
      self.body:setAngle(math.atan2(-vy, -vx))
    end
  end
  
  -- Apply forces
  local netForceX = tractionForceX + dragForceX + steeringForceX
  local netForceY = tractionForceY + dragForceY + steeringForceY
  
  self.body:applyForce(netForceX, netForceY)
  
end


function PlayerCar:draw()
  
  love.graphics.setColor(255, 255, 255)
  love.graphics.draw(self.image, self.body:getX(), self.body:getY(), self.body:getAngle(), 1, 1, self.origin_x, self.origin_y)
  
  -- Debug info
  love.graphics.setColor(0, 0, 0)
  love.graphics.print(string.format("thr, brk, str: %.2f, %.2f, %.2f", self.throttle, self.brake, self.steering), 20, 20)
  love.graphics.print(string.format("Speed: %.2f m/s / %.1f mph", self:getForwardSpeed(), 2.237 * self:getForwardSpeed()), 20, 35)
  
  local gearString = "-"
  if self.gearShiftDelay <= 0 then
    if self.gear == 0 then
      gearString = "R"
    else
      gearString = tostring(self.gear)
    end
  end
  love.graphics.print(string.format("Gear: %s", gearString), 20, 50)
  
  local rpmString = tostring(math.floor(self.rpm))
  if self.redlineRpm - self.rpm < 100 then
    rpmString = rpmString .. " ***"
  elseif self.redlineRpm - self.rpm < 300 then
    rpmString = rpmString .. " **"
  elseif self.redlineRpm - self.rpm < 500 then
    rpmString = rpmString .. " *"
  end
  love.graphics.print(string.format("RPM: %s", rpmString), 20, 65)
  
end


function PlayerCar:getSpeed()
  
  local vx, vy = self.body:getLinearVelocity()
  return math.sqrt(vx^2 + vy^2)
  
end


function PlayerCar:getForwardSpeed()
  
  local vx, vy = self.body:getLinearVelocity()
  local vAngle = math.atan2(vy, vx)
  return math.cos(vAngle - self.body:getAngle()) * math.sqrt(vx^2 + vy^2)
  
end


function PlayerCar:torqueCurveLookup(rpm)
  
  if rpm < self.idleRpm then
    -- Calculate effect of clutch slip and return idle torque times loss from slip
    local deltaRpm = self.idleRpm - rpm
    local slipFactor = -0.0002 * deltaRpm + 1
    return slipFactor * self:torqueCurveLookup(self.idleRpm)
  elseif rpm >= self.redlineRpm then
    -- Fuel cutoff
    return 0
  else
    -- Linear interpolation between nearest values
    local lowerRpmRank = math.floor((rpm - self.idleRpm) / self.torqueStep)
    
    local x0 = self.torqueCurve[lowerRpmRank][1]
    local y0 = self.torqueCurve[lowerRpmRank][2]
    local x1 = self.torqueCurve[lowerRpmRank + 1][1]
    local y1 = self.torqueCurve[lowerRpmRank + 1][2]
    
    return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0)
  end
  
end


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


function PlayerCar:reset()
  
  self.throttle = 0
  self.brake = 0
  self.steering = 0
  self.rpm = self.idleRpm
  self.gear = 1
  self.gearShiftDelay = 0
  
  self.body:setAngle(0)
  self.body:setAngularVelocity(0)
  self.body:setPosition(50, maxY - 50)
  self.body:setLinearVelocity(0, 0)
  
end

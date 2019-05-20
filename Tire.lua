
--[[ Tire
  Class defining the functionality of a wheel and tire.
--]]
Tire = Object:extend()

--[[ Tire:new
  Tire constructor.
  
  x: X coordinate to initialize the tire at.
  y: Y coordinate to initialize the tire at.
  mass: Mass of the wheel and tire.
  radius: Radius of the tire.
  width: Width of the tire.
  friction: Coefficient of friction (Pacejka Magic Formula coefficient D).
  
  longStiffness: Pacejka Magic Formula coefficient B (longitudinal).
  longShape: Pacejka Magic Formula coefficient C (longitudinal).
  longCurvature: Pacejka Magic Formula coefficient E (longitudinal).
  
  latStiffness: Pacejka Magic Formula coefficient B (lateral).
  latShape: Pacejka Magic Formula coefficient C (lateral).
  latCurvature: Pacejka Magic Formula coefficient E (lateral).
--]]
function Tire:new(x, y, mass, radius, width, friction, longStiffness, longShape, longCurvature, latStiffness, latShape, latCurvature)
  
  -- Define attributes
  self.radius = radius
  self.width = width
  self.height = radius*2
  self.angInertia = mass * radius^2
  self.friction = friction
  
  -- Pacejka Magic Formula coefficients
  self.longStiffness = longStiffness
  self.longShape = longShape
  self.longCurvature = longCurvature
  self.latStiffness = latStiffness
  self.latShape = latShape
  self.latCurvature = latCurvature
  
  self.idealSlipLat = self:idealSlip('lat')
  self.idealSlipLong = self:idealSlip('long')
  
  -- Set up physics
  self.body = love.physics.newBody(world, x, y, "dynamic")
  self.shape = love.physics.newRectangleShape(self.height, self.width)
  self.fixture = love.physics.newFixture(self.body, self.shape, 0)
  
  -- Create image
  local canvas = love.graphics.newCanvas(self.width*pxPerMtr, self.height*pxPerMtr)
  love.graphics.setCanvas(canvas)
    love.graphics.setColor(0, 1, 0)
    love.graphics.rectangle("fill", 0, 0, self.width*pxPerMtr, self.height*pxPerMtr)
  love.graphics.setCanvas()
  self.image = Sprite(canvas)
  
  -- Initialize state variables
  self.angVelocity = 0
  
end

--[[ Tire:update
  Updates state of tire for current program cycle.
  
  wheelLoad: Vertical load on the tire.
  accelTorque: Torque applied to the wheel from acceleration.
  brakeTorque: Torque applied to the wheel from braking.
  dt: Time in seconds since last program cycle.
--]]
function Tire:update(wheelLoad, accelTorque, brakeTorque, dt)
  
  local ux, uy = PhysicsHelper.getUnitFacingVector(self.body)
  local forwardSpeed, lateralSpeed = PhysicsHelper.getRelativeSpeed(self.body)
  
  -- Compute slip ratio and sideslip angle
  local slipRatio = 0
  if forwardSpeed ~= 0 then
    slipRatio = (self.radius * self.angVelocity - forwardSpeed) / math.abs(forwardSpeed)
  elseif self.angVelocity ~= 0 then
    slipRatio = self.angVelocity / math.abs(self.angVelocity)
  end
  
  local sideSlip = math.atan2(lateralSpeed, math.abs(forwardSpeed))
  
  -- Traction forces
  local longForce = PhysicsHelper.pacejka(wheelLoad, slipRatio, self.longStiffness, self.longShape, self.friction, self.longCurvature)
  local latForce = PhysicsHelper.pacejka(wheelLoad, sideSlip, self.latStiffness, self.latShape, self.friction, self.latCurvature)
  
  -- Update angular velocity
  local wheelTorque = accelTorque + brakeTorque - (longForce * self.radius)
  self.angVelocity = self.angVelocity + (wheelTorque * dt / self.angInertia)
  
  -- Apply forces
  local tractionForceX = (longForce * ux) + (latForce * uy)
  local tractionForceY = (longForce * uy) + (latForce * -ux)
  self.body:applyForce(tractionForceX, tractionForceY)
  
end


--[[ Tire:draw
  Draws the tire in its current position.
--]]
function Tire:draw()
  
  love.graphics.setColor(1, 1, 1)
  self.image:draw(self.body:getX()*pxPerMtr, self.body:getY()*pxPerMtr, self.body:getAngle())
  
end


--[[ Tire:idealSlip
  Returns the slip ratio or sideslip angle that gives the maximum traction force.
  Uses Newton's Method to find point where derivative of Magic Formula is zero.
  Adapted from LuaMath package here: https://github.com/aryajur/LuaMath
  
  direction: 'long' or 'lat'
--]]
function Tire:idealSlip(direction)
  
  local B, C, E
  if direction == 'long' then
    B = self.longStiffness
    C = math.tan(math.pi/(2*self.longShape))
    E = self.longCurvature
  elseif direction == 'lat' then
    B = self.latStiffness
    C = math.tan(math.pi/(2*self.latShape))
    E = self.latCurvature
  end
  
  -- First and second derivatives of Pacejka Magic Formula
  -- Simplified to remove unecessary terms
  local function f(x) return E*math.atan(B*x) + (1 - E)*B*x - C end
  local function fp(x) return B*E*(1/((B^2)*(x^2) + 1) - 1) + B end
  
  local x = 0
  local maxError = 0.001
  
  for i=0, 100 do
    x = x - (f(x)/fp(x))
    local err = f(x)
    local iter = i
    if math.abs(err) <= maxError then
      return x
    end
  end
  return x
  
end

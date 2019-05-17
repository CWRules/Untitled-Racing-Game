
--[[ Tire
  Class defining the functionality of a wheel and tire.
--]]
Tire = Object:extend()

--[[ Tire:new
  Tire constructor.
  
  x: X coordinate to initialize the tire at.
  y: Y coordinate to initialize the tire at.
  mass: Mass of the wheel and tire in kg.
  radius: Radius of the tire in meters.
  width: Width of the tire in meters.
--]]
function Tire:new(x, y, mass, radius, width)
  
  self.radius = radius
  self.width = width
  self.height = radius*2
  self.angInertia = mass * radius^2
  self.angVelocity = 0
  
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
  
end

--[[ Tire:update
  Updates state of tire for current program cycle.
  
  torque: Total torque applied to the wheel.
  dt: Time in seconds since last program cycle.
--]]
function Tire:update(torque, dt)
  
  -- Compute slip ratio and sideslip angle
  -- Update angular velocity
  -- Compute lateral traction forces using Pacejka Magic Formula
  -- Deal with zero-crossing
  
  -- Slip ratio
  --if forwardSpeed ~= 0 or self.frontWheelAngV ~= 0 then
  --  frontSlipRatio = (self.wheelRadius * self.frontWheelAngV - forwardSpeed) / math.abs(forwardSpeed)
  --end
  
end


--[[ Tire:draw
  Draws the tire in its current position.
--]]
function Tire:draw()
  
  love.graphics.setColor(1, 1, 1)
  self.image:draw(self.body:getX()*pxPerMtr, self.body:getY()*pxPerMtr, self.body:getAngle())
  
end

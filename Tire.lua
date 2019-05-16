
--[[ Tire
  Class defining the functionality of a wheel and tire.
--]]
Tire = Object:extend()

--[[ Tire:new(x, y)
  Tire constructor.
  
  x: X coordinate to initialize the tire at.
  y: Y coordinate to initialize the tire at.
  mass: Mass of the wheel and tire in kg.
  radius: Radius of the tire in meters.
  width: Width of the tire in meters.
--]]
function Tire:new(x, y, mass, radius, width)
  
  self.angVelocity = 0
  self.radius = radius
  self.width = width
  self.height = radius*2
  self.angInertia = mass * radius^2
  
  self.body = love.physics.newBody(world, x, y, "dynamic")
  self.shape = love.physics.newRectangleShape(self.height, self.width)
  self.fixture = love.physics.newFixture(self.body, self.shape, 0)
  
  -- Create image
  self.image = love.graphics.newCanvas(self.width*pxPerMtr, self.height*pxPerMtr)
  love.graphics.setCanvas(self.image)
  love.graphics.setColor(0, 1, 0)
  love.graphics.rectangle('fill', 0, 0, self.width*pxPerMtr, self.height*pxPerMtr)
  love.graphics.setCanvas()
  
end

--[[ Tire:update()
  Updates state of tire for current program cycle.
  
  torque: Total torque applied to the wheel.
  dt: Time in seconds since last program cycle.
--]]
function Tire:update(torque, dt)
  
  -- Model tire as separate physics object to get position and velocity!
  -- Connect with Revolute Joints
  
  -- Compute slip ratio and sideslip angle
  -- Update angular velocity
  -- Compute lateral traction forces using Pacejka Magic Formula
  -- Deal with zero-crossing
  
end


--[[ Tire:draw(dt)
  Draws the tire in its current position.
--]]
function Tire:draw(dt)
  
  love.graphics.draw(self.image, self.body:getX()*pxPerMtr, self.body:getY()*pxPerMtr, self.body:getAngle(), 1, 1, self.image:getWidth() / 2, self.image:getHeight() / 2)
  
end
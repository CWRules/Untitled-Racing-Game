
--[[ Tire
  Class defining the functionality of a wheel and tire.
--]]
Tire = Object:extend()

--[[ Tire:new(x, y)
  Tire constructor.
  
  mass: Mass of the wheel and tire in kg.
  radius: Radius of the tire in meters.
--]]
function Tire:new(mass, radius)
  
  self.angVelocity = 0
  self.radius = radius
  self.angInertia = mass * radius^2
  
end

--[[ Tire:update()
  Updates state of tire for current program cycle.
  
  torque: Total torque applied to the wheel.
  carSpeed: Forward speed of the car.
  sideslip: Angle between the tire's direction of facing and direction of travel.
  dt: Time in seconds since last program cycle.
--]]
function Tire:update(torque, carSpeed, sideslip, dt)
  
  -- Compute slip ratio
  -- Update angular velocity
  -- Deal with zero-crossing
  -- Return longitudinal and lateral traction forces using Pacejka Magic Formula
  
end
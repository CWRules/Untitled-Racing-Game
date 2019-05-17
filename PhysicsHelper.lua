
--[[ PhysicsHelper
  Defines some useful physics functions not provided natively by Box2D.
--]]
local PhysicsHelper = {}


--[[ PhysicsHelper.getSpeed
  Returns the magnitude of an object's linear velocity.
  
  body: Body object.
--]]
function PhysicsHelper.getSpeed(body)
  
  local vx, vy = body:getLinearVelocity()
  return math.sqrt(vx^2 + vy^2)
  
end


--[[ PhysicsHelper.getRelativeSpeed
  Returns the speed of an object relative to its own facing, in the forward and lateral directions.
  
  body: Body object.
--]]
function PhysicsHelper.getRelativeSpeed(body)
  
  local vx, vy = body:getLinearVelocity()
  local vAngle = math.atan2(vy, vx)
  local theta = (math.pi / 2) + body:getAngle() - vAngle
  
  local vForward = math.cos(vAngle - body:getAngle()) * math.sqrt(vx^2 + vy^2)
  local vLateral = math.cos(theta) * math.sqrt(vx^2 + vy^2)
  
  return vForward, vLateral
  
end


--[[ PhysicsHelper.getUnitFacingVector
  Returns the unit vector corrsponding to an objects's current direction of facing.
  
  body: Body object.
--]]
function PhysicsHelper.getUnitFacingVector(body)
  
  local ux = math.cos(body:getAngle())
  local uy = math.sin(body:getAngle())
  
  return ux, uy
  
end


return PhysicsHelper

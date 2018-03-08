
--[[ love.load()
  LOVE initialization function. Run once at program start.
--]]
function love.load()
  
  Object = require "lib/classic"
  require "PlayerCar"
  --require("mobdebug").start()
  
  -- Set up window
  love.graphics.setBackgroundColor(255, 255, 255)
  pxPerMtr = 10
  maxX = love.graphics.getWidth() / pxPerMtr
  maxY = love.graphics.getHeight() / pxPerMtr
  
  -- Initialize physics
  love.physics.setMeter(1)
  world = love.physics.newWorld(0, 0, true)
  gravity = 9.81
  
  -- Declare car
  car = PlayerCar(5, maxY - 5)
  
  -- Create walls
  walls = {}
  local wallImage = love.graphics.newImage("images/Wall.png")
  local wallShape = love.physics.newRectangleShape(wallImage:getWidth() / pxPerMtr, wallImage:getHeight() / pxPerMtr)
  
  for i = 1, 3 do
    walls[i] = {}
    walls[i].image = wallImage
    walls[i].originX = wallImage:getWidth() / 2
    walls[i].originY = wallImage:getHeight() / 2
    walls[i].body = love.physics.newBody(world, 0, 0)
    walls[i].shape = wallShape
    walls[i].fixture = love.physics.newFixture(walls[i].body, walls[i].shape)
  end
  
  walls[1].body:setPosition(maxX/2, 0)
  
  walls[2].body:setPosition(maxX/2, maxY)
  
  walls[3].body:setPosition(0, maxY/2)
  walls[3].body:setAngle(math.pi/2)
  
end


--[[ love.update()
  LOVE update function. Run once each program cycle.
  
  dt: Time in seconds since last program cycle.
--]]
function love.update(dt)
  
  world:update(dt)
  car:update(dt)
  
end


--[[ love.draw()
  LOVE graphics function. Run once each program cycle after update() finishes.
--]]
function love.draw()
  
  car:draw()
  
  love.graphics.setColor(0, 0, 0)
  for k, wall in ipairs(walls) do
    love.graphics.draw(wall.image, wall.body:getX() * pxPerMtr, wall.body:getY() * pxPerMtr, wall.body:getAngle(), 1, 1, wall.originX, wall.originY)
  end
  
end

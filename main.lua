
--[[ love.load()
  LOVE initialization function. Run once at program start.
--]]
function love.load()
  
  Object = require "lib/classic"
  require "PlayerCar"
  --require("mobdebug").start()
  
  -- Set up window
  love.graphics.setBackgroundColor(255, 255, 255)
  maxX = love.graphics.getWidth()
  maxY = love.graphics.getHeight()
  
  -- Initialize physics
  love.physics.setMeter(5)
  world = love.physics.newWorld(0, 0, true)
  gravity = 9.81
  
  -- Declare car
  car = PlayerCar(50, maxY - 50)
  
  -- Create walls
  walls = {}
  
  -- North
  walls[1] = {}
  walls[1].body = love.physics.newBody(world, maxX/2, 0)
  walls[1].shape = love.physics.newRectangleShape(maxX, 20)
  walls[1].fixture = love.physics.newFixture(walls[1].body, walls[1].shape)
  
  -- South
  walls[2] = {}
  walls[2].body = love.physics.newBody(world, maxX/2, maxY)
  walls[2].shape = love.physics.newRectangleShape(maxX, 20)
  walls[2].fixture = love.physics.newFixture(walls[2].body, walls[2].shape)
  
  -- West
  walls[3] = {}
  walls[3].body = love.physics.newBody(world, 0, maxY/2)
  walls[3].shape = love.physics.newRectangleShape(20, maxY)
  walls[3].fixture = love.physics.newFixture(walls[3].body, walls[3].shape)
  
  -- East
  --walls[4] = {}
  --walls[4].body = love.physics.newBody(world, maxX, maxY/2)
  --walls[4].shape = love.physics.newRectangleShape(20, maxY)
  --walls[4].fixture = love.physics.newFixture(walls[4].body, walls[4].shape)
  
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
    love.graphics.polygon("fill", wall.body:getWorldPoints(wall.shape:getPoints()))
  end
  
end

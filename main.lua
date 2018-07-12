
--[[ love.load()
  LOVE initialization function. Run once at program start.
--]]
function love.load()
  
  Object = require "lib/classic"
  require "PlayerCar"
  require "Sprite"
  
  -- Set up window
  love.graphics.setBackgroundColor(255, 255, 255)
  pxPerMtr = 10
  maxX = love.graphics.getWidth() / pxPerMtr
  maxY = love.graphics.getHeight() / pxPerMtr
  
  -- Fonts
  fontCourier = love.graphics.newFont("cour.ttf", 12)
  love.graphics.setFont(fontCourier)
  
  -- Initialize physics
  -- Set meter size to 1, do graphics scaling manually
  love.physics.setMeter(1)
  world = love.physics.newWorld(0, 0, true)
  gravity = 9.81
  
  -- Declare car
  car = PlayerCar(5, maxY - 5)
  
  -- Create walls
  walls = {}
  local wallImage = Sprite("images/Wall.png")
  local wallShape = love.physics.newRectangleShape(wallImage.width / pxPerMtr, wallImage.height / pxPerMtr)
  
  for i = 1, 3 do
    walls[i] = {}
    walls[i].image = wallImage
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
  
  -- Move camera
  local cameraAngle = -car.body:getAngle() - math.pi/2
  local cameraOffsetX = (-car.body:getX() * pxPerMtr) - 0 --love.graphics.getHeight()/2
  local cameraOffsetY = (-car.body:getY() * pxPerMtr) - 0 --love.graphics.getWidth()/2
  
  --love.graphics.translate(cameraOffsetX, cameraOffsetY)
  --love.graphics.rotate(cameraAngle)
  
  
  -- Car
  car:draw()
  
  -- Car info
  love.graphics.setColor(0, 0, 0)
  love.graphics.print(string.format("FPS: %d", 1/love.timer.getAverageDelta()), 20, 20)
  love.graphics.print(string.format("thr, brk, str: %.2f, %.2f, % .2f/% 3.1f", car.throttle, car.brake, car.steering, d0), 20, 35)
  love.graphics.print(string.format("Speed: %.1f mph", 2.237 * car:getForwardSpeed()), 20, 50)
  
  -- Current gear
  local gearString = ""
  if car.gearShiftDelay <= 0 then
    if car.gear == 0 then
      gearString = "R"
    else
      gearString = tostring(car.gear)
    end
  else
    local gearChangeProgress = math.floor(10 * (car.gearShiftTime - car.gearShiftDelay) / car.gearShiftTime)
    for i = 1, gearChangeProgress do gearString = gearString .. "-" end
    gearString = gearString .. "|"
    for i = 1, 9 - gearChangeProgress do gearString = gearString .. "-" end
  end
  
  love.graphics.print(string.format("Gear: %s", gearString), 120, 65)
  
  -- RPM
  local rpmString = tostring(math.floor(car.rpm))
  if car.redlineRpm - car.rpm < 100 then
    rpmString = rpmString .. " ***"
  elseif car.redlineRpm - car.rpm < 300 then
    rpmString = rpmString .. " **"
  elseif car.redlineRpm - car.rpm < 500 then
    rpmString = rpmString .. " *"
  end
  love.graphics.print(string.format("RPM: %s", rpmString), 20, 65)
  
  ------ DEBUG
  love.graphics.print(string.format("x, y: %.1f, %.1f", car.body:getX(), car.body:getY()), 20, 80)
  
  -- Facing and velocity vectors
  love.graphics.setColor(0, 0, 255)
  love.graphics.line( car.body:getX()*pxPerMtr, car.body:getY()*pxPerMtr,
    car.body:getX()*pxPerMtr + 40*math.cos(car.body:getAngle()), car.body:getY()*pxPerMtr + 40*math.sin(car.body:getAngle()) )
  
  if math.abs(car:getSpeed()) > 0 then
    
    love.graphics.setColor(255, 0, 0)
    local vx, vy = car.body:getLinearVelocity()
    love.graphics.line( car.body:getX()*pxPerMtr, car.body:getY()*pxPerMtr,
      car.body:getX()*pxPerMtr + 40*vx/car:getSpeed(), car.body:getY()*pxPerMtr + 40*vy/car:getSpeed() )
    
  end
  
  -- Walls
  love.graphics.setColor(255, 255, 255)
  for k, wall in ipairs(walls) do
    wall.image:draw(wall.body:getX() * pxPerMtr, wall.body:getY() * pxPerMtr, wall.body:getAngle())
  end
  
end

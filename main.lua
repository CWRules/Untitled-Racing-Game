
Object = require "lib/classic"
Gamera = require "lib/gamera"
PhysicsHelper = require "PhysicsHelper"

require "PlayerCar"
require "Sprite"

--[[ love.run
  LOVE main function. Need to override to cap framerate.
--]]
function love.run()
	if love.load then love.load(love.arg.parseGameArguments(arg), arg) end

	-- We don't want the first frame's dt to include time taken by love.load.
	if love.timer then love.timer.step() end

	local dt = 0

	-- Main loop time.
	return function()
		-- Process events.
		if love.event then
			love.event.pump()
			for name, a,b,c,d,e,f in love.event.poll() do
				if name == "quit" then
					if not love.quit or not love.quit() then
						return a or 0
					end
				end
				love.handlers[name](a,b,c,d,e,f)
			end
		end

		-- Update dt, as we'll be passing it to update
		if love.timer then dt = love.timer.step() end

		-- Call update and draw
		if love.update then love.update(dt) end -- will pass 0 if love.timer is disabled

    -- Only draw new frame if older than framePeriod
		if love.graphics and love.graphics.isActive() and frameAge >= framePeriod then
      frameAge = 0
			love.graphics.origin()
			love.graphics.clear(love.graphics.getBackgroundColor())

			if love.draw then love.draw() end

			love.graphics.present()
		end

		if love.timer then love.timer.sleep(0.001) end
	end
end


--[[ love.load
  LOVE initialization function. Run once at program start.
--]]
function love.load()
  
  -- DEBUG MODE
  --require("mobdebug").start()
  
  -- Set up window
  love.graphics.setBackgroundColor(1, 1, 1)
  pxPerMtr = 10
  maxX = love.graphics.getWidth() / pxPerMtr
  maxY = love.graphics.getHeight() / pxPerMtr
  
  camera = Gamera.new(-100000, -100000, 200000, 200000)
  
  framePeriod = 1/60
  frameAge = 1.0
  
  -- Fonts
  fontCourier = love.graphics.newFont("cour.ttf", 12)
  love.graphics.setFont(fontCourier)
  
  -- Initialize physics
  -- Set meter size to 1, do graphics scaling manually
  love.physics.setMeter(1)
  world = love.physics.newWorld(0, 0, true)
  gravity = 9.81
  
  -- Declare car
  car = PlayerCar(0, 0)
  
  -- Create walls
  walls = {}
  local wallWidth = 100
  local wallHeight = 2
  local canvas = love.graphics.newCanvas(wallWidth*pxPerMtr, wallHeight*pxPerMtr)
  love.graphics.setCanvas(canvas)
    love.graphics.setColor(0, 0, 0)
    love.graphics.rectangle("fill", 0, 0, wallWidth*pxPerMtr, wallHeight*pxPerMtr)
  love.graphics.setCanvas()
  local wallImage = Sprite(canvas)
  local wallShape = love.physics.newRectangleShape(wallWidth, wallHeight)
  
  for i = 1, 3 do
    walls[i] = {}
    walls[i].image = wallImage
    walls[i].body = love.physics.newBody(world, 0, 0)
    walls[i].shape = wallShape
    walls[i].fixture = love.physics.newFixture(walls[i].body, walls[i].shape)
  end
  
  walls[1].body:setPosition(0, 50)
  
  walls[2].body:setPosition(0, -50)
  
  walls[3].body:setPosition(-50, 0)
  walls[3].body:setAngle(math.pi/2)
  
end


--[[ love.update
  LOVE update function. Run once each program cycle.
  
  dt: Time in seconds since last program cycle.
--]]
function love.update(dt)
  
  world:update(dt)
  car:update(dt)
  
  local ux, uy = PhysicsHelper.getUnitFacingVector(car.body)
  local cameraX = car.body:getX() + (maxY/3)*ux
  local cameraY = car.body:getY() + (maxY/3)*uy
  
  camera:setPosition(cameraX * pxPerMtr, cameraY * pxPerMtr)
  camera:setAngle(car.body:getAngle() + math.pi/2)
  
  frameAge = frameAge + dt
  
end


--[[ love.draw
  LOVE graphics function. Run once each program cycle after update() finishes.
--]]
function love.draw()
  
  camera:draw(function(l,t,w,h)
    
    drawFloorPattern(l,t,w,h)
  
    -- Car
    car:draw()
  
    -- Facing and velocity vectors
    love.graphics.setColor(0, 0, 1)
    love.graphics.line( car.body:getX()*pxPerMtr, car.body:getY()*pxPerMtr,
      car.body:getX()*pxPerMtr + 40*math.cos(car.body:getAngle()), car.body:getY()*pxPerMtr + 40*math.sin(car.body:getAngle()) )
  
    if math.abs(PhysicsHelper.getSpeed(car.body)) > 0 then
    
      love.graphics.setColor(1, 0, 0)
      local vx, vy = car.body:getLinearVelocity()
      love.graphics.line( car.body:getX()*pxPerMtr, car.body:getY()*pxPerMtr,
        car.body:getX()*pxPerMtr + 40*vx/PhysicsHelper.getSpeed(car.body), car.body:getY()*pxPerMtr + 40*vy/PhysicsHelper.getSpeed(car.body) )
    
    end
  
    -- Walls
    love.graphics.setColor(1, 1, 1)
    for _, wall in ipairs(walls) do
      wall.image:draw(wall.body:getX()*pxPerMtr, wall.body:getY()*pxPerMtr, wall.body:getAngle())
    end
  
  end)
  
  -- Car info
  love.graphics.setColor(0, 0, 0)
  love.graphics.print(string.format("FPS: %d", 1/love.timer.getAverageDelta()), 20, 20)
  love.graphics.print(string.format("thr, brk, str: %.2f, %.2f, % .2f", car.throttle, car.brake, car.steering), 20, 35)
  local ForwardSpeed = PhysicsHelper.getRelativeSpeed(car.body)
  love.graphics.print(string.format("Speed: %.1f mph", 2.237 * ForwardSpeed), 20, 50)
  
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
    for _ = 1, gearChangeProgress do gearString = gearString .. "-" end
    gearString = gearString .. "|"
    for _ = 1, 9 - gearChangeProgress do gearString = gearString .. "-" end
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
  
  love.graphics.print(string.format("x, y, ang: %.1f, %.1f, %.3f", car.body:getX(), car.body:getY(), car.body:getAngle()), 20, 80)
  
  love.graphics.print(string.format("Steering ang: %.3f", car.frontTire.body:getAngle() - car.body:getAngle()), 20, 95)
  
end


--[[ drawFloorPattern
  Function to draw checkered background pattern adapted from the gamera demo.
--]]
function drawFloorPattern(cl, ct, cw, ch)
  
  local _, _, ww, wh = camera:getWorld()
  local rows = 1000
  local columns = 1000
  local w = ww / columns
  local h = wh / rows

  local minX = math.max(math.floor(cl/w), -columns)
  local maxX = math.min(math.floor((cl+cw)/w), columns-1)
  local minY = math.max(math.floor(ct/h), -rows)
  local maxY = math.min(math.floor((ct+ch)/h), rows-1)

  for y=minY, maxY do
    for x=minX, maxX do
      if (x + y) % 2 == 0 then
        love.graphics.setColor(0.6, 0.6, 0.6)
      else
        love.graphics.setColor(0.8, 0.8, 0.8)
      end
      love.graphics.rectangle("fill", x*w, y*h, w, h)
    end
  end
  
end

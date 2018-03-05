
--[[ love.conf()
  Overrides default LOVE configuration function.
--]]
function love.conf(t)
  t.window.title = "Car Physics Test"
  t.window.width = 1000
  t.window.height = 600
  t.modules.joystick = false
  t.window.vsync = false
end

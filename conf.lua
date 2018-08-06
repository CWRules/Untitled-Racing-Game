
--[[ love.conf()
  Overrides default LOVE configuration function.
--]]
function love.conf(t)
  t.window.title = "Car Physics Test"
  t.window.width = 1800
  t.window.height = 900
  t.modules.joystick = false
  t.window.vsync = false
end

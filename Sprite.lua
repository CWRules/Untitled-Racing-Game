
--[[ Sprite
  Defines the Sprite class; a wrapper to simplify drawing images for physics objects.
--]]
Sprite = Object:extend()

--[[ Sprite:new(target)
  Sprite constructor.
  
  target: Overloaded parameter.
    A) Path to the image to create the Sprite with.
    B) Drawable object to create the Sprite with.
--]]
function Sprite:new(target)
  
  if type(target) == "string" then
    self.image = love.graphics.newImage(target)
  else
    self.image = target
  end
  
  self.width = self.image:getWidth()
  self.height = self.image:getHeight()
  self.originX = self.width / 2
  self.originY = self.height / 2
  
end

--[[ Sprite:draw()
  Draws the Sprite.
  
  x: X coordinate to draw the Sprite at.
  y: Y coordinate to draw the Sprite at.
  angle: Angle coordinate to draw the Sprite at.
--]]
function Sprite:draw(x, y, angle)
  
  love.graphics.draw(self.image, x, y, angle, 1, 1, self.originX, self.originY)
  
end

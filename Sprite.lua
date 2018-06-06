
--[[ Sprite
  Defines the Sprite class; a wrapper to simplify drawing images for physics objects.
--]]
Sprite = Object:extend()

--[[ Sprite:new(imagePath)
  Sprite constructor.
  
  imagePath: Path to the image to create the Sprite with.
--]]
function Sprite:new(imagePath)
  
  self.image = love.graphics.newImage(imagePath)
  self.width = self.image:getWidth()
  self.height = self.image:getHeight()
  self.originX = self.image:getWidth() / 2
  self.originY = self.image:getHeight() / 2
  
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

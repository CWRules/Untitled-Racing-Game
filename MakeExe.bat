set ZIP="C:\Program Files\7-Zip\7z.exe"
set LOVE="C:\Program Files\LOVE\love.exe"

for %%I in (.) do set NAME=%%~nxI

%ZIP% a -tzip %NAME%.love images\* lib\* conf.lua cour.ttf main.lua PlayerCar.lua Sprite.lua
copy /b %LOVE%+%NAME%.love %NAME%.exe
%ZIP% a -tzip %NAME%.zip %NAME%.exe .\DLLs\*

del %NAME%.love %NAME%.exe
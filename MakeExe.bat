set ZIP="C:\Program Files\7-Zip\7z.exe"
set LOVEDIR="C:\Program Files\LOVE"

for %%I in (.) do set NAME=%%~nxI

%ZIP% a -tzip %NAME%.love images\* lib\* conf.lua cour.ttf main.lua PlayerCar.lua Sprite.lua
copy /b %LOVEDIR%\love.exe+%NAME%.love %NAME%.exe
%ZIP% a -tzip %NAME%.zip %NAME%.exe %LOVEDIR%\*.dll

del %NAME%.love %NAME%.exe
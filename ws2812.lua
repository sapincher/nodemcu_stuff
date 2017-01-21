ws2812.init()

brightness = 16
ledIndex = 1
fillColor = red

red = string.char(0, brightness, 0)
yellow = string.char(brightness*0.75, brightness, 0)
green = string.char(brightness, 0, 0)

buffer = ws2812.newBuffer(8,4)
buffer:fill(0,0,0,255)

--[[for i=1,6 do
	if (i == 1 or i == 4) then
		fillColor = red
	elseif (i == 2 or i == 5) then
		fillColor = yellow
	else
		fillColor = green
	end
	for j=ledIndex,ledIndex + 7 do
		buffer:set(j, fillColor)
	end
	ledIndex = ledIndex + 8
end]]

ws2812.write(buffer)

print("\nBuffer power: "..(buffer:power() / 255 * 20) .. "mA")
timer = tmr.create()

--[[timer:register(250, tmr.ALARM_AUTO, function()
    buffer:shift(1, ws2812.SHIFT_CIRCULAR)
    ws2812.write(buffer)
end)]]

timer:register(100, tmr.ALARM_AUTO, function()
	brightVal = adc.read(0) / 4
	if brightVal == 256 then
		brightVal = 255
	end
	buffer:fill(brightVal,brightVal,brightVal,brightVal)
	ws2812.write(buffer)
end)
timer:start()

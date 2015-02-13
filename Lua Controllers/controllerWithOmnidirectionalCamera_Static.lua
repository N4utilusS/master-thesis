-- Put your global variables here
LEFT_COLOUR = {0,255,255}
RIGHT_COLOUR = {255,0,255}

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()


  robot.wheels.set_velocity(0, 0)
  if string.sub(robot.id, 6, 6) == "1" then
    robot.overo_leds.set_all_colors(LEFT_COLOUR[1],LEFT_COLOUR[2],LEFT_COLOUR[3])
  elseif string.sub(robot.id, 6, 6) == "2" then
    robot.overo_leds.set_all_colors(RIGHT_COLOUR[1], RIGHT_COLOUR[2], RIGHT_COLOUR[3])
  end
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()

end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
  robot.wheels.set_velocity(1, 1.1)
  if string.sub(robot.id, 6, 6) == "1" then
    robot.overo_leds.set_all_colors(LEFT_COLOUR[1],LEFT_COLOUR[2],LEFT_COLOUR[3])
  elseif string.sub(robot.id, 6, 6) == "2" then
    robot.overo_leds.set_all_colors(RIGHT_COLOUR[1], RIGHT_COLOUR[2], RIGHT_COLOUR[3])
  end
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end

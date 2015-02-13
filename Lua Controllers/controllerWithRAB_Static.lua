-- Put your global variables here



--[[ This function is executed every time you press the 'execute'
     button ]]
function init()


  robot.wheels.set_velocity(0, 0)
  if string.sub(robot.id, 7, 7) == "1" then
    robot.range_and_bearing.set_data({1,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "2" then
    robot.range_and_bearing.set_data({2,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "3" then
    robot.range_and_bearing.set_data({3,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "4" then
    robot.range_and_bearing.set_data({4,0,0,0})
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
  if string.sub(robot.id, 7, 7) == "1" then
    robot.range_and_bearing.set_data({1,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "2" then
    robot.range_and_bearing.set_data({2,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "3" then
    robot.range_and_bearing.set_data({3,0,0,0})
  elseif string.sub(robot.id, 7, 7) == "4" then
    robot.range_and_bearing.set_data({4,0,0,0})
  end
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end

-- Put your global variables here
initialSpeed = 5

-- Anti blocking system:
direction = 0
count = 0
MAX_COUNT = 20

-- Human Potential
HUMAN_GAIN = 1000
HUMAN_DISTANCE = 100
HUMAN_SIGNAL_MIN = 1
HUMAN_SIGNAL_MAX = 4

-- Agent Repulsion Potential
AGENT_GAIN = 200
AGENT_DISTANCE = 100
AGENT_SIGNAL = 0

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()

end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
  if string.sub(robot.id, 1, 1) == "s" then
    staticRobot()
  else
    movingRobot()
  end
end

function staticRobot()
  robot.wheels.set_velocity(0, 0)
end

function movingRobot()
  if count < MAX_COUNT then
    normalMode()
  else
    blockedMode()
  end
end

function normalMode()
  obstacleVector = {x = 0, y = 0}
  resultVector = {x = 0, y = 0}
  maxValue = 0
  unMaxIndexPlusOne = #robot.proximity + 1

for i = 1, 2 do
    -- Add the vector corresponding to this left sensor
    obstacleVector.x = obstacleVector.x + robot.proximity[i].value*math.cos(robot.proximity[i].angle)
    obstacleVector.y = obstacleVector.y + robot.proximity[i].value*math.sin(robot.proximity[i].angle)
    
    -- Add the vector corresponding to this right sensor
    obstacleVector.x = obstacleVector.x + robot.proximity[unMaxIndexPlusOne-i].value*math.cos(robot.proximity[unMaxIndexPlusOne-i].angle)
    obstacleVector.y = obstacleVector.y + robot.proximity[unMaxIndexPlusOne-i].value*math.sin(robot.proximity[unMaxIndexPlusOne-i].angle)

    -- Update the maximum value of the proximity values
    if robot.proximity[i].value > maxValue then
      maxValue = robot.proximity[i].value
    end
    if robot.proximity[unMaxIndexPlusOne-i].value > maxValue then
      maxValue = robot.proximity[unMaxIndexPlusOne-i].value
    end
  end

  -- Get the direction vector, straight forward if no obstacle
  if maxValue > 0.15 then
    resultVector.x = -obstacleVector.x
    resultVector.y = -obstacleVector.y

  else -- INSERT HERE THE DIRECTION COMPUTATION
    resultVector = computeDirection()
    count = 0
  end

  -- Get the angle of the direction vector
  angle = math.atan2(resultVector.y, resultVector.x)
  speed = math.sqrt(math.pow(resultVector.x, 2) + math.pow(resultVector.y, 2))
  if speed > 10 then -- Limit speed to 10 cm/s
    speed = 10
  end

  if (angle > 0) and (angle < math.pi) then -- Turn left
    newDirection = 1
    robot.wheels.set_velocity(speed * math.cos(angle), speed)
  else  -- Turn right
    newDirection = -1
    robot.wheels.set_velocity(speed, speed * math.cos(angle))
  end

  -- Save the current direction (for the antiblock system)
  if direction ~= newDirection then
    count = count + 1
    direction = newDirection
  end
end

function computeDirection()
  resultVector = {x = 0, y = 0}

  -- Add the human potential:
  potentialFragment = humanPotential()
  resultVector.x = resultVector.x + potentialFragment.x
  resultVector.y = resultVector.y + potentialFragment.y

  -- Add the gravity potential:
  potentialFragment = gravityPotential()
  resultVector.x = resultVector.x + potentialFragment.x
  resultVector.y = resultVector.y + potentialFragment.y
  
  -- Add the agent repulsion potential:
  potentialFragment = agentRepulsionPotential()
  resultVector.x = resultVector.x + potentialFragment.x
  resultVector.y = resultVector.y + potentialFragment.y
  
  -- Add default potential:
  potentialFragment = defaultPotential()
  resultVector.x = resultVector.x + potentialFragment.x
  resultVector.y = resultVector.y + potentialFragment.y

  return resultVector
end

function blockedMode()
  unMaxIndexPlusOne = #robot.proximity + 1

  if robot.proximity[i].value > robot.proximity[unMaxIndexPlusOne-i].value then
    maxValue = robot.proximity[i].value
  else
    maxValue = robot.proximity[unMaxIndexPlusOne-i].value
  end

  if maxValue > 0.2 then  -- Somethin ahead, turn right.
    robot.wheels.set_velocity(initialSpeed, -initialSpeed)
  else  -- Clear ahead, go straight forward and quit blocked mode.
    count = 0
    robot.wheels.set_velocity(initialSpeed, initialSpeed)
  end
end

function defaultPotential()
  vector = {x = 0, y = 0}
  humanFound = false

  for i = 1, #robot.range_and_bearing do
    if (robot.range_and_bearing[i].data[1] >= HUMAN_SIGNAL_MIN) and (robot.range_and_bearing[i].data[1] <= HUMAN_SIGNAL_MAX) then
      humanFound = true
      break
    end
  end
  
  if humanFound == false then
    vector.x = 5
  end

  return vector
end

function agentRepulsionPotential()
  vector = {x = 0, y = 0}
  
  for i = 1, #robot.range_and_bearing do
    if (robot.range_and_bearing[i].data[1] == AGENT_SIGNAL) and (robot.range_and_bearing[i].range < AGENT_DISTANCE) then
      lj = lennardJones(robot.range_and_bearing[i].range, AGENT_GAIN, AGENT_DISTANCE)
      vector.x = vector.x + lj * math.cos(robot.range_and_bearing[i].horizontal_bearing)
      vector.y = vector.y + lj * math.sin(robot.range_and_bearing[i].horizontal_bearing)
    end
  end

  return vector
end

function gravityPotential()
  vector = {x = 0, y = 0}

  d = 0
  amountOfMessages = #robot.range_and_bearing
  minimum = math.huge
  minimumIndex = -1

  for i=1, amountOfMessages do
    if (robot.range_and_bearing[i].data[1] >= 1) and (robot.range_and_bearing[i].data[1] <= 4) and (robot.range_and_bearing[i].range < minimum) then
      minimum = robot.range_and_bearing[i].range
      minimumIndex = i
    end
  end

  if minimumIndex ~= -1 then
    if robot.range_and_bearing[minimumIndex].data[1] <= 2 then
      rotationModification = math.pi/2
    else
      rotationModification = -math.pi/2
    end

    vector.x = math.cos(robot.range_and_bearing[minimumIndex].horizontal_bearing + rotationModification)
    vector.y = math.sin(robot.range_and_bearing[minimumIndex].horizontal_bearing + rotationModification)
  end
  

  return vector
end

function humanPotential()
  vector = {x = 0, y = 0}

  d = 0
  amountOfMessages = #robot.range_and_bearing

  i = 0
  repeat
    i = i + 1
  until (i >= amountOfMessages) or (robot.range_and_bearing[i].data[1] ~= AGENT_SIGNAL)

  if (amountOfMessages ~= 0) and (robot.range_and_bearing[i].data[1] ~= AGENT_SIGNAL) then
    lj = lennardJones(robot.range_and_bearing[i].range, HUMAN_GAIN, HUMAN_DISTANCE)
    vector.x = lj * math.cos(robot.range_and_bearing[i].horizontal_bearing)
    vector.y = lj * math.sin(robot.range_and_bearing[i].horizontal_bearing)
  end

  return vector
end

function lennardJones(d, gain, distance)
  return -4*gain/d * (math.pow(distance/d, 4) - math.pow(distance/d, 2))
end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end

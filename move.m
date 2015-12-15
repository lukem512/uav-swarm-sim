function robot = move(robot, v, mu)
    robot.x = robot.x + v*sind(robot.theta);
    robot.y = robot.y + v*cosd(robot.theta);
    robot.theta = robot.theta + v*mu;
end
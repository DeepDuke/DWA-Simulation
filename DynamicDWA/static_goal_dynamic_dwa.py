# Simualation for a modified dwa algorithm in dynamic environment

import pygame, os, math, time, random, copy
from pygame.locals import *
import time


class Environment:
    """ Simulation Environment contains moving obstacles 
        and agent which performs collision aovidance algorithm.
    """
    # Class public variables
    sim_over = False  # indicates whether a simulation is over or not
    sim_times = 0  # num of simulation times
    collision_times = 0  # num of collision times during all simulations
    def __init__(self, agent):   
        # Record simlation times
        Environment.sim_times += 1     
        # set the width and height of the screen (pixels)
        # The region we will fill with obstacles
        self.PLAYFIELDCORNERS = (-4.0, -3.0, 4.0, 3.0)
        self.WIDTH = 1500
        self.HEIGHT = 1000
        self.size = [self.WIDTH, self.HEIGHT]
        self.black = (20,20,40)
        self.lightblue = (0,120,255)
        self.darkblue = (0,40,160)
        self.red = (255,100,0)
        self.white = (255,255,255)
        self.blue = (0,0,255)
        self.grey = (70,70,70)

        # Constants for graphics display
        # Transformation from metric world frame to graphics frame
        # k pixels per metre
        # Horizontal screen coordinate:     u = u0 + k * x
        # Vertical screen coordinate:       v = v0 - k * y
        self.k = 160 # pixels per metre for graphics
        # Screen center will correspond to (x, y) = (0, 0)
        self.u0 = self.WIDTH / 2
        self.v0 = self.HEIGHT / 2
        
        # Initiate obstacles
        self.OBSTACLE_RADIUS = 0.10
        self.OBSTACLE_MAX_VEL = 0.15
        self.obstacles = []  # obstalces list
        self.init_obstacle_num = 10
        for i in range(self.init_obstacle_num):
            pos_x = random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2])
            pos_y = random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3])
            vx = random.gauss(0.0, self.OBSTACLE_MAX_VEL)
            vy = random.gauss(0.0, self.OBSTACLE_MAX_VEL)
            obstacle = [pos_x, pos_y, vx, vy]
            self.obstacles.append(obstacle)
        
        # Simulatation time interval
        self.dt = 0.1
        # Initiate robot's goal
        self.goal = (self.PLAYFIELDCORNERS[2]-0.5, self.PLAYFIELDCORNERS[3]-0.5)
        self.goal_flag = False  # True if robot reached athe goal
        # Initiate pose of robot
        self.init_x = self.PLAYFIELDCORNERS[0] + 0.5
        self.init_y = self.PLAYFIELDCORNERS[1] + 0.5
        self.init_theta = 0.0
        init_pose = (self.init_x, self.init_y, self.init_theta)
        config = (self.dt, self.k, self.u0, self.v0, self.OBSTACLE_RADIUS)
        self.agent = agent(init_pose, config)
        # Record robot's history positions and paths
        self.history_positions = []
        
        # Configure parameters for visulization in pygame
        pygame.init()
        # Initialise Pygame display screen
        self.screen = pygame.display.set_mode(self.size)
        # This makes the normal mouse pointer invisible/visible (0/1) in graphics window
        pygame.mouse.set_visible(1)
        
        # Record collision times
        self.collision_times = 0


    def move_obstacles(self):
        """ Update locations and velocties of moving obstacles"""
        for idx in range(len(self.obstacles)):
            # Update x coordinate
            self.obstacles[idx][0] += self.obstacles[idx][2] * self.dt
            if self.obstacles[idx][0] < self.PLAYFIELDCORNERS[0]:
                self.obstacles[idx][2] = -self.obstacles[idx][2]
            if self.obstacles[idx][0] > self.PLAYFIELDCORNERS[2]:
                self.obstacles[idx][2] = -self.obstacles[idx][2]
            # Update y coordinate
            self.obstacles[idx][1] += self.obstacles[idx][3] * self.dt  
            if self.obstacles[idx][1] < self.PLAYFIELDCORNERS[1]:
                self.obstacles[idx][3] = -self.obstacles[idx][3]
            if self.obstacles[idx][1] > self.PLAYFIELDCORNERS[3]:
                self.obstacles[idx][3] = -self.obstacles[idx][3]

    
    def draw_obstacles(self):
        """ Draw circular obstalces on screen """
        for (idx, obstacle) in enumerate(self.obstacles):
            color = self.lightblue
            pygame.draw.circle(self.screen, color, (int(self.u0 + self.k * obstacle[0]), int(self.v0 - self.k * obstacle[1])), int(self.k * self.OBSTACLE_RADIUS), 0)


    def draw_goal(self):
        """ Draw the goal of robot on screen """
        color  =  self.red
        goal = self.goal
        pygame.draw.circle(self.screen, color, (int(self.u0 + self.k * goal[0]), int(self.v0 - self.k * goal[1])), int(self.k * self.OBSTACLE_RADIUS), 0)
        
    
    def draw_robot(self):
        """ Draw cirular robot on screen """
        color = self.white
        radius = self.agent.ROBOT_RADIUS
        position = (self.agent.x, self.agent.y) 
        pygame.draw.circle(self.screen, color, (int(self.u0 + self.k * position[0]), int(self.v0 - self.k * position[1])), int(self.k * radius), 3)

    
    def draw_history_trajectory(self):
        """ Draw trajectory of moving robot """
        color = self.grey
        # Draw locations
        for pos in self.history_positions:
            pygame.draw.circle(self.screen, color, (int(self.u0 + self.k * pos[0]), int(self.v0 - self.k * pos[1])), 3, 0)
    
    def draw_predicted_tracjetory(self, predicted_path_to_draw):
        # Draw paths(straight lines or arcs)
        for path in predicted_path_to_draw:
            if path[0] == 0:  # Straight line
                straightpath = path[1]
                linestart = (self.u0 + self.k * self.agent.x, self.v0 - self.k * self.agent.y)
                lineend = (self.u0 + self.k * (self.agent.x + straightpath * math.cos(self.agent.theta)), self.v0 - self.k * (self.agent.y + straightpath * math.sin(self.agent.theta)))
                pygame.draw.line(self.screen, (0, 200, 0), linestart, lineend, 1)
            if path[0] == 1:  # Rotation, nothing to draw
                pass
            if path[0] == 2:  # General case: circular arc
                # path[2] and path[3] are start and stop angles for arc but they need to be in the right order to pass
                if (path[3] > path[2]):
                    startangle = path[2]
                    stopangle = path[3]
                else:
                    startangle = path[3]
                    stopangle = path[2]
                # Pygame arc doesn't draw properly unless angles are positive
                if (startangle < 0):
                    startangle += 2*math.pi
                    stopangle += 2*math.pi
                if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
                    #print (path[1], startangle, stopangle)
                    pygame.draw.arc(self.screen, (0, 200, 0), path[1], startangle, stopangle, 1)


    def draw_frame(self, predicted_path_to_draw):
        """ Draw each frame of simulation on screen"""
        # Set pygame
        Eventlist = pygame.event.get()
        # Start drawing
        self.screen.fill(self.black)  # Set screen background color
        self.draw_goal()
        self.draw_obstacles()
        self.draw_robot()
        self.draw_history_trajectory()
        self.draw_predicted_tracjetory(predicted_path_to_draw)
        # Update display
        pygame.display.flip()

    
    def check_collsion(self):
        """ Check if collision happened between robot and moving obstacles """
        for obstacle in self.obstacles:
            ox, oy = obstacle[0], obstacle[1]
            dist = math.sqrt((ox-self.agent.x)**2 + (oy-self.agent.y)**2)
            if dist < self.agent.ROBOT_RADIUS + self.OBSTACLE_RADIUS:  # Collision happened
                self.collision_times += 1
                Environment.collision_times += 1


    def run(self):
        """ Do simulation """
        # Start simulation
        predicted_path_to_draw = []
        # Save robot's locations for display of trail
        self.history_positions.append((self.agent.x, self.agent.y))
        # Planning velocities and path
        predicted_path_to_draw = self.agent.planning(self.goal, self.obstacles)
        # Visualization
        self.draw_frame(predicted_path_to_draw)
        # Check collison
        self.check_collsion()
        if self.goal_flag == False and self.collision_times > 0:
            print('Collsion happened during simulation')
            Environment.sim_over = True
            Environment.sim_times += 1
        # Check if arrive at the goal
        dist_to_goal = math.sqrt((self.agent.x-self.goal[0])**2 + (self.agent.y-self.goal[1])**2)
        if self.goal_flag == False and self.collision_times == 0 and round(dist_to_goal, 3) < 0.5*self.agent.ROBOT_RADIUS:
            self.goal_flag = True
            print('Arrive at the goal, Simulation finished!')
            Environment.sim_over = True
            Environment.sim_times += 1
        if self.goal_flag == False and self.collision_times == 0:
            # Move obstacles
            self.move_obstacles()
            # print('Moving Obstacles')
            # Move robot
            self.agent.move_robot()
            # print('Moving Robots')

    

class NewDWA:
    """ Collision avoidance algorithm """
    def __init__(self, init_pose=(0, 0, 0), config=(0.10, 0, 0, 0)):
        # parameters of robot
        self.ROBOT_RADIUS = 0.10
        # Linear velocity limits
        self.MAX_VEL_LINEAR = 0.5     # ms^(-1) max speed of each wheel
        self.MAX_ACC_LINEAR = 0.5     # ms^(-2) max rate we can change speed of each wheel
        # Angular velocity limits
        self.MAX_VEL_ANGULAR = 0.5
        self.MAX_ACC_ANGULAR = 0.5
        # Current linear velocity and angular velocity
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        # Current positions
        self.x, self.y, self.theta = init_pose
        # Parameters for prediction trajectory
        self.dt, self.k, self.u0, self.v0, self.OBSTACLE_RADIUS = config
        self.STEPS_AHEAD_TO_PLAN = 10
        self.TAU = self.dt * self.STEPS_AHEAD_TO_PLAN
        # Safe distance between robot and closest obstacle after minus robot's radius and obstacle's radius
        self.SAFE_DIST = self.ROBOT_RADIUS 
        # Weights for predicted trajectory evaluation
        self.FORWARDWEIGHT = 12
        self.OBSTACLEWEIGHT = 6666


    def predict_position(self, vLpossible, vRpossible, delta_time):
        """ Predict robot's position in delta_time  
            
            @param vLpossible:    possible linear velocity

            @param vRpossible:    possible angular velocity

            @return:    (new_x, new_y, new_theta, path)
        """

        # Go straight line
        if round(vRpossible, 3) == 0:
            new_x = self.x + vLpossible*delta_time*math.cos(self.theta)
            new_y = self.y + vLpossible*delta_time*math.sin(self.theta)
            new_theta = self.theta
            path = (0, vLpossible*delta_time)  # 0 indciates pure translation
        # Pure rotation motion
        elif round(vLpossible, 3) == 0:
            new_x = self.x
            new_y = self.y
            new_theta = self.theta + vRpossible*delta_time
            path = (1, 0)  # 1 indicates pure rotation
        else:
            # Rotation and arc angle of general circular motion
            R = vLpossible / vRpossible
            delta_theta = vRpossible * delta_time
            new_x = self.x + R * (math.sin(delta_theta + self.theta) - math.sin(self.theta))
            new_y = self.y - R * (math.cos(delta_theta + self.theta) - math.cos(self.theta))
            new_theta = self.theta + delta_theta

            # Calculate parameters for drawing arc
            # We need center of circle
            (cx, cy) = (self.x - R * math.sin(self.theta), self.y + R * math.cos(self.theta))
            # Turn this into  Rect
            Rabs = abs(R)
            ((tlx, tly), (Rx, Ry)) = ((int(self.u0 + self.k * (cx - Rabs)), int(self.v0 - self.k * (cy + Rabs))), (int(self.k * (2 * Rabs)), int(self.k * (2 * Rabs))))
            if (R > 0):
                start_angle = self.theta - math.pi/2.0
            else:
                start_angle = self.theta + math.pi/2.0
            stop_angle = start_angle + delta_theta
            path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion
        
        return (new_x, new_y, new_theta, path)


    def calculateClosestObstacleDistance(self, predict_x, predict_y, obstacles):
        """ Calculate  distance to closest obstacle 
            
            @param predict_x: predicted x coordiante of robot

            @param predict_y: predicted y coordiante of robot

            @param obstacles: contains obstacles' information,that is [pos_x, pos_y, vx, vy] 

            @return: distance between robot and closest obstacle
        """
        closestdist = 100000.0  
        for (idx, obstacle) in enumerate(obstacles):
            dx = obstacle[0] - predict_x
            dy = obstacle[1] - predict_y
            d = math.sqrt(dx**2+dy**2)
            # Distance between closest touching point of circular robot and circular obstacle
            dist = d - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS
            if dist < closestdist:
                closestdist = dist

        return closestdist 


    def planning(self, goal, obstacles):
        """ Planning trajectory and select linear and angular velocities for robot
            
            @param goal:  goal postion of robot

            @param obstacles:  [pos_x, pos_y, vx, vy] of each obstacles
        
            @return:  predicted_path_to_draw
        """
        bestBenefit = -100000
        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (self.linear_vel - self.MAX_ACC_LINEAR * self.dt, self.linear_vel, self.linear_vel + self.MAX_ACC_LINEAR * self.dt)
        vRpossiblearray = (self.angular_vel - self.MAX_ACC_ANGULAR * self.dt, self.angular_vel, self.angular_vel + self.MAX_ACC_ANGULAR * self.dt)
        vLchosen = 0
        vRchosen = 0
        predicted_path_to_draw = []
        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # Check if in veolicties's range
                if vLpossible <= self.MAX_VEL_LINEAR and vRpossible <= self.MAX_VEL_ANGULAR and vLpossible >= 0 and vRpossible >= -self.MAX_VEL_ANGULAR:
                    # Predict robot's new position in TAU seconds
                    predict_x, predict_y, predict_theta, path = self.predict_position(vLpossible, vRpossible, self.TAU)
                    predicted_path_to_draw.append(path)
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)
                    newTargetDistance = math.sqrt((predict_x - goal[0])**2 + (predict_y - goal[1])**2)
                    distanceForward = previousTargetDistance - newTargetDistance
                    # Cost term about distance to goal for evaluation
                    dist_goal_cost  =  self.FORWARDWEIGHT * distanceForward
                    # Cost term about distance to closest obstacle for evaluation
                    distanceToObstacle = self.calculateClosestObstacleDistance(predict_x, predict_y, obstacles)
                    if distanceToObstacle < self.SAFE_DIST:
                        dist_obstacle_cost = self.OBSTACLEWEIGHT * (self.SAFE_DIST - distanceToObstacle)
                    else:
                        dist_obstacle_cost = 0
                            
                    # Total cost
                    benefit = dist_goal_cost - dist_obstacle_cost
                    if benefit > bestBenefit:
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit
        # Update velocities
        self.linear_vel = vLchosen
        self.angular_vel = vRchosen
        # print('[vLchosen:\t{:.3f}]\t[vRchosen:\t{:.3f}]'.format(vLchosen, vRchosen))


        # Return path to draw
        return predicted_path_to_draw


    def move_robot(self):
        """ Move robot based on chosen velocities in dt time"""
        self.x, self.y, self.theta, tmp_path = self.predict_position(self.linear_vel, self.angular_vel, self.dt) 

    

       
if __name__ == '__main__':
    env = Environment(NewDWA)
    while 1:
        if Environment.sim_over == True:
            # Start a new simualtion 
            env = Environment(NewDWA)
            Environment.sim_over = False
            print('Finished Simulation Times: #{}\tCollision Times: #{}'.format(Environment.sim_times, Environment.collision_times))
        env.run()
        time.sleep(0.01)

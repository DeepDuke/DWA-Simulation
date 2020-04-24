# Simulation for origin version of DWA (Dynamic Window Approach) algorithm
# Simualation for a modified dwa algorithm in dynamic environment
import pygame, os, math, time, random, copy
from pygame.locals import *
import time
import pandas as pd 
from matplotlib import pyplot as plt 


class Environment:
    """ Simulation Environment contains moving obstacles 
        and agent which performs collision aovidance algorithm.
    """
    # Class public variables
    sim_over = False  # indicates whether a simulation is over or not
    sim_times = 0  # num of simulation times
    collision_times = 0  # num of collision times during all simulations
    avg_tg = 0
    tg_vec = []
    vl_vec = []
    vr_vec = []
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
        self.time_to_goal = 0.0
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
        self.history_vl_vec = []
        self.history_vr_vec = []
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
        while True:
            # t_start = time.time()
            # Start simulation
            self.time_to_goal += self.dt
            predicted_path_to_draw = []
            # Save robot's locations for display of trail
            self.history_positions.append((self.agent.x, self.agent.y))
            # Planning velocities and path
            predicted_path_to_draw, vLchosen, vRchosen = self.agent.planning(self.goal, self.obstacles)
            self.history_vl_vec.append(vLchosen)
            self.history_vr_vec.append(vRchosen)
            # Visualization
            self.draw_frame(predicted_path_to_draw)
            # Check collison
            self.check_collsion()
            if self.goal_flag == False and self.collision_times > 0:
                Environment.sim_over = True
                print('#{}\tCollision happened during simulation !'.format(Environment.sim_times))
                break
            # Check if arrive at the goal
            dist_to_goal = math.sqrt((self.agent.x-self.goal[0])**2 + (self.agent.y-self.goal[1])**2)
            if self.goal_flag == False and self.collision_times == 0 and round(dist_to_goal, 3) < 0.5*self.agent.ROBOT_RADIUS:
                self.goal_flag = True
                Environment.sim_over = True
                Environment.tg_vec.append(self.time_to_goal)
                Environment.vl_vec.extend(self.history_vl_vec)
                Environment.vr_vec.extend(self.history_vr_vec)
                Environment.avg_tg = ((Environment.sim_times - 1)*Environment.avg_tg+self.time_to_goal)/Environment.sim_times
                print( '#{}\tArrive at the goal, Simulation finished!  tg:  {:.4f}'.format(Environment.sim_times, self.time_to_goal))
                break
            if self.goal_flag == False and self.collision_times == 0:
                # Move obstacles
                self.move_obstacles()
                # print('Moving Obstacles')
                # Move robot
                self.agent.move_robot()
                # print('Moving Robots')
            # t_end = time.time()
            # print('FPS:{:.2f}, Decision Time:{:.2f}'.format(1/(t_end-t_start), t_end-t_start))
    

class DWA:
    """ Collision avoidance algorithm """
    def __init__(self, init_pose=(0, 0, 0), config=(0.10, 0, 0, 0)):
        # parameters of robot
        self.ROBOT_RADIUS = 0.1
        # Linear velocity limits
        self.MAX_VEL_LINEAR = 0.8     # ms^(-1) max speed of each wheel
        self.MAX_ACC_LINEAR = 0.3     # ms^(-2) max rate we can change speed of each wheel
        # Angular velocity limits
        self.MAX_VEL_ANGULAR = 0.8
        self.MAX_ACC_ANGULAR = 1.0
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
        self.SAFE_DIST = self.ROBOT_RADIUS   #  
        # Weights for predicted trajectory evaluation
        self.heading_gain = 50
        self.dist_obstacle_gain = 35
        self.vel_gain = 2


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
        sample_num = 10
        vLUpBound = self.linear_vel + self.MAX_ACC_LINEAR * self.dt
        vLDownBound = self.linear_vel - self.MAX_ACC_LINEAR * self.dt
        vLpossiblearray = tuple(vLDownBound + i/(sample_num-1)*(vLUpBound-vLDownBound) for i in range(sample_num))
        # print('vLpossiblearray:', tuple(vLpossiblearray))
        vRUpBound = self.angular_vel + self.MAX_ACC_ANGULAR * self.dt
        vRDownBound = self.angular_vel - self.MAX_ACC_ANGULAR * self.dt
        vRpossiblearray = tuple(vRDownBound + i/(sample_num-1)*(vRUpBound-vRDownBound) for i in range(sample_num))
        # print('vRpossiblearray:', tuple(vRpossiblearray))
        # vLpossiblearray = (self.linear_vel - self.MAX_ACC_LINEAR * self.dt, self.linear_vel, self.linear_vel + self.MAX_ACC_LINEAR * self.dt)
        # vRpossiblearray = (self.angular_vel - self.MAX_ACC_ANGULAR * self.dt, self.angular_vel, self.angular_vel + self.MAX_ACC_ANGULAR * self.dt)
        vLchosen = 0
        vRchosen = 0
        predicted_path_to_draw = []
        # Record for normalize
        costVec = []
        sum_term1 = 0
        sum_term2 = 0
        sum_term3 = 0
        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # Check if in veolicties's range
                if vLpossible <= self.MAX_VEL_LINEAR and vRpossible <= self.MAX_VEL_ANGULAR and vLpossible >= 0 and vRpossible >= -self.MAX_VEL_ANGULAR:
                    # Predict robot's new position in TAU seconds
                    predict_x, predict_y, predict_theta, path = self.predict_position(vLpossible, vRpossible, self.TAU)
                    predicted_path_to_draw.append(path)
                    # Calculate how much close we've moved to target location
                    # previousTargetDistance = math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)
                    # newTargetDistance = math.sqrt((predict_x - goal[0])**2 + (predict_y - goal[1])**2)
                    # distanceForward = previousTargetDistance - newTargetDistance
                    # Cost term about  heading angle between predicted robot location and goal point
                    alpha = math.atan2(goal[1]-predict_y, goal[0]-predict_x)
                    alpha = alpha+2*math.pi if alpha < 0 else alpha
                    k = int(abs(predict_theta) / (2*math.pi))
                    beta = predict_theta+(k+1)*(2*math.pi) if predict_theta < 0 else predict_theta-k*(2*math.pi)
                    if beta > math.pi:
                        beta = beta - 2*math.pi
                    
                    assert alpha >= 0, "ERROR:  alpha angle is negative [alpha={:.4f}]".format(alpha)
                    # assert beta >= 0, "ERROR:  beta angle is negative [beta={:.4f}]".format(beta)
                    if beta > 0:
                        heading_angle = abs(alpha - beta)
                    else:
                        heading_angle = alpha + abs(beta)
                    # print('[Heading angle is {:.2f} degrees]\t[alpha={:.2f}]\t[beta={:.2f}]\t[theta={:.2f}]'.format(heading_angle/math.pi*180, alpha/math.pi*180, beta/math.pi*180, predict_theta/math.pi*180))
                    heading_angle_term = math.pi - heading_angle
                    sum_term1 += heading_angle_term
                    # Cost term about distance to goal for evaluation
                    # dist_goal_cost  =  self.forward_gain * distanceForward
                    # Cost term about distance to closest obstacle for evaluation
                    dist_obstacle_term = self.calculateClosestObstacleDistance(predict_x, predict_y, obstacles)
                    sum_term2 += dist_obstacle_term
                    # if distanceToObstacle < self.SAFE_DIST:
                    #     dist_obstacle_cost = self.obstacle_gain * (self.SAFE_DIST - distanceToObstacle)
                    # else:
                    #     dist_obstacle_cost = 0
                    sum_term3 += vLpossible
                    costVec.append((heading_angle_term, dist_obstacle_term, vLpossible, vRpossible))        
                    # Total cost
                    # benefit = dist_goal_cost - dist_obstacle_cost
                    # if benefit > bestBenefit:
                    #     vLchosen = vLpossible
                    #     vRchosen = vRpossible
                    #     bestBenefit = benefit
        # Normalize cost terms
        for ele in costVec:
            if ele[1] < 0:
                continue
            if round(ele[2], 3) > round(math.sqrt(2*self.MAX_ACC_LINEAR*ele[1]), 3):
                # Cannot reduce to zero before collision
                continue
            heading_angle_term = ele[0] / sum_term1
            dist_obstacle_term = ele[1] / sum_term2
            vel_term = ele[2] / sum_term3
            benefit  =  self.heading_gain*heading_angle_term + self.dist_obstacle_gain*dist_obstacle_term + self.vel_gain*vel_term
            if benefit > bestBenefit:
                bestBenefit = benefit
                vLchosen = ele[2]
                vRchosen = ele[3]
        # Update velocities
        self.linear_vel = vLchosen
        self.angular_vel = vRchosen
        # print('[vLchosen:\t{:.3f}]\t[vRchosen:\t{:.3f}]'.format(vLchosen, vRchosen))


        # Return path to draw
        return predicted_path_to_draw, vLchosen, vRchosen


    def move_robot(self):
        """ Move robot based on chosen velocities in dt time"""
        self.x, self.y, self.theta, tmp_path = self.predict_position(self.linear_vel, self.angular_vel, self.dt) 

    

       
if __name__ == '__main__':
    env = Environment(DWA)
    while Environment.sim_times <= 10:
        if Environment.sim_over == True:
            print('Finished Simulation Times: #{}\tCollision Times: #{}'.format(Environment.sim_times, Environment.collision_times))
            # Start a new simualtion 
            env = Environment(DWA)
            Environment.sim_over = False
        env.run()
        # time.sleep(0.01)
    print("\n" + "* "*50 + "\n")
    print("Collision Rate:\t[{}/{}={:.2f}%]".format(
        Environment.collision_times, 
        Environment.sim_times-1, 
        Environment.collision_times/(Environment.sim_times-1)*100)
        )
    print('Average time to goal:\t{:.2f} secs'.format(Environment.avg_tg))
    print("\n" + "* "*50 + "\n")
    
    # Save tg_vec into txt file
    tg_file = pd.DataFrame(data=[[Environment.tg_vec, Environment.vl_vec, Environment.vr_vec]], columns=["tg", "vl", "vr"])
    tg_file.to_csv("origin_dwa_tg{}.csv".format(env.init_obstacle_num))
    # Plot
    plt.figure()
    plt.plot(list(range(len(Environment.tg_vec))), Environment.tg_vec, 'bo', list(range(len(Environment.tg_vec))), Environment.tg_vec, 'k')
    plt.title("time to goal")
    plt.xlabel("simulation times")
    plt.ylabel("tg(sec)")

    plt.figure()
    plt.subplot(211)
    plt.plot(list(range(len(Environment.vl_vec))), Environment.vl_vec, 'r', label="linear velocity")
    plt.xlabel("simulation steps")
    plt.ylabel("vl(m/s)")
    plt.legend(loc="upper right")
    plt.subplot(212)
    plt.plot(list(range(len(Environment.vr_vec))), Environment.vr_vec, 'g', label="angular velocity")
    plt.xlabel("simulation steps")
    plt.ylabel("vr(rad/s)")
    plt.legend(loc="upper right")
    plt.show()
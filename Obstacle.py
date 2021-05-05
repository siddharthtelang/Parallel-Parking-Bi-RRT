

class Obstacle:
    def __init__(self, clearance):
        self.x = 10
        self.y = 10
        self.clearance = clearance
        self.robot_radius = 0.354 / 2
        self.clearance = self.robot_radius + self.clearance
        self.dynamic_Obstacle = False

        # self.rect1_corner1_x = 3
        # self.rect1_corner1_y = 0
        self.rect1_corner1_x = 0
        self.rect1_corner1_y = 2.75
        self.rect1_length = 3
        self.rect1_width = 0.01

        # self.rect2_corner1_x = 6
        # self.rect2_corner1_y = 0
        self.rect2_corner1_x = 0
        self.rect2_corner1_y = 6.25
        self.rect2_length = 3
        self.rect2_width = 0.01
    def isInObstacleSpace(self, x, y):

        if (x < 1 or x > 9 or y < 1 or y > 9):
          #print('Out of boundary !')
          return 1

        #rectangle obstacle 1
        x1 = self.rect1_corner1_x - self.clearance
        x2 = x1 + self.rect1_length + 2*self.clearance
        y1 = self.rect1_corner1_y - self.clearance
        y2 = y1 + self.rect1_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            #print('Inside rectangle 1, avoid')
            return 1

        #rectangle obstacle 2
        x1 = self.rect2_corner1_x - self.clearance
        x2 = x1 + self.rect2_length + 2*self.clearance
        y1 = self.rect2_corner1_y - self.clearance
        y2 = y1 + self.rect2_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            #print('Inside rectangle 1, avoid')
            return 1

        if self.dynamic_Obstacle == True:
            x1 = self.dynamic_obs_corner_x - self.clearance
            x2 = x1 + self.dynamic_obs_length + 2*self.clearance
            y1 = self.dynamic_obs_corner_y - self.clearance
            y2 = y1 + self.dynamic_obs_width  + 2*self.clearance
            if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
                # print('Hitting new dynamic obstacle')
                return 1

        return 0

    def addNewObstacle(self, x, y, length, width):
        self.dynamic_obs_corner_x = x
        self.dynamic_obs_corner_y = y
        self.dynamic_obs_length = length
        self.dynamic_obs_width = width
        self.dynamic_Obstacle = True

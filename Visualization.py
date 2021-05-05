import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Visualization:
    def __init__(self, obstacle):
        self.radius = obstacle.robot_radius
        self.obstacle = obstacle

    def getRadius(self):
        return self.radius

    def updateMapViz(self, space_map, state, color):
        X, Y, _ = space_map.shape
        transformed_y = state[0]
        transformed_x = X - state[1] -1
        space_map[transformed_x, transformed_y, :] = color

    def addObstacles2Map(self, ax):

        rect1_corner1 = (self.obstacle.rect1_corner1_x - self.obstacle.clearance, self.obstacle.rect1_corner1_y - self.obstacle.clearance)
        rect1_length = self.obstacle.rect1_length + (2 * self.obstacle.clearance)
        rect1_width = self.obstacle.rect1_width + (2 * self.obstacle.clearance)

        rect2_corner1 = (self.obstacle.rect2_corner1_x - self.obstacle.clearance, self.obstacle.rect2_corner1_y - self.obstacle.clearance)
        rect2_length = self.obstacle.rect2_length + (2 * self.obstacle.clearance)
        rect2_width = self.obstacle.rect2_width + (2 * self.obstacle.clearance)

        rect1 = patches.Rectangle(rect1_corner1, rect1_length, rect1_width, linewidth=1, edgecolor='r', facecolor='none')
        rect2 = patches.Rectangle(rect2_corner1, rect2_length, rect2_width, linewidth=1, edgecolor='r', facecolor='none')

        rect3 = None
        if self.obstacle.dynamic_Obstacle == True:
            rect3_corner1 = (self.obstacle.dynamic_obs_corner_x - self.obstacle.clearance, self.obstacle.dynamic_obs_corner_y - self.obstacle.clearance)
            rect3_length = self.obstacle.dynamic_obs_length + (2 * self.obstacle.clearance)
            rect3_width = self.obstacle.dynamic_obs_width + (2 * self.obstacle.clearance)
            rect3 = patches.Rectangle(rect3_corner1, rect3_length, rect3_width, linewidth=1, edgecolor='b', facecolor='none')

        ax.add_patch(rect1)
        ax.add_patch(rect2)
        if (rect3 is not None):
            ax.add_patch(rect3)

        return ax



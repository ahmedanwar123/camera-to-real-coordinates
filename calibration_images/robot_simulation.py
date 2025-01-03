import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class RobotSimulation:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        (self.arm,) = self.ax.plot([], [], "bo-")

    def update_arm(self, target_position):
        x = [0, target_position[0]]
        y = [0, target_position[1]]
        self.arm.set_data(x, y)
        plt.draw()
        plt.pause(0.01)

    def show(self):
        plt.show()

"""
RRT algorithm with no obstacle
"""

import numpy as np
import math 
import matplotlib.pyplot as plt

class RRT():

    def __init__(self, q_init, q_final):
        """
        Initialization
        Input:
            q_init: desired initial coordinate
            q_final: desired final coordinate
        """
        self.size = 100 # Size of the map
        self.K = 500 # Number of iteration
        self.delta = 2 # Increment distance
        self.q_init = q_init # Initial coordinate of the node
        self.q_final = q_final # Final coordinate of the node 
        self.dis_thresh = 4  # Distance threshold 

        # Store the path from start to end node
        self.qs = [q_init] # List to store coordinates of each node
        self.qs_parent = [[None,None]] # List to store the parent node of each node
        self.fig = plt.figure() # Initialize figure

    def rand_gen(self):
        """
        To generate nodes with random coordinates
        """
        self.q_rand = [np.random.rand()*self.size , np.random.rand()*self.size]
        return self.q_rand

    def nearest_vertex(self):
        """
        To find the nearest node to the random node from the node list
        """
        l_dis = []
        for v in self.qs:
            dis = math.dist(v,self.q_rand)
            l_dis.append(dis)
        min_dis_idx = np.argmin(l_dis)
        self.q_near = self.qs[min_dis_idx]
        return self.q_near

    def new_config(self):
        """
        To generate a new node
        """
        dis = math.dist(self.q_near,self.q_rand)
        x_new = self.q_near[0] + (self.q_rand[0]-self.q_near[0])/(dis/self.delta)
        y_new = self.q_near[1] + (self.q_rand[1]-self.q_near[1])/(dis/self.delta)
        self.q_new = [x_new,y_new]
        return self.q_new

    def generate_figure(self):
        """
        To Implement RRT algorithm generate an animated figure
        """
        i = 0
        ax = self.fig.add_subplot(111)
        plt.xlim(0,100)
        plt.ylim(0,100)
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot(self.q_init[0],self.q_init[1],'*',color = 'blue')
        ax.plot(self.q_final[0],self.q_final[1],'*',color = 'green')
        q_current = self.q_init
        while math.dist(q_current,self.q_final) >= self.dis_thresh:
            i+=1
            ax.set_title("Iterations: "+str(i))
            self.q_rand = self.rand_gen()
            self.q_near = self.nearest_vertex()
            self.q_new = self.new_config()
            self.qs.append(self.q_new)
            self.qs_parent.append(self.q_near)
            x_values = [self.q_near[0],self.q_new[0]]
            y_values = [self.q_near[1],self.q_new[1]]
            plt.plot(x_values,y_values,color = "black")
            self.fig.canvas.draw()
            plt.pause(0.01)
            q_current = self.q_new
        
        self.qs.append(self.q_final)
        self.qs_parent.append(self.q_new)
        x_values = [self.q_new[0],self.q_final[0]]
        y_values = [self.q_new[1],self.q_final[1]]
        plt.plot(x_values,y_values,color = "black")
        self.fig.canvas.draw()
        plt.pause(0.01)

        q_parent = self.qs_parent[-1]
        q_current = self.qs[-1]
        while q_parent != [None,None]:
            x_values = [q_current[0],q_parent[0]]
            y_values = [q_current[1],q_parent[1]]
            plt.plot(x_values,y_values,color = "red")
            self.fig.canvas.draw()
            plt.pause(0.01)
            parent_idx = self.qs.index(q_current)
            q_current = q_parent
            q_parent = self.qs_parent[parent_idx]

        plt.show()

        return 


if __name__ == "__main__":
    rrt = RRT(q_init=[10,10],q_final=[80,90])
    rrt.generate_figure()
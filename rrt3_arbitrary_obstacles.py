"""
RRT algorithm with arbitrary obstacles
"""

import numpy as np
import math 
import matplotlib.pyplot as plt
import imageio as iio

class RRT():

    def __init__(self, q_init, q_final,img):
        """
        Initialization
        Input:
            q_init: desired initial coordinate
            q_final: desired final coordinate
            img: png image data
        """
        self.size = len(img) # Size of the map
        self.K = 500 # Number of iteration
        self.delta = 5 # Increment distance
        self.q_init = q_init # Initial coordinate of the node
        self.q_final = q_final # Final coordinate of the node
        self.img = img # PNG image data
        self.dis_thresh = 10  # Distance threshold 

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
        x_new = int(self.q_near[0] + (self.q_rand[0]-self.q_near[0])/(dis/self.delta))
        y_new = int(self.q_near[1] + (self.q_rand[1]-self.q_near[1])/(dis/self.delta))
        self.q_new = [x_new,y_new]
        return self.q_new
    

    def check_point_collision(self,point):
        """
        To check if the point is in the image
        """
        if self.img[point[1]][point[0]][0] == 0:
            return True
        else:
            return False


    
    def generate_figure(self):
        """
        To Implement RRT algorithm generate an animated figure
        """
        i = 0
        ax = self.fig.add_subplot(111)
        plt.imshow(self.img)
        ax.plot(self.q_init[0],self.q_init[1],'*',color = 'blue')
        ax.plot(self.q_final[0],self.q_final[1],'*',color = 'green')
        q_current = self.q_init
        
        while math.dist(q_current,self.q_final) >= self.dis_thresh:
            i+=1
            ax.set_title("Iterations: "+str(i))
            self.q_rand = self.rand_gen()
            self.q_near = self.nearest_vertex()
            self.q_new = self.new_config()
            if self.check_point_collision(self.q_new):
                continue
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
    img = iio.imread('image/JZ_192*192.png')
    rrt = RRT(q_init=[50,150],q_final=[175,50],img=img)
    rrt.generate_figure()
"""
RRT algorithm with circular obstacles
"""

import numpy as np
import math 
import matplotlib.pyplot as plt

class RRT():

    def __init__(self, q_init, q_final, num_circle, size_circle):
        """
        Initialization
        Input:
            q_init: desired initial coordinate
            q_final: desired final coordinate
            num_circle: desired number of circular obstacles
            size_circle: desired size of the circles
        """
        self.size = 100 # Size of the map
        self.K = 500 # Number of iteration
        self.delta = 5 # Increment distance
        self.q_init = q_init # Initial coordinate of the node
        self.q_final = q_final # Final coordinate of the node
        self.num_circle = num_circle # Number of circles
        self.size_circle = size_circle # Size of circles 
        self.dis_thresh = 10  # Distance threshold 
        self.generate_circles() # Implement generate circles function
        self.remove_bad_circles() # Implement remove bad circles function

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
    
    def generate_circles(self):
        """
        To generate circles with random positions and sizes. The circles'
        radii and their x,y positions will be stored in a array
        """
        self.circles = np.zeros([self.num_circle,3])
        self.circles[:,0] = np.random.normal(self.size_circle, 1.0, size=(self.num_circle,))
        self.circles[:,1:] = np.random.uniform(self.size_circle, self.size-self.size_circle, size=(self.num_circle,2))
        return self.circles

    def check_point_collision(self,circle,point):
        """
        To check if the point is in the circle
        """
        circle_center = (circle[1],circle[2])
        circle_rad = circle[0]
        dis = math.dist(circle_center,point)
        if dis <= circle_rad:
            return True
        else:
            return False

    def check_edge_collision(self,circle,edge):
        """
        To check if the straight line collides with the circle
        """
        x0, y0 = circle[1:]
        x1, y1 = edge[0]
        x2, y2 = edge[1]
        dis0=abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/(math.sqrt((x2-x1)**2+(y2-y1)**2))
        dis1=math.dist((x0,y0),(x1,y1))
        dis2=math.dist((x0,y0),(x2,y2))
        delta = math.dist((x1,y1),(x2,y2))
        q1=math.sqrt(dis1**2-dis0**2)
        q2=math.sqrt(dis2**2-dis0**2)
        if q1 <= delta and q2 <= delta: # within the line
            x = dis0
        elif q2 > delta:
            x = dis1
        elif q1 > delta:
            x = dis2
    
        if x <= circle[0]: # collide
            return True
        else: # does not collide
            return False

    def check_collision_free_path(self):
        for c in self.circles:
            edge = [self.q_new, self.q_final]
            if self.check_edge_collision(c,edge):
                return False
        return True


    def remove_bad_circles(self):
        """
        To remove circles that intersect with the initial and final point
        """
        for i,c in enumerate(self.circles):
            if self.check_point_collision(c,self.q_init) or self.check_point_collision(c,self.q_final):
                self.circles = np.delete(self.circles,i,axis=0)
        return self.circles

    def check_collision(self):
        """
        To check if the new node collides with all the circles
        """
        for c in self.circles:
            if self.check_point_collision(c,self.q_new) or self.check_edge_collision(c,[self.q_near,self.q_new]):
                return True
        return False

    
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
        for c in self.circles:
            circle = plt.Circle((c[1],c[2]),c[0],alpha=0.5)
            ax.add_patch(circle)
        while math.dist(q_current,self.q_final) >= self.dis_thresh:
            i+=1
            ax.set_title("Iterations: "+str(i))
            self.q_rand = self.rand_gen()
            self.q_near = self.nearest_vertex()
            self.q_new = self.new_config()
            if self.check_collision():
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
    rrt = RRT(q_init=[10,10],q_final=[80,90],num_circle=20, size_circle=5)
    rrt.generate_figure()
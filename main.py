import numpy as np
import random 
from tkinter import *
from tkinter import Tk, Canvas, Frame, BOTH

class Example(Frame):

  def __init__(self):
    super().__init__()
    self.initUI()
    self.nodes = []

  def initUI(self):

    self.master.title("RRT")
    self.pack(fill=BOTH, expand=1)

    #obstacles
    canvas = Canvas(self)
    canvas.create_oval(10, 10, 80, 80, outline="#f11", fill="#1f1", width=2)
    canvas.create_oval(500, 300, 600, 400, outline="#f11", fill="#1f1", width=2)
    canvas.create_rectangle(200, 130, 290, 160, outline="#f11", fill="#1f1", width=2)
    
    #starting node
    x_start = random.random() * 500
    y_start = random.random() * 500
    # while (x_start>10 and x_start<80 and y_start>10 and y_start<80) or (x_start>500 and x_start<600 and y_start>300 and y_start<400) or (x_start>200 and x_start<290 and y_start>130 and y_start<160):
    #         pos[0] = random.random() * 500
    #         pos[1] = random.random() * 500
    canvas.create_oval(x_start, y_start, x_start+25,y_start+25, fill = "red")

    #goal
    x_end = random.random() * 500
    y_end = random.random() * 500
    # while (x_end>10 and x_end<80 and y_end>10 and y_end<80) or (x_end>500 and x_end<600 and y_end>300 and y_end<400) or (x_end>200 and x_end<290 and y_end>130 and y_end<160):
    #   x_end = random.random() * 500
    #   y_end = random.random() * 500
    canvas.create_oval(x_end, y_end, x_end+50,y_end+50, fill = "blue")

    #create graph 
    vertices = [(x_start, y_start)]
    edges = []
    map_to_distance = {(x_start, y_start):0}
    neighbors = {0:[]}
    distances = {0:0}
    dx = x_end - x_start
    dy = y_end - y_start

    def add_vertex(pos): 
      try:
          # Get path length 
          while (pos[0]>10 and pos[0]<80 and pos[1]>10 and pos[1]<80) or (pos[0]>500 and pos[0]<600 and pos[1]>300 and pos[1]<400) or (pos[0]>200 and pos[0]<290 and pos[1]>130 and pos[1]<160):
            pos[0] = random.random() * 500
            pos[1] = random.random() * 500
          path_dist = map_to_distance[pos]
          canvas.create_oval(pos[0], pos[1], pos[0] + 25, pos[1]+ 25, fill = "red")
      except:
          # If path length has not been calculated already...
          # Length of path = # of vertices 
          path_dist = len(vertices)
          vertices.append(pos)
          map_to_distance[pos] = path_dist
          neighbors[path_dist] = []
      return path_dist

    def add_edge(pos1, pos2, cost):
        edges.append((pos1, pos2))
        neighbors[pos1].append((pos2, cost))
        neighbors[pos2].append((pos1, cost))
        canvas.create_line(pos1[0], pos1[1], pos2[0], pos2[1], fill = "black")

    def randomPosition(): #creating random start position
        rx = random()
        ry = random()

        posx = x_start - (dx / 2.) + rx * dx * 2
        posy = y_start - (dy / 2.) + ry * dy * 2
        return posx, posy

    canvas.pack(fill=BOTH, expand=1)

    class Line():
      def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.direction = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.direction)
        self.direction /= self.dist # normalize

      def path(self, t):
        return self.p + t * self.direction

    def distance(x, y):
      return np.linalg.norm(np.array(x) - np.array(y))
      
    def Intersection(line, center, radius):
      ''' Check line-sphere (circle) intersection to identify if collisions with obstacles
      '''
      a = np.dot(line.direction, line.direction)
      b = 2 * np.dot(line.direction, line.p - center)
      c = np.dot(line.p - center, line.p - center) - radius * radius

      discriminant = b * b - 4 * a * c
      if discriminant < 0:
        return False

      t1 = (-b + np.sqrt(discriminant)) / (2 * a);
      t2 = (-b - np.sqrt(discriminant)) / (2 * a);

      if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False
      return True

    def isInObstacle(vex, obstacles, radius):
      for obs in obstacles:
        if distance(obs, vex) < radius:
          return True
      return False

    def isThruObstacle(line, obstacles, radius):
      for obs in obstacles:
        if Intersection(line, obs, radius):
          return True
      return False

    def nearest(vex, obstacles, radius):
      Nvex = None
      Nidx = None
      minDist = float("inf")

      for idx, v in enumerate(vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
          continue

        dist = distance(v, vex)
        if dist < minDist:
          minDist = dist
          Nidx = idx
          Nvex = v
      print("newvertex:", Nvex)
      return Nvex, Nidx
       
    def newVertex(randvex, nearvex, stepSize):
      direction = np.array(randvex) - np.array(nearvex)
      length = np.linalg.norm(direction)
      direction = (direction / length) * min (stepSize, length)

      newvex = (nearvex[0]+direction[0], nearvex[1]+direction[1])
      return newvex

    def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
      for _ in range(n_iter):
        randvex = randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = add_vertex(newvex)
        dist = distance(newvex, nearvex)
        print("Values: ", newidx, nearidx, dist)
        add_edge(newidx, nearidx, dist)

        dist = distance(newvex, endpos)
        if dist < 2 * radius:
            endidx = add_vertex(endpos)
            add_edge(newidx, endidx, dist)
            success = True
            break

    def isInGoal(x, y, z, a):
      goal = canvas.find_overlapping(x, y, z, a)
      if vertices[len(vertices)-1] not in goal:
        return True
      return False

    startpos = (0., 0.)
    endpos = (5., 5.)
    obstacles = [(3., 3.), (7., 6.)]
    n_iter = 200
    radius = 0.5
    stepSize = 0.7
    G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)


def main():
  root = Tk()
  ex = Example()
  root.geometry("1000x1000")
  while vertices[len(vertices)-1] not in goal:
    startpos = (0., 0.)
    endpos = (5., 5.)
    #obstacles = [(3., 3.), (7., 6.)]
    n_iter = 200
    radius = 0.5
    stepSize = 0.7
  root.mainloop()


if __name__ == '__main__':
  main()
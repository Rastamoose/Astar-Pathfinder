import pygame, math
from queue import PriorityQueue

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

#COLOURS
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE= (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Node:
  def __init__(self, row, col, width, total_rows):
    self.row = row
    self.col = col
    self.x = row * width
    self.y = col * width
    self.colour = WHITE
    self.neighbours = []
    self.width = width
    self.total_rows = total_rows
    
  def get_pos(self):
    return self.row, self.col
  
  def is_closed(self):       #tells if node has already been considered
    return self.colour == RED       #if it's red, then it has been considered
  
  def is_open(self):
    return self.colour == GREEN       #tells if node is in the open set
  
  def is_barrier(self):              #tells if node is an obstacle/barrier
    return self.colour == BLACK
  
  def is_start(self):               #tells if node is starting pos
    return self.colour == ORANGE
    
  def is_end(self):                 #tells if node is end pos
    return self.colour == TURQUOISE
  
  def reset(self):            #changes colour back to white
    self.colour = WHITE
    
  def make_start(self):     #turns node to start pos
    self.colour = ORANGE
    
  def make_closed(self):      #turns node to closed
    self.colour = RED
    
  def make_open(self):    #turns node to open
    self.colour = GREEN
    
  def make_barrier(self):   #turns node to barrier    
    self.colour = BLACK
    
  def make_end(self):     #turns node to end node
    self.colour = TURQUOISE
    
  def make_path(self):    #turns node to part of path
    self.colour = PURPLE
    
  def draw(self, win):      #draws node as cube
    pygame.draw.rect(win, self.colour, (self.x, self.y, self.width, self.width))
    
  def update_neighbours(self, grid):
    self.neighbours = []
    if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():    #if self is not on bottom row and node below it isnt a barrier
      self.neighbours.append(grid[self.row + 1][self.col])                   #then node underneath it is a neighbour  
    
    if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():    #if self is not on top row and node on top isnt a barrier
      self.neighbours.append(grid[self.row - 1][self.col])                   #then node on top is a neighbour  
    
    if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():    #if self is not on rightmost column and node to right isnt a barrier
      self.neighbours.append(grid[self.row][self.col + 1])                   #then node to right is a neighbour  
    
    if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():    #if self is not on leftmost column and node on left isnt a barrier
      self.neighbours.append(grid[self.row][self.col - 1])                   #then node on left is a neighbour  
  

  
nodes = []
def h(p1, p2):      #heuristic function - h is estimated distance from current node to end node (ignoring grid)
  x1, y1 = p1       #using manhattan/taxicab distance (uses L shapes because cant draw diagonal on square grid (even if you did it would be the same distance))
  x2, y2 = p2
  return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw):
  while current in came_from:
    current = came_from[current]    #goes back node by node turning each into the path (purple)
    current.make_path()
    draw()

def algorithm(draw, grid, start, end):
  count = 0
  open_set = PriorityQueue()
  open_set.put((0, count, start))                         #enter start node with its f value (0). count added so if two nodes have same f value we use the one that came first
  came_from = {}                                          #keeps track of which node current node came from
  g_score = {node: float("inf") for row in grid for node in row}    #sets all g scores to infinity - g is distance (using grid) from initial node (node from) to current
  g_score[start] = 0
  f_score = {node: float("inf") for row in grid for node in row}    #sets all f scores to infinity - f is g + h
  f_score[start] = h(start.get_pos(), end.get_pos())                #f score for start will therefore just be h

  open_set_hash = {start}       #used to help keep track of items in open_set (because PriorityQueue doesn't allow for searching)
  
  while not open_set.empty():       #if open set is empty, all possibilities have been considered
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
      
    current = open_set.get()[2]     #pops node with lowest f score, if f score same then with lowest count - if starting then only think in open_set is start so goes for that
    open_set_hash.remove(current)   #syncs both sets  
    
    if current == end:    #if node we're at is the end, shortest path has been found 
      reconstruct_path(came_from, end, draw)    #so make the path
      end.make_end()
      return True
    
    for neighbour in current.neighbours:
      temp_g_score = g_score[current] + 1     #gets neighbour node's g_score (assuming distance is 1)
      
      if temp_g_score < g_score[neighbour]:   #if this distance from current to neighbour is less than neighbour's old g_score
        came_from[neighbour] = current         #then, it is faster route to get to neighbour so update details
        g_score[neighbour] = temp_g_score
        f_score[neighbour] = temp_g_score + h(neighbour.get_pos(), end.get_pos())
        if neighbour not in open_set_hash:      #if neighbour not in open_set (hash used to search)
          count += 1                            #then we add it into the set (so counter goes up 1)
          open_set.put((f_score[neighbour], count, neighbour))
          open_set_hash.add(neighbour)
          neighbour.make_open()
          
    draw()
    
    if current != start:      #if current node not start node
      current.make_closed()     #set to close as it has been considered
      
  return False      #path not found
  
def make_grid(rows, width):
  grid = []
  gap = width // rows
  for i in range(rows):
    grid.append([])
    for j in range(rows):
      node = Node(i, j, gap, rows)
      grid[i].append(node)
      
  return grid

def draw_grid(win, rows, width):    #draws gridlines 
  gap = width // rows
  for i in range(rows):
    pygame.draw.line(win, GREY, (0,i * gap), (width, i * gap))
    for j in range(rows):
      pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))
      
def draw(win, grid, rows, width):   #draws everything
  win.fill(WHITE)
  
  for row in grid:
    for node in row:
      node.draw(win)
  
  draw_grid(win, rows, width)
  pygame.display.update()
  
def get_clicked_pos(pos, rows, width):
  gap = width // rows
  x,y = pos
  
  row = x // gap      #gets what node (square) is clicked
  col = y // gap
  
  return row, col

def main(win, width):
  ROWS = 50
  
  grid = make_grid(ROWS, width)
  
  start = None
  end = None
  
  run = True
  
  map = []
  while run:
    draw(win, grid, ROWS, width)
    
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        run = False
      
      if pygame.mouse.get_pressed()[0]:     #if lmb pressed
        pos = pygame.mouse.get_pos()
        row, col = get_clicked_pos(pos, ROWS, width)
        node = grid[row][col]         #index row, col in grid
        
        if not start and node != end:              #if no start pos and node not end, then make this start
          start = node
          start.make_start() 
          
        elif not end and node != start:           #if no end pos and node not start, then make this end
          end = node 
          end.make_end()       
          
        elif node != end and node != start:      #if it's neither start nor finish then it must be player putting down a barrier
          node.make_barrier()       
      elif pygame.mouse.get_pressed()[2]:   #if rmb pressed
          pos = pygame.mouse.get_pos()
          row, col = get_clicked_pos(pos, ROWS, width)
          node = grid[row][col]                           #delete whatever node marked as (reset it)
          node.reset()
          if node == start:
            start = None          #reset start and end if that is what is being deleted
          elif node == end:
            end = None
            
      if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE and start and end:     #if there is a start and end node
          for row in grid:
            for node in row:
              node.update_neighbours(grid)
          algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)    #call algorithm func
        
        if event.key == pygame.K_c:
          start = None
          end = None
          grid = make_grid(ROWS, width)
             
  pygame.quit()
  
main(WIN, WIDTH)

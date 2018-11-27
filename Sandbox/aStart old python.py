#!/usr/bin/python
#source for the tutorial: https://www.youtube.com/watch?v=ob4faIum4kQ
from queue import PriorityQueue

class State(object): #this class stores all the results of Astar algorithm functions writte below  
	
	def __init__(self, value, parent):
		self.children = []
		self.parent = parent
		self.value = value
		self.dist = 0
		if parent:
			self.path = parent.path[:] #[:] makes a copy of a list of parents and stores it in self.path
			self.path.append(value)
			self.start = parent.start
			self.goal = parent.goal
		else:
			self.path = [value]
			self.start = 0
			self.goal = 0
	
	def GetDist(self):
		pass
	
	def CreateChildren(self):
		pass 

class State_String(State): #this is the class defining the state (I think so)
	
	def __init__(self, value, parent, start = 0, goal = 0):
		super(State_String, self).__init__(value, parent)#constructor
		self.dist = self.GetDist()

	def GetDist(self):
		if self.value == self.goal: #if the goal location is the same as start location
			return 0
		dist = 0
		for i in range(self.goal): #in this example the function goes through the each letter of the goal
			pos = self.goal[i]
			dist += abs(i - self.value.index(pos)) #index of the position; in this case it's an index of a letter, we need an index of waypoint
		return dist


	def CreateChildren(self): # the function goes through all the possible combinations of the waypoints
		if not self.children: #precaution so no children are created twice
			for i in range(self.goal-1):
				val = self.value #the value is stored 
				val = val[:i] + val[i+1] + val[i] + val[i+2:] #this is where switcheru between letters in the word works in this example: 1st switched with 2nd and 2nd with 3rd, etc <-- this is where we need a grid with waypoints
				child = State_String(val, self)
				self.children.append(child) #adding the child to the children list


class Astar_Solver:
	
	def __init__(self, start, goal):
		self.path = [] #this stores the solution from start state to goal state
		self.visitQueue = [] #this keeps track of all the children that have been visited, so we don't end up in forever loop
		self.PriorityQueue = PriorityQueue()
		self.start = start 
		self.goal = goal

	def Solve(self):
		startState = State_String(self.start, 0, self.start, self.goal)
		count = 0 #this creates an id for each of the children
		self.PriorityQueue.put((0, count, startState))
		while (not self.path and (not self.PriorityQueue.empty())): #while path is empty and while priorityqueue has a size
			closestChild = self.PriorityQueue.get()[2] #[2] is the array, where "2" corresponds to the startState 
			closestChild.CreateChildren()
			self.visitQueue.append(closestChild.value) #keeps track on which children has been visited
			for child in closestChild.children:
				if child.value not in self.visitQueue:
					count += 1
					if not child.dist: 
						self.path = child.path 
						break # this breaks inner for loop and program goes back into while loop
					self.PriorityQueue.put((child.dist, count, child))
		print(len(self.path))
		if not self.path:
			pass
			#print ("Goal of " + self.goal "is not possible")
		return self.path

### main 
start1 = "qwerty"
goal1 = "wtreqy"
a = Astar_Solver (start1, goal1) #initialize the solver
a.Solve() #call solve function
for i in range(len(a.path)): #get the result
	print(a.path[i])


##### the task now is to change the logic from rearranging the letters to selecting waypoints. 
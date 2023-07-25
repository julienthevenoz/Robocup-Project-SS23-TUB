
import pprint
from collections import defaultdict



""" Implementation of a graph Data Structure for finding shortest Path. The GridGraph automatically
	builds the grid in the __init__ method. Some parts of this class are inspired by:
	https://stackoverflow.com/questions/19472530/representing-graphs-data-structure-in-python
	The methods bfs, pathTo, hasPathTo are implemented following the AlgoDat Lecture Script
	by Prof. Benjamin Blankertz and Vera R  """
class GridGraph(object):

	def __init__(self, size):
	    self.adj = defaultdict(set)
	    self.parent = defaultdict(None)
	    x = lambda: False
	    self.visited = defaultdict(x)
	    self.size = size
	    self.startnode = None
	    self.goal = None
	    self.position = None

	    # build the grid
	    for x in range(size[0] - 1):
	    	for y in range(size[1]):
	    		self.addEdge((x,y), (x+1,y))
	    		self.addEdge((y,x), (y,x+1))

	def addEdge(self, v, w):
	    """ Add edge between v and w """

	    self.adj[v].add(w)
	    self.adj[w].add(v)

	def remove(self, node):
	    """ Remove all references to node """

	    for n, cxns in self.adj.iteritems():  # python3: items(); python2: iteritems()
	        try:
	            cxns.remove(node)
	        except KeyError:
	            pass
	    try:
	        del self.adj[node]
	    except KeyError:
	        pass


	def bfs(self,startnode):
		""" Run Breadth-First-Search """
		
		self.parent.clear()
		self.visited.clear()
		self.startnode = startnode
		queue = [startnode]

		while queue:
			v = queue.pop(0)
			for w in self.adj[v]:
				if not self.visited[w]:
					self.parent[w] = v
					self.visited[w] = True
					queue.append(w)


	def hasPathTo(self, goal):
		""" returns true if there is a path from v to startnode """
		return self.visited[goal]			

	def pathTo(self, goal):
		""" returns the short path from v to startnode """
		
		if not self.hasPathTo(goal):
			return None	

		self.goal = goal
		path = []
		w = goal
		while w != self.startnode:
			path.append(w)
			w = self.parent[w]
		path.append(self.startnode)
		path.reverse()
		return path

	def setPosition(self,position):
		""" set the current position in the grid
		only relevant for string representation"""
		self.position = position

	def movesTo(self, goal):
		""" returns the moves necessary to get from v to startnode """

		rPosToMove = { (0,-1): "north",
				 (1,0): "east",
				 (0,1): "south",
				 (-1,0): "west"
		}

		path = self.pathTo(goal)
		if not path:
			return None
		prev = path.pop(0)
		moves = []
		for node in path:
			rPos = (node[0] - prev[0], node[1] - prev[1])
			prev = node
			moves.append(rPosToMove[rPos])
		return moves

	def __str__(self):
		""" returns a string representation of the graph """
		string = ""
		path = []
		if self.goal != None:
			path = self.pathTo(self.goal)
		if path == None:
			path = []

		for y in range(self.size[1]):
			for x in range(self.size[0]):
				
				if (x,y) == self.startnode:
					string += "S"
				elif (x,y) == self.position:
					string += "N"	
				elif (x,y) == self.goal:
					string += "G"	
				elif (x,y) in path:
					string += "="
				elif self.adj[(x,y)]:
					string += "."
				else:
					string += "#"
			string += "\n"
		return string

def main():

	g = GridGraph((7,7))
	g.remove((2,1))
	g.remove((3,2))
	g.bfs((3,3))
	g.pathTo((3,0))
	print(g)

if __name__ == "__main__":
	main()










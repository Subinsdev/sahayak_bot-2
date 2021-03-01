# # Python program to check if a given graph is Eulerian or not
# #Complexity : O(V+E)
#
# from collections import defaultdict
#
# #This class represents a undirected graph using adjacency list representation
# class Graph:
#
# 	def __init__(self,vertices):
# 		self.V= vertices #No. of vertices
# 		self.graph = defaultdict(list) # default dictionary to store graph
#
# 	# function to add an edge to graph
# 	def addEdge(self,u,v):
# 		self.graph[u].append(v)
# 		self.graph[v].append(u)
#
# 	#A function used by isConnected
# 	def DFSUtil(self,v,visited):
# 		# Mark the current node as visited
# 		visited[v]= True
#
# 		#Recur for all the vertices adjacent to this vertex
# 		for i in self.graph[v]:
# 			if visited[i]==False:
# 				self.DFSUtil(i,visited)
#
#
# 	'''Method to check if all non-zero degree vertices are
# 	connected. It mainly does DFS traversal starting from
# 	node with non-zero degree'''
# 	def isConnected(self):
#
# 		# Mark all the vertices as not visited
# 		visited =[False]*(self.V)
#
# 		# Find a vertex with non-zero degree
# 		for i in range(self.V):
# 			if len(self.graph[i]) > 1:
# 				break
#
# 		# If there are no edges in the graph, return true
# 		if i == self.V-1:
# 			return True
#
# 		# Start DFS traversal from a vertex with non-zero degree
# 		self.DFSUtil(i,visited)
#
# 		# Check if all non-zero degree vertices are visited
# 		for i in range(self.V):
# 			if visited[i]==False and len(self.graph[i]) > 0:
# 				return False
#
# 		return True
#
#
# 	'''The function returns one of the following values
# 	0 --> If grpah is not Eulerian
# 	1 --> If graph has an Euler path (Semi-Eulerian)
# 	2 --> If graph has an Euler Circuit (Eulerian) '''
# 	def isEulerian(self):
# 		# Check if all non-zero degree vertices are connected
# 		if self.isConnected() == False:
# 			return 0
# 		else:
# 			#Count vertices with odd degree
# 			odd = 0
# 			for i in range(self.V):
# 				if len(self.graph[i]) % 2 !=0:
# 					odd +=1
#
# 			'''If odd count is 2, then semi-eulerian.
# 			If odd count is 0, then eulerian
# 			If count is more than 2, then graph is not Eulerian
# 			Note that odd count can never be 1 for undirected graph'''
# 			if odd == 0:
# 				return 2
# 			elif odd == 2:
# 				return 1
# 			elif odd > 2:
# 				return 0
#
#
# 	# Function to run test cases
# 	def test(self):
# 		res = self.isEulerian()
# 		if res == 0:
# 			print "graph is not Eulerian"
# 		elif res ==1 :
# 			print "graph has a Euler path"
# 		else:
# 			print "graph has a Euler cycle"
#
#
#
# #Let us create and test graphs shown in above figures
# # g1 = Graph(5);
# # g1.addEdge(1, 0)
# # g1.addEdge(0, 2)
# # g1.addEdge(2, 1)
# # g1.addEdge(0, 3)
# # g1.addEdge(3, 4)
# # g1.test()
#
# g2 = Graph(4)
# g2.addEdge(1, 2)
# g2.addEdge(1, 2)
# g2.addEdge(1, 3)
# g2.addEdge(1, 3)
# g2.addEdge(2, 3)
# g2.addEdge(2, 3)
# g2.addEdge(3, 0)
# g2.addEdge(3, 0)
# g2.addEdge(2, 0)
# g2.addEdge(2, 0)
# g2.addEdge(1, 0)
# g2.addEdge(1, 0)
# g2.test();
#
# # g3 = Graph(5)
# # g3.addEdge(1, 0)
# # g3.addEdge(0, 2)
# # g3.addEdge(2, 1)
# # g3.addEdge(0, 3)
# # g3.addEdge(3, 4)
# # g3.addEdge(1, 3)
# # g3.test()
#
# #Let us create a graph with 3 vertices
# # connected in the form of cycle
# # g4 = Graph(3)
# # g4.addEdge(0, 1)
# # g4.addEdge(1, 2)
# # g4.addEdge(2, 0)
# # g4.test()
#
# # Let us create a graph with all veritces
# # with zero degree
# # g5 = Graph(3)
# # g5.test()
#
# #This code is contributed by Neelam Yadav



# Python program print Eulerian Trail in a given Eulerian or Semi-Eulerian Graph

from collections import defaultdict

#This class represents an undirected graph using adjacency list representation
class Graph:

	def __init__(self,vertices):
		self.V= vertices #No. of vertices
		self.graph = defaultdict(list) # default dictionary to store graph
		self.Time = 0

	# function to add an edge to graph
	def addEdge(self,u,v):
		self.graph[u].append(v)
		self.graph[v].append(u)

	# This function removes edge u-v from graph
	def rmvEdge(self, u, v):
		for index, key in enumerate(self.graph[u]):
			if key == v:
				self.graph[u].pop(index)
		for index, key in enumerate(self.graph[v]):
			if key == u:
				self.graph[v].pop(index)

	# A DFS based function to count reachable vertices from v
	def DFSCount(self, v, visited):
		count = 1
		visited[v] = True
		for i in self.graph[v]:
			if visited[i] == False:
				count = count + self.DFSCount(i, visited)
		return count

	# The function to check if edge u-v can be considered as next edge in
	# Euler Tour
	def isValidNextEdge(self, u, v):
		# The edge u-v is valid in one of the following two cases:

		# 1) If v is the only adjacent vertex of u
		if len(self.graph[u]) == 1:
			return True
		else:
			'''
			2) If there are multiple adjacents, then u-v is not a bridge
				Do following steps to check if u-v is a bridge

			2.a) count of vertices reachable from u'''
			visited =[False]*(self.V)
			count1 = self.DFSCount(u, visited)

			'''2.b) Remove edge (u, v) and after removing the edge, count
				vertices reachable from u'''
			self.rmvEdge(u, v)
			visited =[False]*(self.V)
			count2 = self.DFSCount(u, visited)

			#2.c) Add the edge back to the graph
			self.addEdge(u,v)

			# 2.d) If count1 is greater, then edge (u, v) is a bridge
			return False if count1 > count2 else True


	# Print Euler tour starting from vertex u
	def printEulerUtil(self, u):
		#Recur for all the vertices adjacent to this vertex
		for v in self.graph[u]:
			#If edge u-v is not removed and it's a a valid next edge
			if self.isValidNextEdge(u, v):
				print("%d-%d " %(u,v)),
				self.rmvEdge(u, v)
				self.printEulerUtil(v)



	'''The main function that print Eulerian Trail. It first finds an odd
degree vertex (if there is any) and then calls printEulerUtil()
to print the path '''
	def printEulerTour(self):
		#Find a vertex with odd degree
		u = 0
		for i in range(self.V):
			if len(self.graph[i]) %2 != 0 :
				u = i
				break
		# Print tour starting from odd vertex
		print ("\n")
		self.printEulerUtil(u)

# Create a graph given in the above diagram

g2 = Graph(4)
g2.addEdge(1, 2)
g2.addEdge(1, 2)
g2.addEdge(1, 3)
g2.addEdge(1, 3)
g2.addEdge(2, 3)
g2.addEdge(2, 3)
g2.addEdge(3, 0)
g2.addEdge(3, 0)
g2.addEdge(2, 0)
g2.addEdge(2, 0)
# g2.addEdge(1, 0)
# g2.addEdge(1, 0)
g2.printEulerTour()

# g1 = Graph(4)
# g1.addEdge(0, 1)
# g1.addEdge(0, 2)
# g1.addEdge(1, 2)
# g1.addEdge(2, 3)
# g1.printEulerTour()
#
# g3 = Graph (5)
# g3.addEdge(1, 0)
# g3.addEdge(0, 2)
# g3.addEdge(2, 1)
# g3.addEdge(0, 3)
# g3.addEdge(3, 4)
# g3.addEdge(3, 2)
# g3.addEdge(3, 1)
# g3.addEdge(2, 4)
# g3.printEulerTour()


#This code is contributed by Neelam Yadav

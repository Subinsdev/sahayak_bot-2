# import cv2

class Graph:
    def __init__(self):
        self.graph = {}
        self.graph_weights = {}
        self.objects_id = {'glue':0, 'battery':1, 'coke':2, 'water_bottle':3, 'glass':4, 'fpga':5}
        self.may_be_present_objects = [[0,0,0,0,0,0], [1,1,0,0,0,0], [1,1,0,0,0,0], [1,1,0,0,0,0], [1,1,0,0,0,0], [1,1,0,0,0,0]]
        self.present_objects = [[0,0,0,0,0,0], [0,0,0,0,0,0], [0,0,0,0,0,0], [0,0,1,1,1,0], [0,0,0,0,0,0], [0,0,0,0,0,1]]
        self.permutations_list = []

    def add_node(self, node_id, weights):
        #self.graph[parent] = childs
        self.graph_weights[node_id] = weights

    def add_detected_objects(self, detected_objects, room_id):
        self.may_be_present_objects[room_id] = [0, 0, 0, 0, 0, 0]
        l = [0]*6
        for object_name in detected_objects:
            l[self.objects_id[object_name]] = 1
        self.present_objects[room_id] = l

    def remove_object(self, object_name, room_id):
        self.present_objects[room_id][self.objects_id[object_name]] = 0

    def check_object_present(self, object_name, room_id):
        if self.present_objects[room_id][self.objects_id[object_name]] == 1:
            return 1
        return 0

    def permutations(self, start, end, l):
        if start == end:
            self.permutations_list.append(l.copy())
            return
        for i in range(start, end+1):
            l[start], l[i] = l[i], l[start]
            self.permutations(start+1, end, l)
            l[start], l[i] = l[i], l[start]
        return

    def find_path(self, current_node, objects_name, room):
        #objects
        #room -> Start=0, Meeting=1, Conference=2, Pantry=3, Research=4, Store=5

        #First find the no. of permuations for the given number of objects and return a list of values
        #Example
        #permuations = [[0,1,2], [0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]]

        #Generating permutaions
        self.permutations_list = []
        l = [int(x) for x in range(0, len(objects_name))]
        self.permutations(0, len(objects_name)-1, l)
        permuations = self.permutations_list

        permuations_length = []
        route = []
        for perm in permuations:
            path_length = 0
            temp_route = []
            curr_room = current_node
            for i in perm:
                object = objects_name[i]
                object_id = self.objects_id[object]
                nearest_room_idx = -1
                nearest_room_distance = 0
                for j in range(6):
                    if self.may_be_present_objects[j][object_id] or self.present_objects[j][object_id]:
                        if nearest_room_idx == -1:
                            nearest_room_idx = j
                            nearest_room_distance = self.graph_weights[curr_room][j]
                        elif nearest_room_distance>self.graph_weights[curr_room][j]:
                            nearest_room_idx = j
                            nearest_room_distance = self.graph_weights[curr_room][j]
                path_length += nearest_room_distance + self.graph_weights[nearest_room_idx][room[i]]
                temp_route.append(nearest_room_idx)
                temp_route.append(room[i])
                curr_room = room[i]
            #Going back to start postion
            path_length += self.graph_weights[curr_room][0]
            temp_route.append(0)
            permuations_length.append(path_length)
            route.append(temp_route)

        #Finding the index of minimum cost permutation
        mini = permuations_length[0]
        min_idx = 0
        for i in range(1, len(permuations_length)):
            if permuations_length[i]<mini:
                min_idx = i
                mini = permuations_length[i]

        return permuations[min_idx], route[min_idx]

def draw_route(image, route, current_room):
    lab_map = image.copy()
    room_locations = [(440, 396), (690, 318), (608, 420), (894, 430), (824, 96), (1246, 486)]
    drop_location = [(0, 0), (660, 314), (644, 416), (0, 0), (782, 108), (0, 0)]
    window_name = 'Image'
    start_point = room_locations[current_room]
    for i in range(len(route)):
        start_point = start_point
        if i%2==0:
        # Green color in BGR
            color = (0, 255, 0)
            end_point = room_locations[route[i]]
        else:
            color = (255, 255, 0)
            end_point = drop_location[route[i]]
        thickness = 2
        lab_map = cv2.arrowedLine(lab_map, start_point, end_point, color, thickness, tipLength = 0.05)
        start_point = end_point

    cv2.imshow("Computed Path", lab_map)
    cv2.waitKey(0)


g = Graph()
#Adding weights from one node to every other node
#location_id -> Start=0, Meeting=1, Conference=2, Pantry=3, Research=4, Store=5
rooms = ["Start", "Meeting", "Conference", "Pantry", "Research", "Store"]
g.add_node(0, [0, 6, 5, 8, 10, 4])
g.add_node(1, [6, 0, 7, 8, 9, 11])
g.add_node(2, [5, 7, 0, 9, 12, 13])
g.add_node(3, [8, 8, 9, 0, 8, 10])
g.add_node(4, [10, 9, 12, 8, 0, 9])
g.add_node(5, [14, 11, 13, 10, 9, 0])

current_room = 0
object_names = ["coke", "fpga", "glue"]
drop_room = [1, 2, 4]       #drop_rooms name based on room_id

room_visited = [0, 0, 0, 0, 0, 0]

while len(object_names)>0:
    result, route = g.find_path(current_room, object_names, drop_room)

    ##Printing the found route
    print("Route: ", rooms[current_room], end="")
    for i in range(len(route)):
        print("->", rooms[route[i]], end=" ")
    print()
    current_room = route[0]

    ##Printing the order of picking the objects
    print("Order of objects getting picked:", object_names[result[0]], end=" ")
    for i in range(1, len(result)):
        print("->", object_names[result[i]], end=" ")
    print()

    print("Going to", rooms[route[0]], "room. Reached!!")
    ##Find all the objects that are present in the room if you are visiting it the first time and save them
    if room_visited[route[0]]==0:
        print("This is the first time you have entered", rooms[route[0]], "room. Scan all the objects that are present in the room.")
        detected_objects = []
        while True:
            object = input("Enter the object name: ")
            if object=="0":
                break
            detected_objects.append(object)
        g.add_detected_objects(detected_objects, route[0])
        room_visited[route[0]] = 1
        continue

    print("You have already visited this room or you are currently in this room for the first time.")
    if g.check_object_present(object_names[result[0]], route[0])==1:
        print(object_names[0], "Object Detected!!")
        print("Picking the ", object_names[0], "Object and going to", rooms[route[1]], "room to drop")
        print("Reached", rooms[route[1]], "Room. Dropped", object_names[result[0]])
        g.remove_object(object_names[result[0]], route[0])
        #Since the object is detected, we will remove the object from the object_names list and drop_room list will be updated
        object_names.pop(result[0])
        drop_room.pop(result[0])
        current_room = route[1]
    else:
        #The object which you are looking for is not there so update the may_be_present_objects list.
        g.may_be_present_objects[route[0]][result[0]] = 0   #0 indicates that the object is not there

    print("------------------------")
    print(object_names)
    print("------------------------")

# image = cv2.imread('lab2.jpg')
# draw_route(image, route, current_room)
# cv2.destroyAllWindows()

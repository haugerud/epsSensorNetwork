#Rajouter une situation où certains nodes ne peuvent plus bouger (drône en panne)
#-> définir une nouvelle liste où l'on met les nodes statiques
#-> ou un paramètre booléen dans la class node pour indiquer si le noeud peut bouger ou non.


#Source for mathematique: https://fr.wikibooks.org/wiki/Math%C3%A9matiques_avec_Python_et_Ruby/Points_en_Python
from math import *
#Source for drawing: https://matplotlib.org/
#https://stackoverflow.com/questions/21519203/plotting-a-list-of-x-y-coordinates-in-python-matplotlib
import numpy.random as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

#obstacle class, define a rectangular area or polygon (python polygon intersection with line)
#http://geoexamples.blogspot.com/2014/08/shortest-distance-to-geometry-in.html
#Install libaries : $ sudo pip3 install Shapely
#                   $ sudo pip3 install descartes
from shapely.geometry import Polygon, Point, LinearRing, LineString
from descartes import PolygonPatch

#library for JSON object
import json

class Node:#Definition of a class Node
    """This class defines a node described by:
    - its coordinate on X axis
    - its coordinate on Y axis
    - its node ID"""


    def __init__(self, x, y, id): 
        self.coord = Point(x,y) # class Point from shapely.geometry
        self.id=id
        self.s=0 # variable to know for how long the node is stable
        self.mobil=True # variable to know if the node is mobile or not (in case of broken node)

    def display(self):#Display the coordinates between () separed by ; after converted them in string
        return 'Node '+str(self.id)+'='+str(self.coord.wkt)

    def middle(self, p):
        return Node((self.coord.x+p.coord.x)/2,(self.coord.y+p.coord.y)/2)

    def vector(self, p):
        return Vector(p.coord.x-self.coord.x , p.coord.y-self.coord.y)

    def distance(self, p):
        return self.vector(p).norm()

    def translation(self, p):
        return Node(self.coord.x+p.x, self.coord.y+p.y, self.id)

class Vector:#Definition of a class Vector
    """This class defines a vector described by:
    - its coordinate on X axis
    - its coordinate on Y axis"""

    def __init__(self, x, y):
        self.x=x
        self.y=y

    def display(self):
        return '('+str(self.x)+';'+str(self.y)+')'

    def norm(self):
        return hypot(self.x, self.y)
    """.hypot(x, y) returns the Euclidean norm, sqrt(x*x + y*y).
    This is the length of the vector from the origin to point (x, y)."""

    def __add__(self, v):#Method to add 2 vectors
        return Vector(self.x+v.x, self.y+v.y)

def VF_sensors(i, j):#Function to calculte the VF exert on a node by a neighboor node
    """This function takes 2 inputs:
    - i: the node on which the force is exerted
    - j: the neighboor node which exerted the force
    It returns a vector Fij_temp"""
    Fij_temp = Vector(0,0)#temporary Vector initialized to zero vector
#    d_ij = i.distance(j)
    d_ij = i.coord.distance(j.coord)
    if Cr >= d_ij and d_ij>d_th:#In this case, Si and Sj are too far and an attractive force is exerted by Sj on Si
        #print("Node {} is too far from node {}, Cr({}) >= d_ij({}) and d_ij > d_th ({}): Attractive force".format(i.id, j.id, Cr, d_ij, d_th))
        Fij_temp.x = (Ka * (d_ij - d_th)) * ((j.coord.x - i.coord.x) / d_ij)
        Fij_temp.y = (Ka * (d_ij - d_th)) * ((j.coord.y - i.coord.y) / d_ij)
    elif d_ij < d_th:#In this case, Si and Sj are too close and a repulsive force is exerted by Sj on Si
        #print("Node {} is too close from node {}, d_ij({}) < d_th ({}): Repulsive force".format(i.id, j.id, d_ij, d_th))
        Fij_temp.x = (Kr * (d_th - d_ij)) * ((i.coord.x - j.coord.x) / d_ij);
        Fij_temp.y = (Kr * (d_th - d_ij)) * ((i.coord.y - j.coord.y) / d_ij);
    #If none of the previous conditions are met, the force vector is still null because no force is exerted on node i.
    return Fij_temp

def VF_obstacles(i ,j):
    """This function takes 2 inputs:
    - i: the node on which the force is exerted
    - j: the obstacle (a polygon) which exerted the force
    It returns a vector Fij_temp"""
    Fiobs_temp = Vector(0,0)
    d_iobs = i.coord.distance(j) # Distance between point Si (i.coord) and Obsj (a polygon)
    if d_iobs < d_thobs and d_iobs>0:#In this case, Si is too close from the obstable and a repulsive force is exerted by the obstable.
#        print("Obstacle detected, d_iobs<d_thobs")
        pol_ext = LinearRing(j.exterior.coords)
        d = pol_ext.project(i.coord)
        closest_point = pol_ext.interpolate(d)
        Fiobs_temp.x = (Kr_obs * (d_thobs - d_iobs)) * ((i.coord.x - closest_point.x) / d_iobs);
        Fiobs_temp.y = (Kr_obs * (d_thobs - d_iobs)) * ((i.coord.y - closest_point.y) / d_iobs);
    #else the obstacle is too far, so no force is exerted on node i and Fiobs_temps = vector zero
    return Fiobs_temp

def Node_translation(i, F, list_o):
    """This function takes 3 inputs:
    - i: the node to move
    - F: the force that moves the node
    - list_o: the obstacle list
    It returns a new node that is the result of the input node translation"""

    temp = i.translation(F)
    dist_init = i.coord.distance(temp.coord)
    g = F.x * 1000
    h = F.y * 1000
    F_temp=Vector(g,h)
    projection = i.translation(F_temp)
    line = LineString([(i.coord.x, i.coord.y),(projection.coord.x, projection.coord.y)])

    dist_min = None
    closest_obs = None
    for elt in list_o:
        difference = line.difference(elt)
        if difference.geom_type == 'MultiLineString': # In this case we meet an obstacle on our way
            dist = list(difference.geoms)[0].length
            if dist_min is None or dist_min > dist:
                dist_min = dist
                closest_obs = elt
    if dist_min != None and dist_min < dist_init: # If dist_min is different from None, that means we meet and osbtacle and we need to reduce the translation.
        #print("CHANGEMENT CAR distance initale {} > dist_min {}".format(dist_init, dist_min))
        ratio = (dist_min * 0.9)/dist_init
        F.x = F.x * ratio
        F.y = F.y * ratio
        temp = i.translation(F)
    #We must also verify that the node doesn't move out of our field of interest.
    inField=True
    x=temp.coord.x
    y=temp.coord.y
    if temp.coord.x < (-xfield)/2:
        x = (-xfield)/2
        inField = False
    elif temp.coord.x > xfield/2:
        x = xfield/2
        inField = False
    if temp.coord.y < (-yfield)/2:
        y = (-yfield)/2
        inField = False
    elif temp.coord.y > yfield/2:
        y = yfield/2
        inField = False
    if inField == False:
        temp=Node(x, y, i.id)

    return temp
# As range() doesn't allow to use decimal, I defined my own function to use decimal
def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step

def Estimate_coverage(list_n):
    """This function takes only 1 input:
    - list_n: the list of nodes 
    It returns the % of covered area"""
    covered=0 # number of covered point
    total=0 # total number of point
    for x in frange(-xfield/2, xfield/2, 0.5):
        for y in frange(-yfield/2, yfield/2, 0.5):
            test=False
            for node in list_n:
                # If the distance between the current point and a node is less than the sensing range the point is covered
                if node.coord.distance(Point(x,y)) < Sr:
                    test=True # Change the value of test
                    break # Now leave the loop, it's useless to verify for the other nodes
                # else, the point is uncovered
            if test == True: # In this case, the point is covered
                covered+=1
            total+=1

    coverage=(covered/total)*100
    return coverage

#Parameters definition 
global Cr #Communication range
global Sr #Sensing range
global L_th # Distance threshold where a node stop to move if its movement is less than this one
global S_th #Time duration after what we consider a node reach its optimal position (use number of iteration, no time units)
global d_th #Distance threshold (= sqrt(3)*Sr)
global Ka #Attraction coefficient
global Kr #Repulsion coefficient
global Kr_obs #Repulsion coefficient for obstacle
global d_thobs #Distance threshold (= sqrt(3)*Sr / 2)
global xfield
global yfield
global xfield_min
global yfield_min
Cr=20
Sr=Cr/2
S_th=10
L_th=0.001
Ka=0.001
Kr=0.3
d_th = sqrt(3)*Sr
Kr_obs = 0.6
d_thobs = d_th / 2
Max_iteration=300
iteration=0
xfield=75
yfield=75


#System definition (field, nodes, obstacles)
#field = Polygon([(xfield/2,yfield/2),(-xfield/2,yfield/2),(xfield/2,-yfield/2),(-xfield/2,-yfield/2)])

list_node=[]
master_node=False
#Initialise each node with the json object
#The first object read will be considered as Master Node
with open("demonstration_input.txt", "r") as file:
    for line in file:
        rdata = json.loads(line)
        if master_node == False:#we need to center the grah on the master node (0,0)
            x_to_center=rdata['la']
            y_to_center=rdata['lo']
            master_node=True
        #Need to convert the data to use them on the map.
        x=(rdata['la']-x_to_center)/100
        y=(rdata['lo']-y_to_center)/100
        temp = Node(x, y, rdata['nr'])
        print(temp.display())
        list_node.append(temp)

x_to_center=59976109
y_to_center=11043916

#n10.mobil=False
poly0 = Polygon([(-3,8),(-7,6),(-5,11)])
poly1 = Polygon([(8,1),(7,4),(10,3)])
poly2 = Polygon([(3,29),(2,32),(5,28)])
poly3 = Polygon([(-7,-24),(-5,-27),(-7,-28)])
poly4 = Polygon([(0,-8),(0,-11),(3,-5)])
poly5 = Polygon([(-13,29),(-10,28),(-14,27)])
poly6 = Polygon([(-24,12),(-22,11),(-25,10)])
poly7 = Polygon([(34,17),(33,19),(34,15)])
poly8 = Polygon([(-27,-34),(-28,-35),(-27,-33)])
poly9 = Polygon([(-27,-18),(-24,-18),(-26,-15)])
poly10 = Polygon([(18,-31),(16,-32),(17,-29)])
poly11 = Polygon([(14,-18),(15,-16),(13,-15)])
poly12 = Polygon([(22,31),(20,32),(23,29)])
poly13 = Polygon([(30,16),(29,14),(28,15)])
poly14 = Polygon([(25,-3),(25,-1),(23,-0)])
poly15 = Polygon([(32,-24),(30,-21),(33,-22)])
#list_node=[n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13]
list_poly=[poly0, poly1, poly2, poly3, poly4, poly5, poly6, poly7, poly8, poly9, poly10, poly11, poly12, poly13, poly14, poly15]

#Plot the initial positions on the graph and legend
#xx, yy = np.meshgrid(np.arange(-25, 26), np.arange(-25,26), sparse=True)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim(-(xfield/2), xfield/2)
ax.set_ylim(-(yfield/2), yfield/2)
"""for elt in list_node:
    plt.scatter(elt.coord.x, elt.coord.y, color="#7f7f7f")"""
#Plot obstacles (polygons)
for elt in list_poly:
    patch2b = PolygonPatch(elt, fc='#ff7f7f', ec='#ff3232', alpha=1., zorder=2)
    ax.add_patch(patch2b)
#Legend display
"""label = "Cr={} | Sr={} | S_th={} | L_th={}\nKr={} | Ka={} | Kr_obs={}".format(Cr, Sr, S_th, L_th, Kr, Ka, Kr_obs)
legend = mpatches.Patch(color='none', label=label)
plt.legend(handles=[legend])"""

#Main loop
while iteration<Max_iteration:
    print("Iteration n°", iteration)
    test=0#Testing variable, reset at each iteration

    for index, i in enumerate(list_node):#For each node in the system
        SumF_node=Vector(0,0)#Reset the force sums at each iteration
        SumF_obs=Vector(0,0)

        if i.s < S_th and i.mobil==True: # If the node isn't stable for a certain amount of time and still mobile, use VF on it.

            for jndex, j in enumerate(list_node):#For each node Sj in the system, calculation of the force it exertes on Si
 
                if index!=jndex:#To avoid to calculate the force exerted by Si on itself.
                    F_node=VF_sensors(i, j)
                    SumF_node=SumF_node.__add__(F_node)

            for obs in list_poly:#For each obstacle Oj in the system, calculation of the force it exertes on Si
                F_obs=VF_obstacles(i, obs)
                SumF_obs=SumF_obs.__add__(F_obs)

            F_tot=SumF_node.__add__(SumF_obs)#Total of the forces exerted on Si (SumF_obs + SumF_node)

            if i.distance(i.translation(F_tot)) < L_th:#Stable ? If the node should move more than a distance treshold: YES
                i.s+=1#Increment the stability timer

            else:#Stable ? NO, so translation
                i.s=0#Reset stability timer
                list_node[index]=Node_translation(list_node[index], F_tot, list_poly)#Move the node to its new position

        elif i.s >= S_th: # The node is stable for more than the time threshold, we don't use VF on it. It already reach its optimal position.
            test+=1 # Increment the testing variable
        else:
            test+=1 # Still incrementing the testing variable

    #Plot every points on a graph
    for elt in list_node:#elt takes the value of each elements in list_node
        plt.scatter(elt.coord.x, elt.coord.y, color="#cbcbcb")

    #Test
    if test==index+1:#If all nodes are stable, the system is optimize so leave the loop 
        break

    iteration+=1

if test==index+1:
    print("Opimized after {} iterations".format(iteration, index))
else:
    print("Non optimized after {} iterations".format(Max_iteration))
print("Area covered at {}% with {} nodes".format(Estimate_coverage(list_node), index))

for elt in list_node:
    print(elt.display())

#Write the new positions in a JSON object
#JSON object format : {"nr":unint32_t,"cn":boolean,"al":boolean,"la":int32_t,"lo":int32_t}
with open("demonstration_output.txt", "w") as wfile:
    for elt in list_node:
#        #Note that is neccessary to double any { or } that are not part of a formatting placeholder.
        #data='{{"nr":{},"cn":true,"al":false,"la":{},"lo":{}}}'.format(elt.id, int(elt.coord.x), int(elt.coord.y))
        x=elt.coord.x*100+x_to_center
        y=elt.coord.y*100+y_to_center
#        data="{'nr':"+str(elt.id)+",'cn':true,'al':false,'la':"+str(int(elt.coord.x))+",'lo':"+str(int(elt.coord.y))+"}"
        data="{'nr':"+str(elt.id)+",'cn':true,'al':false,'la':"+str(int(x))+",'lo':"+str(int(y))+"}"
        jsonlike_data = data.replace("'", "\"")
        wfile.write(jsonlike_data)
        wfile.write("\n")


#Plot the final positions 
for index,elt in enumerate(list_node):#elt takes the value of each elements in list_node
    if elt.mobil == False: # use different color if node is broken
        circ = plt.Circle((elt.coord.x, elt.coord.y), radius=Sr, edgecolor='#FFA500', facecolor="none")#Draw a circle around the point, show the sensing range
        plt.scatter(elt.coord.x, elt.coord.y, color="#FFA500")
        ax.add_patch(circ)
    else:
        circ = plt.Circle((elt.coord.x, elt.coord.y), radius=Sr, edgecolor='b', facecolor="none")#Draw a circle around the point, show the sensing range
        plt.scatter(elt.coord.x, elt.coord.y, color="b")
        ax.add_patch(circ)
plt.show()
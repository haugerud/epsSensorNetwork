#Rajouter une situation où certains nodes ne peuvent plus bouger (drône en panne)
#-> définir une nouvelle liste où l'on met les nodes statiques
#-> ou un paramètre booléen dans la class node pour indiquer si le noeud peut bouger ou non.


#Source for mathematique: https://fr.wikibooks.org/wiki/Math%C3%A9matiques_avec_Python_et_Ruby/Points_en_Python
from math import *
#Source for drawing: https://matplotlib.org/
#https://stackoverflow.com/questions/21519203/plotting-a-list-of-x-y-coordinates-in-python-matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

#obstacle class, define a rectangular area or polygon (python polygon intersection with line)
#http://geoexamples.blogspot.com/2014/08/shortest-distance-to-geometry-in.html

class Node:#Definition of a class Node
    """This class defines a node described by:
    - its coordinate on X axis
    - its coordinate on Y axis
    - its node ID"""


    def __init__(self, x, y, id):#Méthode constructeur, 
        self.x=x
        self.y=y
        self.id=id
        self.s=0#variable to know for how long the node is stable

    def display(self):#Display the coordinates between () separed by ; after converted them in string
        return '('+str(self.x)+';'+str(self.y)+')'

    def middle(self, p):
        return Node((self.x+p.x)/2,(self.y+p.y)/2)

    def vector(self, p):
        return Vector(p.x-self.x,p.y-self.y)

    def distance(self, p):
        return self.vector(p).norm()

    def translation(self, p):
        return Node(self.x+p.x, self.y+p.y, self.id)

#    def distance2(self, p):
#        return sqrt(pow(self.x-p.x, 2) + pow(self.y-p.y, 2))

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
    Its returns a vector Fij_temp"""
    Fij_temp = Vector(0,0)#temporary Vector initialized to zero vector
    d_ij = i.distance(j)
    if Cr >= d_ij and d_ij>d_th:#In this case, Si and Sj are too far and an attractive force is exerted by Sj on Si
        #print("Node {} is too far from node {}, Cr({}) >= d_ij({}) and d_ij > d_th ({}): Attractive force".format(i.id, j.id, Cr, d_ij, d_th))
        Fij_temp.x = (Ka * (d_ij - d_th)) * ((j.x - i.x) / d_ij)
        Fij_temp.y = (Ka * (d_ij - d_th)) * ((j.y - i.y) / d_ij)
    elif d_ij < d_th:#In this case, Si and Sj are too close and a repulsive force is exerted by Sj on Si
        #print("Node {} is too close from node {}, d_ij({}) < d_th ({}): Repulsive force".format(i.id, j.id, d_ij, d_th))
        Fij_temp.x = (Kr * (d_th - d_ij)) * ((i.x - j.x) / d_ij);
        Fij_temp.y = (Kr * (d_th - d_ij)) * ((i.y - j.y) / d_ij);
    #If none of the previous conditions are met, the force vector is still null because no force is exerted on node i.
    return Fij_temp

def VF_obstacles(i ,j):
    Fiobs_temp = Vector(0,0)
    d_iobs = i.distance(j)
    if d_iobs < d_thobs:#In this case, Si is too close from the obstable and a repulsive force is exerted by the obstable.
#        print("Obstacle detected, d_iobs<d_thobs")
        Fiobs_temp.x = (Kr_obs * (d_thobs - d_iobs)) * ((i.x - j.x) / d_iobs);
        Fiobs_temp.y = (Kr_obs * (d_thobs - d_iobs)) * ((i.y - j.y) / d_iobs);
    #else the obstacle is too far, so no force is exerted on node i and Fiobs_temps = vector zero
    return Fiobs_temp

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
Cr=10
Sr=5
S_th=10
L_th=0.001
Ka=0.001
Kr=0.2
d_th = sqrt(3)*Sr
Kr_obs = 0.8
d_thobs = d_th / 2
Max_iteration=200
iteration=0
xfield=50
yfield=50


#System definition (nodes, obstacles)
n0=Node(-1,3,0)
n1=Node(5,1,1)
n2=Node(10,3,2)
n3=Node(-4,-5,3)
n4=Node(7,7,4)
n5=Node(3,0,5)
n6=Node(0,-4,6)
n7=Node(8,-4,7)
n8=Node(2,6,8)
n9=Node(6,-2,9)
n10=Node(10,-5,10)
obs0=Node(1,4,0)
obs1=Node(8,-1,1)
obs2=Node(-3,-5,2)
obs3=Node(-3,-4,3)
list_node=[n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10]
list_obs=[obs0, obs1, obs2, obs3]

#Plot the initial positions on the graph and legend
#xx, yy = np.meshgrid(np.arange(-25, 26), np.arange(-25,26), sparse=True)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim(-(xfield/2), xfield/2)
ax.set_ylim(-(yfield/2), yfield/2)
for elt in list_node:#elt takes the value of each elements in list_node
#    circ = plt.Circle((elt.x, elt.y), radius=5, edgecolor='#bfbfbf', facecolor="none")
    plt.scatter(elt.x, elt.y, color="#7f7f7f")
#    ax.add_patch(circ)
for elt in list_obs: 
    circ = plt.Circle((elt.x, elt.y), radius=d_thobs, edgecolor='#ff7f7f', facecolor="none", linestyle='--')
    plt.scatter(elt.x, elt.y, color="r")
    ax.add_patch(circ)
#Legend display
label = "Cr={} | Sr={} | S_th={} | L_th={}\nKr={} | Ka={} | Kr_obs={}".format(Cr, Sr, S_th, L_th, Kr, Ka, Kr_obs)
legend = mpatches.Patch(color='none', label=label)
plt.legend(handles=[legend])

#Main loop
while iteration<Max_iteration:
    print("Iteration n°", iteration)
    test=0#Testing variable, reset at each iteration

    for index, i in enumerate(list_node):#For each node in the system
        SumF_node=Vector(0,0)#Reset the force sums at each iteration
        SumF_obs=Vector(0,0)
        if i.s < S_th:#If the node isn't stable for a certain amount of time, use VF on it.
            for jndex, j in enumerate(list_node):#For each node Sj in the system, calculation of the force it exertes on Si
                if index!=jndex:#To avoid to calculate the force exerted by Si on itself.
                    #print("Comparaison de list_node[",index,"] and list_node[",jndex,"]")
                    F_node=VF_sensors(i, j)
                    SumF_node=SumF_node.__add__(F_node)
            for obs in list_obs:#For each obstacle Oj in the system, calculation of the force it exertes on Si
                F_obs=VF_obstacles(i, obs)
                SumF_obs=SumF_obs.__add__(F_obs)
            F_tot=SumF_node.__add__(SumF_obs)#Total of the forces exerted on Si (SumF_obs + SumF_node)
            if i.distance(i.translation(F_tot)) < L_th:#Stable ? If the node should move more than a distance treshold: YES
                print("Node {} doesn't move, {}<L_th({}), since i.s={}.".format(i.id, i.distance(i.translation(F_tot)), L_th, i.s))
                i.s+=1#Increment the stability timer
            else:#Stable ? NO
                print("Node {} moves, {}>L_th({}).".format(i.id, i.distance(i.translation(F_tot)), L_th))
                i.s=0#Reset stability timer
                list_node[index]=i.translation(F_tot)#Move the node to its new position
        else:#The node is stable for more than the time threshold, we don't use VF on it. It already reach its optimal position.
            print("Node {} is stable for {} iterations.".format(i.id, i.s))
            test+=1#Increment the testing variable

    #Plot every points on a graph
    for elt in list_node:#elt takes the value of each elements in list_node
        plt.scatter(elt.x, elt.y, color="#cbcbcb")
    print("test =",test,"index =",index)

    #Test
    if test==index+1:#If all nodes are stable, the system is optimize so leave the loop 
        break

    iteration+=1

#Plot the final positions 
for elt in list_node:#elt takes the value of each elements in list_node
    circ = plt.Circle((elt.x, elt.y), radius=Sr, edgecolor='b', facecolor="none")#Draw a circle around the point, show the sensing range
    plt.scatter(elt.x, elt.y, color="b")
    ax.add_patch(circ)
plt.show()
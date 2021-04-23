import numpy as np
from itertools import combinations 
from networkx.drawing.nx_pydot import write_dot
from matplotlib import pyplot as plt
import pydot 
import os
import sys
import turtle
import graphviz
import networkx as nx
from pandas.core.indexes import multi

komsuluk_matrisi = []
with open('komsuluk2.csv') as f:
    for satir in f.readlines():
        komsuluk_matrisi.append( list(map(int, satir.split(','))))
        
#Dosyadan Oku
A = np.array(komsuluk_matrisi)
G1 = nx.from_numpy_array(A,create_using=nx.MultiGraph)

print("***********************")
print(nx.is_eulerian(G1))
print("***********************")

#Bir grafiğin Eulerian olup olmadığını kontrol eder.
def is_eulerian(G1):

    degree_list = [val for (node, val) in G1.degree()]
    for i in degree_list:
        if i % 2 == 1:
            return False
    return True

#euler grafı yapmak için tek ola tepeleri çift dereceli hale getirmeliyiz.
#PYDOT için etiketler
for e in G1.edges():
    G1[e[0]][e[1]][0]['label'] = G1[e[0]][e[1]][0]['weight']
      
#Tek dereceli tepe noktasının listesini döndürür       
def get_odd_degree_list(G1):
    odd_degree_list = []
    for i in G1.degree():
        if i[1] % 2 == 1:
            odd_degree_list.append(i[0])
            print(odd_degree_list)
    return odd_degree_list

#tek dereceli köşelerin tüm olası eşleşmelerinin (kombinasyonları) listesini döndürür
def pairing_vertex(od_list, ret_list, singular_ord, len_od_list):

    i = 0
    while i < len_od_list and od_list[i] == "_":
        i += 1

    if i >= len_od_list:
        ret_list.append([])
        ret_list[-1] = [sublist.copy() for sublist in singular_ord]

    else:
        singular_ord.append([od_list[i]])
        od_list[i] = "_"
        print("od_list",od_list[i])
        print("************************")
        for j in range(i + 1, len_od_list):
            
            if od_list[j] != "_":
                #print("od_list",od_list[i])
                #print("///////////////************************")
                singular_ord[-1].append(od_list[j])
                print("singular_ord",singular_ord)
                od_list[j] = "_"
                pairing_vertex(od_list, ret_list, singular_ord, len_od_list)
                od_list[j] = singular_ord[-1][-1]
                print("singular_ord",singular_ord)
                del singular_ord[-1][-1]
                
        od_list[i] = singular_ord[-1][0]
        print("od_list",od_list[i])
        print("************************")
        del singular_ord[-1]
    print("ret_lis",ret_list)
    return ret_list

def find_shortest_distance(G1):
       
    list_of_rows = komsuluk_matrisi
    g = Graph(len(G1.nodes))
    g.graph = list_of_rows
    distances = g.length_from_source()
    #print(distances)
    return distances



#Her kaynak düğümünün grafikteki diğer düğümlere en kısa yollarını döndürür.
def find_shortest_path(G1):

    list_of_rows = komsuluk_matrisi
    g = Graph(len(G1.nodes))
    g.graph = list_of_rows
    path = g.path()
    return path

#Grafikteki bir düğüm alt çiftini bağlayan ayrıntılı bir yolunu bulan fonk
def previous_vertex(G1, pairings, distance):
    node_ids = find_node_ids(G1)
    shortest_pairing = find_shortest_pairing(G1, pairings, distance)
    path = find_shortest_path(G1)

    previous_vertices = []
    for sub_pair in pairings[shortest_pairing]:
        src = node_ids.get(sub_pair[0])
        des = node_ids.get(sub_pair[1])
        pair_path = path[src]

        src_to_des = []
        src_to_des.append(des)
        while src_to_des[0] != src:
            prev = pair_path[src_to_des[0]]
            src_to_des.insert(0, prev)

        previous_vertices.append(src_to_des)

    for i in range(len(previous_vertices)):
        for j in range(len(previous_vertices[i])):
            for keys in node_ids:
                if node_ids[keys] == previous_vertices[i][j]:
                    previous_vertices[i][j] = keys

    return previous_vertices

#Grafta bulunan düğümlerin idsi
def find_node_ids(G1):
    node_ids = {}
    i = 0
    for node in G1.nodes:
        node_ids[node] = i
        i += 1

    return node_ids


#Her bir alt çifti tek dereceli tepe noktasının kombinasyonuna göre toplam yol uzunluğunu verir
def len_path(pairing, node_ids, distance):
    len = 0
    for sub_pair in pairing:
        src = node_ids.get(sub_pair[0])
        des = node_ids.get(sub_pair[1])
        len += distance[src][des]

    return len


#Alt çiftleri birbirine bağlayan minimum en kısa yolla eşleştirme dizinini döndürür
def find_shortest_pairing(G1, pairings, distance):
    node_ids = find_node_ids(G1)
    ret = 0
    min_len = len_path(pairings[0], node_ids, distance)
    for i in range(1, len(pairings)):
        if len_path(pairings[i], node_ids, distance) < min_len:
            ret = i

    return ret

def get_shortest_walk (G1):

    # euler değilse
    if not is_eulerian(G1):
        # tek dereceli köşelerin listesini alır
        odd_degree_list = get_odd_degree_list(G1)
        #print(len(odd_degree_list))
        # tek dereceli köşelerin kombinasyonlarını alır
        pairings = pairing_vertex(odd_degree_list, [], [], len(odd_degree_list))
        #print("pairings",pairings)
        #print("******************************")
        # Her kaynak düğümden en kısa mesafeyi bulmak için Dijkstra algoritmasını uygular
        distances = find_shortest_distance(G1)
        #alt çiftleri birbirine bağlayan minimum en kısa yolla eşleştirmesini döndür
        shortest_pairing = find_shortest_pairing(G1, pairings, distances)

        # her alt çifti birbirine bağlayan en kısa yolları (kenarların sıralaması) döndürür.
        path = find_shortest_path(G1)
        print("path",path)
        # bulunan kenarları grafiğe ekler
        previous_vertices = previous_vertex(G1, pairings, distances)
        new_edges = []
        for i in range(len(previous_vertices)):
            new_edges = list(zip(previous_vertices[i], previous_vertices[i][1:]))
        for edge in new_edges:
            if edge in G1.edges:
                
                G1.add_edge(edge[0], edge[1],label=G1[e[0]][e[1]][0]['weight'], weight=G1[e[0]][e[1]][0]['weight'])
                #G1[edge[0]][edge[1]][0]['weight'] = G1[edge[0]][edge[1]][0]['weight']


class Graph():

    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
    def minDistance(self, dist, sptSet):

        min = sys.maxsize

        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v

        return min_index

    def dijkstra(self, src):

        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
        path = [0] * self.V
     
        for cout in range(self.V):

            u = self.minDistance(dist, sptSet)
 
            sptSet[u] = True

            for v in range(self.V):
                if self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]:
                    dist[v] = dist[u] + self.graph[u][v]
                    path[v] = u

        return dist, path

    def length_from_source(self):
        distance = []
        for i in range(self.V):
            distance.append(self.dijkstra(i)[0])
           # print("distance", distance)
        return distance
    
    def path(self):
        path = []
        for i in range(self.V):
            path.append(self.dijkstra(i)[1])
        return path
    
get_shortest_walk(G1)
print(nx.is_eulerian(G1))
A = nx.drawing.nx_pydot.to_pydot(G1)
A.write_png("graf.png")
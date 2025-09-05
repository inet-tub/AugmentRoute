import networkx as nx
import numpy as np
import random
import glob

MinN = 10
MaxN = 20
Number_Of_Repeatition_Per_Graph = 1
MinFlowRange = 1
MaxFlowRange = 10
MinWeightRange = 1
MaxWeightRAnge = 100
weightAlterationRatio = 1

#Getting input from network topolgy zoo, the files must be downloaded seperatly and be added to zoo folder
#Can be used for any other graphML inputs
def gettingZooGraphs():
    Zlist = list(glob.glob("./zoo/*.graphml"))
    graphs = []
    for zoo in Zlist:
        temp = nx.read_graphml(zoo)
        final = nx.read_graphml(zoo)
        final.graph['myLabel'] = zoo
        final.remove_nodes_from(list(nx.isolates(final)))
        for edge in temp.edges():
            edgeTemp = list(edge)
            final.add_edge(edgeTemp[1], edgeTemp[0]) #add reverse edge to become undirected   
        graphs.append(final)
    graphs.sort(key=len)
    return graphs

#Choosing just two waypoints
def generateSegmentPath(g,source,terminal):
    oldS = random.sample(g.nodes(), 2)
    nowS = list()
    for wayPoint in oldS:
        if(wayPoint != source and wayPoint != terminal):
            nowS.append(wayPoint)
    tempRet = list()
    previous = source
    for waypoint in nowS:
            tempRet.extend(generateShortestPath(g,previous,waypoint))
            previous = waypoint
    tempRet.extend(generateShortestPath(g,previous,terminal))
    
    correctionGraph = nx.Graph()
    correctionGraph.add_edges_from(tempRet)
  
    return generateShortestPath(correctionGraph,source,terminal)

def generateShortestPath(g,source,terminal):
    path = nx.single_source_dijkstra(g,source,terminal)
    retEdgePath = list()
    for i in range(len(path[1])-1):
        retEdgePath.append(tuple([path[1][i],path[1][i+1]]))
    return retEdgePath

def generatePaths(g,previousPaths,isSoruceTerminal,PathMode, number_of_pairs):
    PathsList = list()
    for k in range(number_of_pairs):
        if(isSoruceTerminal):
            [source, terminal] = [ previousPaths[k][0][0], previousPaths[k][ len(previousPaths[k])-1 ][1]]
        else:
            [source, terminal] = random.sample(list(g.nodes()),2)
        if(PathMode == "Shortest"):
            path = generateShortestPath(g,source,terminal)
        elif (PathMode == "Segment"):
            path = generateSegmentPath(g,source,terminal)
        
        PathsList.append(tuple(path))
    return PathsList

def reversed(edge):
    return tuple([edge[1],edge[0]])

#generating a set of flows first, then assigning capacity based on these flows
def generateCapacity(g,PathMode, number_of_pairs):
    capacity = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    PathsList = generatePaths(g,previousPaths=None,isSoruceTerminal =False,PathMode = PathMode, number_of_pairs=number_of_pairs)
    for path in PathsList:
        tempFlow = random.choice(range(MinFlowRange,MaxFlowRange))  
        for edge in path:
            capacity[edge]+= tempFlow 
            capacity[reversed(edge)]+= tempFlow  #for undirected
    return capacity

def checkAddition(path,flows,temp,capacities):
    for e in path:
       if (flows[e]+temp > capacities[e]):
           return True
    return False 

#Defining new flows
def slowStart(g,oldPaths, updatedPaths, capacities,factor):
    tempFactor = factor-1
    numberOfPaths = len(oldPaths)
    reachedLimit = 0
    oldFlows = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    updatedFlows = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    limit = [0]*numberOfPaths
    mark = [False]*numberOfPaths
    counter = 0
    while (reachedLimit < numberOfPaths):
        for i in range(numberOfPaths):
            if(mark[i] == False):
                temp = 1
                if(limit[i] > 0 ):
                    temp = limit[i]
                    temp *= tempFactor
                mark[i] = ( checkAddition(oldPaths[i],oldFlows,temp,capacities) or checkAddition(updatedPaths[i],updatedFlows,temp,capacities) )
                if(mark[i]== False):
                    limit[i]+=temp
                    for edge in oldPaths[i]:
                        oldFlows[edge]+=temp
                    for edge in updatedPaths[i]:
                        updatedFlows[edge]+=temp
        reachedLimit=0
        for i in range(numberOfPaths):
            if(mark[i] == True):
                reachedLimit+=1
    return limit

def alterWeights(g):
    ret = nx.Graph(g)
    mark = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    for edge in g.edges():
        if( mark[edge] == 0 and random.uniform(0, 1) < weightAlterationRatio):
            tempWeight = random.choice(range(MinWeightRange,MaxWeightRAnge))
            mark[reversed(edge)] = tempWeight
            edgeTemp = list(edge)
            ret.remove_edge(*edge)
            ret.add_edge(edgeTemp[0], edgeTemp[1], weight = tempWeight)
    for edge in g.edges():
        if(mark[edge] > 0):
            edgeTemp = list(edge)
            ret.remove_edge(*edge)
            ret.add_edge(edgeTemp[0], edgeTemp[1], weight = tempWeight)
    return ret


def generateInput(Number_of_pairs,graph,mode,factor,flowPercentage):
    capacities = generateCapacity(graph,PathMode = mode, number_of_pairs=Number_of_pairs)
    updatedWeightsGraph = graph
    if(mode == "Segment" or mode=="Shortest"):
        updatedWeightsGraph = alterWeights(graph)
    oldPaths = generatePaths(g=updatedWeightsGraph,previousPaths=None,isSoruceTerminal =False,PathMode = mode, number_of_pairs=int(Number_of_pairs*flowPercentage))
    if(mode == "Segment" or mode=="Shortest"):
        updatedWeightsGraph = alterWeights(graph)
    updatedPaths = generatePaths(g=updatedWeightsGraph,previousPaths=oldPaths,isSoruceTerminal =True,PathMode = mode, number_of_pairs=int(Number_of_pairs*flowPercentage))
    demands = slowStart(graph,oldPaths,updatedPaths,capacities,factor)
    return[graph,capacities,demands,oldPaths,updatedPaths, Number_of_pairs, mode, factor]

numberOfPairOfFlows = [150]
utilities = [1.1]
methods = ["Segment"]
flowPercentage = 1
MaxNodes = 50
graphs = gettingZooGraphs()

toSave = list()
for numFlow in numberOfPairOfFlows:
    for growthFactor in utilities:
        g = graphs[0]
        for g in graphs:
            if(nx.is_directed(g) and nx.is_weakly_connected(g) and len(g.nodes) <= MaxNodes):
                print(len(g.nodes()))
                toSave.append(generateInput(numFlow,g,"Segment",growthFactor,flowPercentage))
np.save("150Pairs.npy",toSave)
#np.load(path, allow_pickle=True)

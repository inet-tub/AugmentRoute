import networkx as nx
import numpy as np
from numpy.lib.function_base import append

def augmentationLowerBound(method):
    if(method == "Multiplicative"):
        return 1.0
    if(method == "Additive"):
        return 0.0

def isAdditive(method):
    if(method == "Additive"):
        return True
    return False

def isMulti(method):
    if(method == "Multiplicative"):
        return True
    return False
    
def calculateAugemntation(g,previousPaths, currentPaths,demands,capacities,method):
    flows = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    for i in range(0,len(currentPaths)):
        for edge in currentPaths[i]:
            flows[edge]+=demands[i]
        for edge in previousPaths[i]:
            if edge not in currentPaths[i]:
                flows[edge]+=demands[i]
    tempAugmentation = augmentationLowerBound(method)
    for edge in g.edges():
        if(flows[edge] > 0 and capacities[edge] > 0):
            if(isMulti(method)):
                tempAugmentation = max(tempAugmentation,flows[edge]/capacities[edge])
            if(isAdditive(method)):
                tempAugmentation = max(tempAugmentation,flows[edge]-capacities[edge])
    return tempAugmentation

def findBranchNodes(g, oldPath, updatedPath):
    branchNode = dict(zip(g.nodes(), np.zeros(len(g.nodes()))))
    for oldEdge in oldPath:
        for updatedEdge in updatedPath:
            if(updatedEdge[0] == oldEdge[0] and not(updatedEdge[1] == oldEdge[1])):
                branchNode[updatedEdge[0]] = True
    return branchNode

#Identifying backward edges
def naiveStrongPreprocess(g, oldPath, updatedPath):
    markOldNode = dict(zip(g.nodes(), np.zeros(len(g.nodes()))))
    markBranch = findBranchNodes(g, oldPath, updatedPath)
    for j in range(len(oldPath)):
        markOldNode[oldPath[j][0]] = j+1
    lastOld = len(oldPath)-1
    markOldNode[oldPath[lastOld][1]] = len(oldPath)

    lastUpdated = updatedPath[len(updatedPath)-1]
    backward = dict(zip(updatedPath, np.zeros(len(g.nodes()), dtype=bool)))

    for i in range(len(updatedPath)):
        updatedStart = updatedPath[i][0]
        for j in range(len(oldPath)):
            oldStart = oldPath[j][0]
            if(oldStart == updatedStart):
                nowNode = updatedPath[i][1]
                counter = i
                while ( markOldNode[nowNode] == 0 ):
                    counter+=1
                    nowNode = updatedPath[counter][1]
                if(nowNode!=lastUpdated and markOldNode[nowNode] < markOldNode[oldStart]):
                    backward[updatedPath[i]] = True

    markOldEdge = dict(zip(g.edges(), np.zeros(len(g.edges()), dtype=bool)))
    for edge in oldPath:
        markOldEdge[edge] = True

    orderUpdated = dict(zip(updatedPath, np.zeros(len(updatedPath))))
    
    for edge in updatedPath:
        if (not markOldEdge[edge] and markBranch[edge[0]]):
            orderUpdated[edge]+=1
    
    for i in range(len(updatedPath)-2,-1,-1):
        edge = updatedPath[i]
        edgeEnd = edge[1]
        if( backward[edge] == True or orderUpdated[updatedPath[i+1]]> 0 ):
            if(markOldNode[edgeEnd] and (not markOldEdge[edge]) ):
                orderUpdated[edge] = orderUpdated[updatedPath[i+1]]+1
            else:
                orderUpdated[edge] = orderUpdated[updatedPath[i+1]]
    
    return orderUpdated

def strongUpdate(g, oldPath,updatedPath,order,run):
    isAllupdated = True
    correctionGraph = nx.DiGraph()

    for edge in oldPath:
        correctionGraph.add_edge(edge[0], edge[1], version="old")

    for j in range(len(updatedPath)):
        if (order[updatedPath[j]] <= run):
            correctionGraph.add_edge(updatedPath[j][0],updatedPath[j][1], version = "updated")
            isAllupdated = False    
    nowNode = updatedPath[0][0]
    terminal = updatedPath[len(updatedPath)-1][1]
    retPath = list()
    mark = dict(zip(g.nodes(), np.zeros(len(g.nodes()), dtype=bool)))
    while(nowNode != terminal):
        nextEdge = tuple()
        for edge in correctionGraph.edges(nowNode):
            edgeData = correctionGraph.get_edge_data(edge[0], edge[1])
            if(edge[0] == nowNode and mark[edge[1]] == False):
                if( edgeData['version'] == "updated" ):
                    nextEdge = edge
                    break
                else:
                    nextEdge = edge
        retPath.append(nextEdge)
        nowNode = nextEdge[1]
    if(isAllupdated): 
        return updatedPath
    else:
        return retPath

def checkPathUpdated(currentPath, updatedPath):
    if(len(updatedPath) != len(currentPath)):
        return False
    currrentTemp = list(currentPath)
    updatedTemp = list(updatedPath)
    currrentTemp.sort()
    updatedTemp.sort()
    for i in range(len(currrentTemp)):
        if(currrentTemp[i] != updatedTemp[i]):
            return False
    return True

def checkNodes(currentPath,updatedPath):
    if(len(updatedPath) != len(currentPath)):
        return False
    oldTempNodes = list()
    oldTempNodes.append(currentPath[0][0])
    updateTempNodes = list()
    updateTempNodes.append(updatedPath[0][0])
    for i in range(len(currentPath)):
        oldTempNodes.append(currentPath[i][1])
        updateTempNodes.append(updatedPath[i][1])
    oldTempNodes.sort()
    updateTempNodes.sort()
    for i in range(len(oldTempNodes)):
        if(oldTempNodes[i] != updateTempNodes[i]):
            return False
    return True

def Schedule(g,oldPaths, updatedPaths,demands,capacities,isStrong,method):
    runs = 0
    order = list()
    if(isStrong):
        for i in range(0,len(updatedPaths)):
            order.append(naiveStrongPreprocess(g,oldPaths[i],updatedPaths[i]))
    fullyUpdated = 0
    currentPaths = list(oldPaths)
    isFulled = [False]*(len(currentPaths)+10)
    maxbeforeDelay = 0
    allRuns = list()
    sumRuns = 0
    allEqual = True
    while (fullyUpdated < len(updatedPaths)):
        allRuns.append(list(currentPaths))
        previousPaths = list(currentPaths)
        for i in range(0,len(currentPaths)):
            if( isFulled[i] == False ):
                if(isStrong):
                    currentPaths[i] = strongUpdate(g,oldPaths[i],updatedPaths[i],order[i],runs)
                if(checkPathUpdated(currentPaths[i], updatedPaths[i])):
                    fullyUpdated+=1
                    isFulled[i] = True
                    sumRuns+=runs+1
        ScheduleAugmentation = calculateAugemntation(g,previousPaths, currentPaths,demands,capacities,method)
        maxbeforeDelay = max(ScheduleAugmentation,maxbeforeDelay)
        runs+=1
    allRuns.append(list(currentPaths))
    numberOfUpdateds = len(oldPaths)
    averageRuns = sumRuns/numberOfUpdateds
    
    for i in range(0,len(oldPaths)):
        if(not checkNodes(oldPaths[i], updatedPaths[i])):
            allEqual = False
            break
    if(allEqual == False):
        averageRuns+=1
        runs+=1
    return [allRuns,runs,averageRuns,maxbeforeDelay]

def findCongesedtPaths(g,previousPaths, currentPaths,demands,capacities):
    flows = dict(zip(g.edges(), np.zeros(len(g.edges()))))
    markCongested = dict(zip(g.edges(), np.zeros(len(g.edges()), dtype=bool)))
    for i in range(0,len(currentPaths)):
        for edge in currentPaths[i]:
            flows[edge]+=demands[i]
        for edge in previousPaths[i]: #To Be checked
            if edge not in currentPaths[i]:
                flows[edge]+=demands[i]
    for edge in g.edges():
        if(flows[edge] > capacities[edge]):
            markCongested[edge] = True
    congestedList = list()
    for i in range(0,len(currentPaths)):
        for edge in currentPaths[i]:
            if (markCongested[edge] == True):
                congestedList.append(i)
    return congestedList

def greedyOneDelay(g,allRuns,demands, capacities, stepsForward, previousAugmentation, previouslyDelayd,method):
    runs = len(allRuns)
    numberofUpdates = len(allRuns[0])
    CongestedPaths = [False]*numberofUpdates
    for j in range(1,runs):
        tempCongest = findCongesedtPaths(g,allRuns[j-1], allRuns[j],demands,capacities)
        for element in tempCongest:
            CongestedPaths[element] = True
    numberofPaths = len(allRuns[0])
    minAugmentation = previousAugmentation
    retPaths = list(allRuns)
    chosenPath = -1
    for i in range(numberofPaths):
        if(CongestedPaths[i] and not previouslyDelayd[i]):
            maxAfterDelay = 0.0
            for j in range(1,runs):
                tempPaths = list(allRuns[j])
                previousPaths = list(allRuns[j-1])
                if(j >= stepsForward):
                    tempPaths[i] = list(allRuns[j-stepsForward][i])
                else:
                    tempPaths[i] = list(allRuns[0][i])
                if( j >= stepsForward+1):
                    previousPaths[i] = list(allRuns[j-stepsForward-1][i])
                else:
                    previousPaths[i] = list(allRuns[0][i])
                nowAugmention = calculateAugemntation(g,previousPaths, tempPaths,demands,capacities,method)
                maxAfterDelay = max(maxAfterDelay,nowAugmention)
            if(maxAfterDelay < minAugmentation):
                minAugmentation = maxAfterDelay
                chosenPath = i
                #break
    if(chosenPath != -1 ):
            for j in range(1,runs):
                if(j >= stepsForward):
                    retPaths[j][i] = list(allRuns[j-stepsForward][i])
                else:
                    retPaths[j][i] = list(allRuns[0][i])
    return [retPaths,minAugmentation,previouslyDelayd]

def greedyDelayLocalSearch(g,allRuns, demands, capacities, simpleAugmentation, stepsForward, simpleRounds,method):
    previousRounds = simpleRounds
    retRounds = simpleRounds
    nowAugment = simpleAugmentation
    for step in range(1,stepsForward+1):
        nowRuns = list(allRuns)
        previouslyDelayd = [False]*len(allRuns[0])
        [tempRuns,tempAugment,previouslyDelayd] = greedyOneDelay(g,nowRuns, demands, capacities, step, simpleAugmentation,previouslyDelayd,method)
        #print(step)
        while(tempAugment < nowAugment):
        #    print(tempAugment)
            retRounds = previousRounds + step
            nowRuns = tempRuns
            nowAugment = tempAugment
            [tempRuns,tempAugment,previouslyDelayd] = greedyOneDelay(g,nowRuns, demands, capacities, step, simpleAugmentation,previouslyDelayd,method)
    return [tempAugment,retRounds]

def greedySearchMoreDelay(g,allRuns, demands, capacities, simpleAugmentation, simpleRounds,method):
    return greedyDelayLocalSearch(g,allRuns, demands, capacities, simpleAugmentation, 3, simpleRounds,method)



def GreedySolve(g,oldPaths, updatedPaths,demands,capacities, AlgMethod, AugmentMethod):
    output = Schedule(g,oldPaths, updatedPaths,demands,capacities, isStrong = True, method=AugmentMethod)
    allRuns = output[0]
    maxRounds = output[1]
    augmentationNeeded = output[3]
    delayAugment = augmentationNeeded
    delayRounds = maxRounds
    if(AlgMethod == "Delay" and augmentationNeeded > augmentationLowerBound(AugmentMethod)):
        augmentationGreedyOutput = greedySearchMoreDelay(g,allRuns,demands,capacities,augmentationNeeded,maxRounds, AugmentMethod)
        delayAugment = augmentationGreedyOutput[0]
        delayRounds = augmentationGreedyOutput[1]
        return [delayRounds,delayAugment]
    return [maxRounds, augmentationNeeded]
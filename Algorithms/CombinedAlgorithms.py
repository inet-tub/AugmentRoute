import Greedy as gr
import numpy as np
import MIPCombined as mip
import time

AugmentMethods = ["Multiplicative","Additive"]
MIPMethods = ["SameRounds","SameAugment"]
AlgMethods = ["Greedy","Delay"]

def augNeeded(method):
    if(method == "Multiplicative"):
        return 1.0
    if(method == "Additive"):
        return 0.0

def runForAllGraphs(tempName, tempInput):
    toExport = list()
    for value in tempInput:
        [g,capacities,demands,oldPaths,updatedPaths] = value[:5]
        def runningProgram(AugmentMethod, MIPMethod, AlgMethod):
            startTimeAlg = time.time()
            [tempRounds,tempAugmentation] = gr.GreedySolve(g,oldPaths, updatedPaths,demands,capacities, AlgMethod, AugmentMethod) 
            endTimeAlg = time.time()
            elapsedAlgTime = endTimeAlg-startTimeAlg
            if(tempAugmentation > augNeeded(AugmentMethod)):
                startTime = time.time()
                if(MIPMethod == "SameRounds"):
                    mipResult = mip.MIPSolve(g, capacities, demands, oldPaths, updatedPaths,tempRounds,AugmentMethod,MIPMethod)
                if(MIPMethod == "SameAugment"):
                    mipResult = mip.MIPSolve(g, capacities, demands, oldPaths, updatedPaths,tempAugmentation,AugmentMethod,MIPMethod)
                endTime = time.time()
                elapsedMIPTime = endTime-startTime
                capacityList = list()
                for edge in g.edges():
                    capacityList.append(capacities[edge])
                maxCapacities = np.max(capacityList)
                avgCpacaties = np.average(capacityList)
                avgDemand = np.average(demands)
                maxDemand = np.max(demands)
                print(AugmentMethod,AlgMethod,MIPMethod,tempAugmentation,tempRounds,mipResult,elapsedAlgTime,elapsedMIPTime, g.graph['myLabel'], len(g.nodes()), maxCapacities, avgCpacaties, maxDemand, avgDemand, tempName)
                toExport.append([AugmentMethod,AlgMethod,MIPMethod,tempAugmentation,tempRounds,mipResult,elapsedAlgTime,elapsedMIPTime, g.graph['myLabel'], len(g.nodes()), maxCapacities, avgCpacaties, maxDemand, avgDemand,tempName])
        for AugmentMethod in AugmentMethods:
            for MIPmethod in MIPMethods:
                for AlgMethod in AlgMethods:
                        runningProgram(AugmentMethod, MIPmethod,AlgMethod)
    return toExport



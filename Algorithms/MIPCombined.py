import gurobipy as gp
import numpy as np
from gurobipy import GRB
import networkx as nx


class Instance:
    graph = nx.Graph()
    oldPaths = list()
    updatedPaths = list()
    demands = {}
    numberOfNodes = 0
    numberOfRounds = 0
    numberOfUpdates = 0
    capacities = dict()
    alpha = 1
    AugmentationMode = {}
    __nodeToIntMem = dict()
    __edgeToIntMem = dict() 
    model = {}
    solvingMode = {}
    isSameAugment = False
    isSameRound = False
    isAdditive = False
    isMultiplicative = False
    maxAugmentation = {}
    def __init__(self, graph, capacities, demands, oldPaths, updatedPaths, solvingVariable, augmentMethod,MIPMethod):
        self.solvingMode = MIPMethod
        self.AugmentationMode = augmentMethod
        self.capacities = capacities
        self.demands = demands
        self.oldPaths = oldPaths
        self.updatedPaths = updatedPaths
        self.graph = graph
        self.numberOfUpdates = len(oldPaths)
        self.numberOfNodes = len(graph.nodes())
        self.__nodeToIntMem = dict(zip(graph.nodes(), np.arange(len(graph.nodes()))))
        self.__edgeToIntMem = dict(zip(graph.edges(), np.arange(len(graph.edges()))))
        if (self.solvingMode == "SameAugment"):
            self.alpha = solvingVariable
            self.numberOfRounds = (self.numberOfNodes-1)
            self.isSameAugment = True
        if (self.solvingMode == "SameRounds"):
            self.numberOfRounds = solvingVariable
            self.isSameRound = True
        if(self.AugmentationMode == "Multiplicative"):
            self.isMultiplicative = True
            self.maxAugmentation = 2
        if(self.AugmentationMode == "Additive"):
            self.isAdditive = True
            self.maxAugmentation = 100 #TODODODOD
        

    def setModel(self, model):
        self.model = model
    
    def getRoundRange(self, start):
        return range(start,self.numberOfRounds+1)
    
    def nodeToInt(self,node):
        return self.__nodeToIntMem[node]
    
    def edgeToInt(self,edge):
        return self.__edgeToIntMem[edge]

    def getCapacity(self,edge):
        return self.capacities[edge]

    def __findNodePathLabel(self, path):
        ret = list()
        for edge in path:
            ret.append(edge[0])
        ret.append( path[len(path)-1][1] )
        return ret

    def findNodesInPath(self, updateIndex):
        ret =set( self.__findNodePathLabel(self.oldPaths[updateIndex]) + self.__findNodePathLabel(self.updatedPaths[updateIndex])  )
        return ret

    def findEdgesInPath(self, updateIndex):
        return set(self.oldPaths[updateIndex] + self.updatedPaths[updateIndex])

    def construct_model():
        return gp.Model("RerouteMIP")

def nameConstructer(name, variables):
    ret = name + "."
    for v in variables:
        ret += str(v) + "."
    return ret
        

class Variables:
    nodeUpdate = {} #x^r_{i,v}
    joinUpdate = {} #join^r_{i,v}
    forkUpdate = {} #fork^r_{i,v}
    edgeUpdate = {} #y^r_{i,e}
    edgeTransientUpdate = {} #y^{r \lor r-1}_{i,v}
    levelUpdate = {} #l^r_{i,v}
    flowUpdate = {} # f^r_{i,e}
    allowedCongestion = {}
    totalRounds = {}
    def __init__(self, instance):
        if (instance.isSameRound):
            self.allowedCongestion = instance.model.addVar(lb=1.0, ub= instance.maxAugmentation, obj=0.0, vtype = GRB.CONTINUOUS)
        if (instance.isSameAugment):
            self.totalRounds = instance.model.addVar(lb=0.0, ub= instance.numberOfRounds, obj=0.0, vtype = GRB.CONTINUOUS)
        for i in range(instance.numberOfUpdates):
            self.nodeUpdate[i] = {}
            self.joinUpdate[i] = {}
            self.forkUpdate[i] = {}
            self.levelUpdate[i] = {}
            self.edgeUpdate[i] = {}
            self.flowUpdate[i] = {}
            self.edgeTransientUpdate[i] = {} 
            for r in instance.getRoundRange(1):
                self.nodeUpdate[i][r] = {}
                self.joinUpdate[i][r] = {}
                self.forkUpdate[i][r] = {}
                self.levelUpdate[i][r] = {}
                self.edgeUpdate[i][r] = {}
                self.flowUpdate[i][r] = {}
                self.edgeTransientUpdate[i][r] = {} 
                for node in instance.findNodesInPath(i): 
                    v = instance.nodeToInt(node)
                    #Line number 3:
                    self.nodeUpdate[i][r][v] = instance.model.addVar(lb=0, ub=1,vtype=GRB.BINARY, obj=0.0, name = nameConstructer("x",[i,r,v]) )
                    #Line number 13:
                    self.levelUpdate[i][r][v] = instance.model.addVar(lb=0, ub=instance.numberOfNodes-1, obj=0.0, vtype=GRB.CONTINUOUS, name = nameConstructer("l",[i,r,v]) )
                    #Line number ?:
                    self.joinUpdate[i][r][v] = instance.model.addVar(lb=0, ub=1,vtype=GRB.BINARY, obj=0.0, name = nameConstructer("join",[i,r,v]) )
                    #Line number ?:
                    self.forkUpdate[i][r][v] = instance.model.addVar(lb=0, ub=1,vtype=GRB.BINARY, obj=0.0, name = nameConstructer("fork",[i,r,v]) )

                for edge in instance.findEdgesInPath(i):
                    e = instance.edgeToInt(edge)
                    #Line number 6:
                    self.edgeUpdate[i][r][e] = instance.model.addVar(lb=0, ub=1, vtype=GRB.BINARY,obj=0.0, name = nameConstructer("y",[i,r,(edge[0],edge[1])]) )
                    #Line number 15:
                    self.flowUpdate[i][r][e] = instance.model.addVar(lb=0, ub=1, vtype=GRB.CONTINUOUS,obj=0.0, name = nameConstructer("flow",[i,r,(edge[0],edge[1])]) )
                    #Line number ?
                    self.edgeTransientUpdate[i][r][e] = instance.model.addVar(lb=0, ub=1, vtype=GRB.CONTINUOUS,obj=0.0, name = nameConstructer("t",[i,r,(edge[0],edge[1])]) )
            #For line number 7 & 8:
            self.edgeUpdate[i][0] = {}
            for edge in instance.findEdgesInPath(i):
                e = instance.edgeToInt(edge)
                self.edgeUpdate[i][0][e] = instance.model.addVar(lb=0, ub=1, vtype=GRB.BINARY,obj=0.0, name = nameConstructer("e",[i,0,e]) )


def construct_constraints(instance,variables):
    for i in range(instance.numberOfUpdates):
        
        tempOldPath = instance.oldPaths[i]
        tempUpdatedPath = instance.updatedPaths[i]
        #Line number 5 : Each node at least needs to be updated once
        for node in instance.findNodesInPath(i):
            at_least_updated_once = gp.LinExpr()
            v = instance.nodeToInt(node)
            for r in instance.getRoundRange(1):
                at_least_updated_once.addTerms(1,variables.nodeUpdate[i][r][v])
            instance.model.addLConstr(at_least_updated_once, GRB.EQUAL, 1, name=nameConstructer("atLeastUpdatedOnce",[i,v]))
       
        #Line number 7: Old edges are active at the beigining
        for edge in tempOldPath:
            e = instance.edgeToInt(edge)
            instance.model.addLConstr(variables.edgeUpdate[i][0][e], GRB.EQUAL, 1, name=nameConstructer("OldAtTheBegining",[i,e]) )
        #Line number 8: other edges are not active at the begining
        for edge in tempUpdatedPath:
                if(edge not in tempOldPath):
                    e = instance.edgeToInt(edge)
                    instance.model.addLConstr(variables.edgeUpdate[i][0][e], GRB.EQUAL, 0, name=nameConstructer("UpdatedAtTheBegining",[i,e]))
        
        for r in instance.getRoundRange(1):
            #Line number 3: Number of rounds is depdendent on the last round with node update
            for node in instance.findNodesInPath(i):
                rounds_used = gp.LinExpr()
                v = instance.nodeToInt(node)
                rounds_used.addTerms(-1*r,variables.nodeUpdate[i][r][v])
                if(instance.isSameRound):
                    instance.model.addLConstr(rounds_used, GRB.LESS_EQUAL, instance.numberOfRounds, name=nameConstructer("RoundThershold",[i,r,v]))
                if(instance.isSameAugment):
                    rounds_used.addTerms(1, variables.totalRounds)
                    instance.model.addLConstr(rounds_used, GRB.GREATER_EQUAL, 0, name=nameConstructer("RoundThershold",[i,r,v]))

            #Line number 9: when updated edges become active
            for edge in tempUpdatedPath:
                updated_Noupdate_until_round = gp.LinExpr()
                e = instance.edgeToInt(edge)
                if(edge not in tempOldPath):
                    u = instance.nodeToInt(edge[0])
                    for prime in range(1,r+1):
                        updated_Noupdate_until_round.addTerms(-1, variables.nodeUpdate[i][prime][u])
                    updated_Noupdate_until_round.addTerms(1,variables.edgeUpdate[i][r][e])
                    instance.model.addLConstr(updated_Noupdate_until_round, GRB.EQUAL, 0)

            # for edge in graph:
            #     for i in range(number of nodes)
            #             for j in range(number of nodes)
            #                 temp = gp.LinExpr()
            #                 temp.addTerms((i-j),x[edge(0)][i])
            #                 temp.addTerms((i-j), x[edge(1)][j])
            #                 temp.addTerms(-1,1)
            #                 instance.model.addLConstr(temp,GRB.LESS_EQUAL,dist_varaible[edge])
            # sum = gp.LinExpr()
            # for edge in graph:
            #     sum.addTerms(1,dist_varaible[edge])
            # instance.model.addLConstr(sum, GRB.EQUAL, dist)

            #Line number 10: When old edges become active
            for edge in tempOldPath:
                old_update_until_round = gp.LinExpr()
                e = instance.edgeToInt(edge)
                if(edge not in tempUpdatedPath):
                    u = instance.nodeToInt(edge[0])
                    for prime in range(1,r+1):
                        old_update_until_round.addTerms(1, variables.nodeUpdate[i][prime][u])
                    old_update_until_round.addTerms(1, variables.edgeUpdate[i][r][e])
                    instance.model.addLConstr(old_update_until_round, GRB.EQUAL, 1)
                #Line number : Edges that are both in updated and old path are always active
                else:
                    instance.model.addLConstr(variables.edgeUpdate[i][r][e], GRB.EQUAL, 1 )         
            
            forkMark = dict(zip(instance.findNodesInPath(i), np.zeros(len(instance.findNodesInPath(i)), dtype=bool)))
            joinMark = dict(zip(instance.findNodesInPath(i), np.zeros(len(instance.findNodesInPath(i)), dtype=bool)))

            for oldEdge in tempOldPath:
                for updatedEdge in tempUpdatedPath:
                    #Line number ? : Fork node if it was fork
                    if(updatedEdge[0] == oldEdge[0] and not(updatedEdge[1] == oldEdge[1])):
                        u = instance.nodeToInt(updatedEdge[0])
                        fork_constr = gp.LinExpr()
                        fork_constr.addTerms(1, variables.forkUpdate[i][r][u])
                        fork_constr.addTerms(-1, variables.nodeUpdate[i][r][u])
                        instance.model.addLConstr(fork_constr, GRB.EQUAL, 0)
                        forkMark[updatedEdge[0]] = True
                     
                     #Line number ?: Join nodes
                    if(updatedEdge[1] == oldEdge[1] and not(updatedEdge[0] == oldEdge[0])):
                        v = instance.nodeToInt(oldEdge[1])
                        e = instance.edgeToInt(oldEdge)
                        join_constr_1 = gp.LinExpr()
                        join_constr_1.addTerms(1,variables.joinUpdate[i][r][v])
                        join_constr_1.addTerms(-1, variables.flowUpdate[i][r][e])
                        instance.model.addLConstr(join_constr_1, GRB.LESS_EQUAL, 0)
                        e = instance.edgeToInt(updatedEdge)
                        join_constr_2 = gp.LinExpr()
                        join_constr_2.addTerms(1,variables.joinUpdate[i][r][v])
                        join_constr_2.addTerms(-1, variables.flowUpdate[i][r][e])
                        instance.model.addLConstr(join_constr_2, GRB.LESS_EQUAL, 0)
                        joinMark[oldEdge[1]] = True
            

            #Line number ? : Not fork if it is not
            for node in instance.findNodesInPath(i):
                if (forkMark[node] == False):
                    fork_constr_2 = gp.LinExpr()
                    u = instance.nodeToInt(node)
                    fork_constr_2.addTerms(1, variables.forkUpdate[i][r][u])
                    instance.model.addLConstr(fork_constr_2, GRB.EQUAL, 0) 
                if (joinMark[node] == False):
                    join_constr_3 = gp.LinExpr()
                    u = instance.nodeToInt(node)
                    join_constr_3.addTerms(1, variables.joinUpdate[i][r][u])
                    instance.model.addLConstr(join_constr_3, GRB.EQUAL, 0)      
            
            for edge in instance.findEdgesInPath(i):

                e = instance.edgeToInt(edge)
                tail_edge = edge[0]
                u = instance.nodeToInt(tail_edge)
                head_edge = edge[1] 
                v = instance.nodeToInt(head_edge)

                
                #Line number 11
                transiant_1 = gp.LinExpr()
                transiant_1.addTerms(+1, variables.edgeTransientUpdate[i][r][e]) 
                transiant_1.addTerms(-1,variables.edgeUpdate[i][r-1][e])
                instance.model.addLConstr(transiant_1 , GRB.GREATER_EQUAL, 0)
                
                #Line number 12
                transiant_2 = gp.LinExpr()
                transiant_2.addTerms(+1, variables.edgeTransientUpdate[i][r][e]) 
                transiant_2.addTerms(-1,variables.edgeUpdate[i][r][e])
                instance.model.addLConstr(transiant_2 , GRB.GREATER_EQUAL, 0)    
                
                #Line number 14
                leveling = gp.LinExpr()

                leveling.addTerms(instance.numberOfNodes-1, variables.edgeTransientUpdate[i][r][e]) 
                leveling.addTerms(+1,variables.levelUpdate[i][r][u])
                leveling.addTerms(-1,variables.levelUpdate[i][r][v])
                instance.model.addLConstr(leveling, GRB.LESS_EQUAL, instance.numberOfNodes-2)

                #Line number ? : Flow update r
                flow_vs_edge_1 = gp.LinExpr()
                flow_vs_edge_1.addTerms(1,variables.flowUpdate[i][r][e])     
                flow_vs_edge_1.addTerms(-1,variables.edgeUpdate[i][r][e])
                flow_vs_edge_1.addTerms(-1, variables.forkUpdate[i][r][u])
                instance.model.addLConstr(flow_vs_edge_1, GRB.LESS_EQUAL, 0)
                
                #Line number ? : Flow update r-1
                flow_vs_edge_2 = gp.LinExpr()
                flow_vs_edge_2.addTerms(1,variables.flowUpdate[i][r][e])     
                flow_vs_edge_2.addTerms(-1,variables.edgeUpdate[i][r-1][e])
                flow_vs_edge_2.addTerms(-1, variables.forkUpdate[i][r][u])
                instance.model.addLConstr(flow_vs_edge_2, GRB.LESS_EQUAL, 0)

            #Line number ?? level start
            s = instance.nodeToInt(tempOldPath[0][0])
            instance.model.addLConstr(variables.levelUpdate[i][r][s],GRB.EQUAL,0)


            source_node = tempOldPath[0][0]
            terminal_node = tempOldPath[len(tempOldPath)-1][1]
            nowSource = instance.nodeToInt(source_node)
            nowTerminal = instance.nodeToInt(terminal_node)

            #Line number 16: Source flow update
            source_outPut_flow = gp.LinExpr()
            neighSource = list([tempOldPath[0], tempUpdatedPath[0]])
            neighSource = set(neighSource)
            for edge in neighSource:
                e = instance.edgeToInt(edge)
                source_outPut_flow.addTerms(1, variables.flowUpdate[i][r][e])
            source_outPut_flow.addTerms(-1, variables.forkUpdate[i][r][nowSource])
            instance.model.addLConstr(source_outPut_flow, GRB.EQUAL, 1)

            #Line number 17: Termninal flow update
            neighTerminal = set(list([tempOldPath[len(tempOldPath)-1] , tempUpdatedPath[len(tempUpdatedPath)-1]]))
            terminal_inPut_flow = gp.LinExpr()
            for edge in neighTerminal:
                e = instance.edgeToInt(edge)
                terminal_inPut_flow.addTerms(1, variables.flowUpdate[i][r][e])
            terminal_inPut_flow.addTerms(-1, variables.joinUpdate[i][r][nowTerminal])
            instance.model.addLConstr(terminal_inPut_flow, GRB.EQUAL, 1)

            #Line number 18
            for node in instance.findNodesInPath(i): 
                if((node != source_node) and (node != terminal_node)):
                    middle_flow = gp.LinExpr()
                    nowNode = instance.nodeToInt(node)
                    for edge in instance.findEdgesInPath(i):
                        e = instance.edgeToInt(edge)
                        if (edge[0] == node):
                            middle_flow.addTerms(1,variables.flowUpdate[i][r][e])
                        if (edge[1] == node):
                            middle_flow.addTerms(-1,variables.flowUpdate[i][r][e])
                    middle_flow.addTerms(-1, variables.forkUpdate[i][r][nowNode])
                    middle_flow.addTerms(1, variables.joinUpdate[i][r][nowNode])
                    instance.model.addLConstr(middle_flow, GRB.EQUAL, 0)

    #Line number 20
    for r in instance.getRoundRange(1):
        for edge in instance.graph.edges():
            capacity_constraint = gp.LinExpr()
            e = instance.edgeToInt(edge)
            for i in range(instance.numberOfUpdates):
                if(edge in instance.findEdgesInPath(i)):
                    capacity_constraint.addTerms(instance.demands[i] , variables.flowUpdate[i][r][e])
            if(instance.isSameRound):
                if(instance.isMultiplicative):
                    capacity_constraint.addTerms(-1*instance.getCapacity(edge), variables.allowedCongestion)
                    instance.model.addLConstr( capacity_constraint , GRB.LESS_EQUAL, 0 )
                if(instance.isAdditive):
                    capacity_constraint.addTerms(-1, variables.allowedCongestion)
                    instance.model.addLConstr( capacity_constraint , GRB.LESS_EQUAL, instance.getCapacity(edge) )
            if(instance.isSameAugment):
                if(instance.isMultiplicative):
                    instance.model.addLConstr( capacity_constraint , GRB.LESS_EQUAL, instance.alpha * instance.getCapacity(edge))
                if(instance.isAdditive):
                    instance.model.addLConstr( capacity_constraint , GRB.LESS_EQUAL, instance.alpha + instance.getCapacity(edge))

def construct_objectives(instance, variables):
    if(instance.isSameRound):
        instance.model.setObjective(variables.allowedCongestion, GRB.MINIMIZE)
    if(instance.isSameAugment):
        instance.model.setObjective(variables.totalRounds, GRB.MINIMIZE)


toExport = list()

def MIPSolve(nowGraph, capacities, demands, oldPaths, updatedPaths, variable, augmentMethod,MIPMethod):
    instance = Instance(nowGraph,capacities, demands, oldPaths, updatedPaths, variable, augmentMethod,MIPMethod)
    instance.setModel(gp.Model("RerouteMIP "+ nowGraph.graph['myLabel'] ))
    varaibles = Variables(instance)
    construct_constraints(instance,varaibles)
    construct_objectives(instance,varaibles)
    instance.model.setParam('OutputFlag', False)   #Quite Opetimzation
    instance.model.optimize()
    if (instance.model.SolCount > 0):
        return instance.model.ObjVal
    else:
         return -1


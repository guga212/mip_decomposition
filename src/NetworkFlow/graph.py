import networkx as nx
import numpy as np
import random as rnd
import numpy.random as nrnd



class NetworkGraph:
    def __init__(self, graph):
        self.network_graph = graph
        self.flow_list = []
        self.src_dst_list = []
        self.path_list = []
        self.capacity_min = 0
        self.capacity_max = 0        

    def SetRandomSeed(seed):
        rnd.seed(seed)
        nrnd.seed(seed)

    def SetSrcDst(self, *args):
        """Add points of interset (source, destination). """

        self.src_dst_list = list(args)

        indx_del = 0
        while True:
            flow_src_attr_prev = nx.get_node_attributes(self.network_graph, f'flow_src_{indx_del}')
            flow_dst_attr_prev = nx.get_node_attributes(self.network_graph, f'flow_dst_{indx_del}')
            nmb_flow_src_attr_prev = len(flow_src_attr_prev)
            nmb_flow_dst_attr_prev = len(flow_dst_attr_prev)
            if nmb_flow_src_attr_prev > 0:
                for k in flow_src_attr_prev:
                    nx.edges(self.network_graph)[k].pop(f'flow_src_{indx_del}')
            if nmb_flow_dst_attr_prev > 0:
                for k in flow_dst_attr_prev:
                    nx.edges(self.network_graph)[k].pop(f'flow_dst_{indx_del}')
            if (nmb_flow_src_attr_prev + nmb_flow_dst_attr_prev) == 0:
                break
            indx_del += 1
        
        nodes_attr = {}
        for indx, node_pair in enumerate(self.src_dst_list):
            if node_pair[0] in nodes_attr:
                nodes_attr[node_pair[0]].update( { f'flow_src_{indx}': f'src_node_{indx}' } )
            else:
                nodes_attr[node_pair[0]] = { f'flow_src_{indx}': f'src_node_{indx}' }
            if node_pair[1] in nodes_attr:
                nodes_attr[node_pair[1]].update( { f'flow_dst_{indx}': f'dst_node_{indx}' } )
            else:
                nodes_attr[node_pair[1]] = { f'flow_dst_{indx}': f'dst_node_{indx}' }
        nx.set_node_attributes(self.network_graph, nodes_attr)
        self.flow_list.extend([ 0 for i in range(len(self.src_dst_list))])

    def SetPath(self, *args):
        """Add the pathes (arcs belongin to the same route). """

        path_attr_prev = nx.get_edge_attributes(self.network_graph, 'path')
        if len(path_attr_prev) > 0:
            for k in path_attr_prev:
                nx.edges(self.network_graph)[k].pop('path')

        self.path_list = list(args)
        for path_nmb, path in enumerate(self.path_list):
            nx.set_edge_attributes(self.network_graph, {edge : { 'path' : f'path_{path_nmb}'} for edge in path } )

    def SetFlows(self, *args):
        """Assign to flow values given arguments"""

        self.flow_list = list(args)
        if len(self.flow_list) != len(self.src_dst_list):
            raise Exception("Flows number must be equal to src dst number")
    
    def GenerateRandomSrcDst(self, pair_nmb):
        """Create specified number of the random tuples: (source, destination). """

        nodes_nmb = nx.number_of_nodes(self.network_graph)
        nodes_array = np.arange(0, nodes_nmb)
        #path number cant be greater than number of arc excluding self loops
        pair_nmb = np.amin([pair_nmb, nodes_nmb*nodes_nmb - nodes_nmb])
        src_dst_list = pair_nmb*[None]
        
        #iterate shuffle nodes array and select pair of nodes from it
        for i in range(len(src_dst_list)):
            
            src_dest_cand = None
            src_dst_list[i] = src_dest_cand 

            while src_dest_cand in src_dst_list:
                nrnd.shuffle(nodes_array)
                src_node = nodes_array[0]
                dst_node = nodes_array[1]
                src_dest_cand  = (src_node, dst_node)
            src_dst_list[i] = src_dest_cand 

            #add arc to the graph if it doesn't exist
            if not nx.has_path(self.network_graph, src_node, dst_node):
                nx.add_path(self.network_graph, src_dst_list[i], capacity = np.around((self.capacity_max - self.capacity_min)*nrnd.ranf()+self.capacity_min, 3))        
        self.SetSrcDst(*src_dst_list)

    def GetNodeList(self):
        """Return list of nodes"""

        nodes_nmb = nx.number_of_nodes(self.network_graph)
        return list(range(0, nodes_nmb))

    def GetArcList(self):        
        """Return graph's list of the connected nodes"""
    
        return [ arc for arc in nx.edges(self.network_graph) ]

    def GetCapacityParam(self):
        """Return dictionary, representing arcs available capacity"""

        return nx.get_edge_attributes(self.network_graph, 'capacity')

    def GetFlowList(self):
        """Return list of flows"""

        return self.flow_list

    def GetSrcDstDict(self):
        """Return dictionary of the source and destination for every flow. """

        src_dst_dict = { k:v for k,v in enumerate(self.src_dst_list)}
        return src_dst_dict

    @classmethod
    def GenerateFull(cls, nodes_nmb, capacity=0):
        """
        Create graph with specified nodes number, where all nodes are connected to the all nodes. 
        Does not contain self loop.
        Set the same capacity to all arcs in the graph.
        """

        network_graph = nx.complete_graph(nodes_nmb, nx.DiGraph())
        nx.set_edge_attributes(network_graph, capacity, name='capacity')
        self = cls(network_graph)
        self.capacity_min = capacity
        self.capacity_max = capacity
        return self

    @classmethod
    def GenerateRandom(cls, nodes_nmb, edges_nmb, capacity_min=0, capacity_max=0):
        """
        Create graph with specified number of nodes and arcs, where random nodes are connected to random nodes.
        Does not contain self loop.
        Set to every arc in the graph random value of the capacity from min to max.
        """

        network_graph = nx.gnm_random_graph(nodes_nmb, edges_nmb, directed=True)
        
        nx.set_edge_attributes(network_graph, {edge: np.around((capacity_max - capacity_min)*nrnd.ranf()+capacity_min, 3) for edge in nx.edges(network_graph)}, name='capacity')
        self = cls(network_graph)
        self.capacity_min = capacity_min
        self.capacity_max = capacity_max
        return self
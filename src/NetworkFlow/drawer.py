import math
import networkx as nx
import matplotlib.pyplot as plt


class NetworkDrawer:
    
    def __init__(self, network):
        self.width_mul = 3
        #create figure, subplots and layout
        flow_nmb = len(network.GetFlowList())
        ncol = max(min(2, flow_nmb), 1)
        nrow = math.ceil((flow_nmb / ncol))
        self.fig, self.axes = plt.subplots(nrow, ncol)
        plt.figure(self.fig.number)
        pos = nx.spring_layout(network.network_graph)
        
        #draw graph for every plot
        def DrawGraph(flow_indx, flow_val):
            #draw source node
            src_nodes = [node for indx, node in enumerate(nx.nodes(network.network_graph)) 
                            if 'src' in nx.nodes(network.network_graph)[indx] and 
                            nx.nodes(network.network_graph)[indx]['src'] == f'src_node_{flow_indx}']
            nx.draw_networkx_nodes(network.network_graph, pos, nodelist=src_nodes, node_color='g')        
            #draw destination node
            dst_nodes = [node for indx, node in enumerate(nx.nodes(network.network_graph)) 
                            if 'dst' in nx.nodes(network.network_graph)[indx] and 
                            nx.nodes(network.network_graph)[indx]['dst'] == f'dst_node_{flow_indx}']
            nx.draw_networkx_nodes(network.network_graph, pos, nodelist=dst_nodes, node_color='y')
            #draw nodes labels
            src_labels = {key : label for key, label in nx.get_node_attributes(network.network_graph, 'src').items() 
                            if label == f'src_node_{flow_indx}'}
            nx.draw_networkx_labels(network.network_graph, pos, labels=src_labels, font_color='y')
            dst_labels = {key : label for key, label in nx.get_node_attributes(network.network_graph, 'dst').items() 
                            if label == f'dst_node_{flow_indx}'}
            nx.draw_networkx_labels(network.network_graph, pos, labels=dst_labels, font_color='g')
            #draw remaining node
            rm_nodes = [node for node in nx.nodes(network.network_graph) if node not in src_nodes and node not in dst_nodes]
            nx.draw_networkx_nodes(network.network_graph, pos, nodelist=rm_nodes, node_color='#aaaaaa')

            #draw all edges            
            capacity_list = [capacity for edge, capacity in nx.get_edge_attributes(network.network_graph, 'capacity').items()]
            width_list = [1 + self.width_mul*c/network.capacity_max for c in capacity_list]
            nx.draw_networkx_edges(network.network_graph, pos, width=width_list )
            #draw edges in path
            path_edges = [edge for edge, path in nx.get_edge_attributes(network.network_graph, 'path').items()
                            if path == f'path_{flow_indx}']
            width_path = 1
            if(flow_val != 0):
                width_path = 1 + self.width_mul*flow_val/network.capacity_max
            nx.draw_networkx_edges(network.network_graph, pos, edgelist=path_edges, edge_color='r', width=width_path)
            
        #draw graph for every flow
        for flow_indx, flow_val in enumerate(network.GetFlowList()):
            if(flow_nmb == 1):
                plt.axes(self.axes)
                DrawGraph(flow)
                break
            if(flow_nmb > 1 and nrow == 1):
                plt.axes(self.axes[flow])
                DrawGraph(flow)
            if(flow_nmb > 1 and nrow > 1):
                plt.axes(self.axes[flow_indx // ncol][flow_indx % ncol])
                DrawGraph(flow_indx, flow_val)
        #draw single graph for no flows
        if(flow_nmb == 0):
            nx.draw_networkx_nodes(network.network_graph, pos, node_color='#aaaaaa')            
            capacity_list = [capacity for edge, capacity in nx.get_edge_attributes(network.network_graph, 'capacity').items()]
            width_list = [1 + self.width_mul*c/network.capacity_max for c in capacity_list]
            nx.draw_networkx_edges(network.network_graph, pos, width=width_list )


    def Show(self):
        plt.show(block=True)

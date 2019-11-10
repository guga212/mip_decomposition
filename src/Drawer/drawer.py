import math
import networkx as nx
import matplotlib.pyplot as plt


def PlotIterationData(name, *data_lists):
    fig = plt.figure()
    fig.suptitle(name)
    fig.canvas.set_window_title(name)
    for dl in data_lists:
        plt.plot(dl, linestyle='-')

def PlotNetwork(network, name):
    width_mul = 3
    #create figure, subplots and layout
    flow_nmb = len(network.GetFlowList())
    ncol = max(min(2, flow_nmb), 1)
    nrow = math.ceil((flow_nmb / ncol))
    fig, axes = plt.subplots(nrow, ncol)
    plt.figure(fig.number)
    fig.canvas.set_window_title(name)
    fig.suptitle(name)
    pos = nx.shell_layout(network.network_graph)
    
    #draw graph on the multiple plot
    def DrawGraph(flow_indx, flow_val):
        #draw source node
        src_nodes = [node for indx, node in enumerate(nx.nodes(network.network_graph)) 
                        if f'flow_src_{flow_indx}' in nx.nodes(network.network_graph)[indx]]
        nx.draw_networkx_nodes(network.network_graph, pos, nodelist=src_nodes, node_color='g')        
        #draw destination node
        dst_nodes = [node for indx, node in enumerate(nx.nodes(network.network_graph)) 
                        if f'flow_dst_{flow_indx}' in nx.nodes(network.network_graph)[indx]]
        nx.draw_networkx_nodes(network.network_graph, pos, nodelist=dst_nodes, node_color='r')
        #draw nodes labels
        node_labels = {node : node for node in nx.nodes(network.network_graph)}
        nx.draw_networkx_labels(network.network_graph, pos, labels=node_labels, font_color='black')
        #draw remaining node
        rm_nodes = [node for node in nx.nodes(network.network_graph) if node not in src_nodes and node not in dst_nodes]
        nx.draw_networkx_nodes(network.network_graph, pos, nodelist=rm_nodes, node_color='#aaaaaa')
        #draw all edges            
        capacity_list = [capacity for edge, capacity in nx.get_edge_attributes(network.network_graph, 'capacity').items()]
        width_list = [1 + width_mul*c/network.capacity_max for c in capacity_list]
        nx.draw_networkx_edges(network.network_graph, pos, width=width_list )
        #draw edges in path
        path_edges = [edge for edge, path in nx.get_edge_attributes(network.network_graph, f'path_{flow_indx}').items()]
        
        width_path = 1
        if(flow_val != 0):
            width_path = 1 + width_mul*flow_val/network.capacity_max
        nx.draw_networkx_edges(network.network_graph, pos, edgelist=path_edges, edge_color='y', width=width_path)
        
    #draw graph for every flow
    for flow_indx, flow_val in enumerate(network.GetFlowList()):
        if(flow_nmb == 1):
            plt.axes(axes)
            DrawGraph(flow_indx, flow_val)
            break
        if(flow_nmb > 1 and nrow == 1):
            plt.axes(axes[flow_indx])
            DrawGraph(flow_indx, flow_val)
        if(flow_nmb > 1 and nrow > 1):
            plt.axes(axes[flow_indx // ncol][flow_indx % ncol])
            DrawGraph(flow_indx, flow_val)
    #draw single graph for no flows
    if(flow_nmb == 0):
        nx.draw_networkx_nodes(network.network_graph, pos, node_color='#aaaaaa')                
        node_labels = {node : node for node in nx.nodes(network.network_graph)}
        nx.draw_networkx_labels(network.network_graph, pos, labels=node_labels, font_color='black')       
        capacity_list = [capacity for edge, capacity in nx.get_edge_attributes(network.network_graph, 'capacity').items()]
        width_list = [1 + width_mul*c/network.capacity_max for c in capacity_list]
        nx.draw_networkx_edges(network.network_graph, pos, width=width_list )
    
    #show drawed plot
    plt.show(block = False)
    
def ShowAll():
    plt.show(block = True)

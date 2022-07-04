import matplotlib.pyplot as plt
import pandas as pd
from scipy.constants import c
from scipy.special import erfcinv
import json
import numpy as np
from Node import Node
from Line import Line
from Lightpath import Lightpath
from Connection import Connection


class Network(object):
    def __init__(self, json_path, nch = 10 , upgrade_line =''):
        node_json = json.load(open(json_path, 'r'))
        self._nodes = {}
        self._lines = {}
        self._connected = False
        self._weighted_paths = None
        self._route_space = None
        self._Nch = nch

        for node_label in node_json:
            # Create the node instance
            node_dict = node_json[node_label]
            node_dict['label'] = node_label
            node = Node(node_dict)
            self._nodes[node_label] = node

            # Create the line instances
            for connected_node_label in node_dict['connected_nodes']:
                line_dict = {}
                line_label = node_label + connected_node_label
                line_dict['label'] = line_label
                node_position = np.array(node_json[node_label]['position'])
                connected_node_position = np.array \
                    (node_json[connected_node_label]['position'])
                line_dict['length'] = np.sqrt \
                    (np.sum(np.float_power((node_position - connected_node_position), 2)))
                line_dict['Nch'] = self.Nch
                line = Line(line_dict)
                self._lines[line_label] = line

        # if specified , upgrade line
        if not upgrade_line == '':
            self.lines[upgrade_line].noise_figure = self.lines[upgrade_line].noise_figure - 3

    @property
    def Nch(self):
        return self._Nch

    @property
    def nodes(self):
        return self._nodes

    @property
    def lines(self):
        return self._lines

    @property
    def connected(self):
        return self._connected

    @property
    def weighted_paths(self):
        return self._weighted_paths

    @property
    def route_space(self):
        return self._route_space

    def draw(self):
        nodes = self.nodes
        for node_label in nodes:
            n0 = nodes[node_label]
            x0 = n0.position[0]
            y0 = n0.position[1]
            plt.plot(x0, y0, 'go', markersize=10)
            plt.text(x0 + 20, y0 + 20, node_label)
            for connected_node_label in n0.connected_nodes:
                n1 = nodes[connected_node_label]
                x1 = n1.position[0]
                y1 = n1.position[1]
                plt.plot([x0, x1], [y0, y1], 'b')
        plt.title('Network')
        plt.show()

    def find_paths(self, label1, label2):
        cross_nodes = [key for key in self.nodes.keys() if ((key != label1) & (key != label2))]
        cross_lines = self.lines.keys()
        inner_paths = {}
        inner_paths['0'] = label1
        for i in range(len(cross_nodes) + 1):
            inner_paths[str(i + 1)] = []
            for inner_path in inner_paths[str(i)]:
                inner_paths[str(i + 1)] += [inner_path + cross_node
                                            for cross_node in cross_nodes
                                            if ((inner_path[-1] + cross_node in cross_lines) &
                                                (cross_node not in inner_path))]
        paths = []
        for i in range(len(cross_nodes) + 1):
            for path in inner_paths[str(i)]:
                if path[-1] + label2 in cross_lines:
                    paths.append(path + label2)
        return paths

    def connect(self):
        nodes_dict = self.nodes
        lines_dict = self.lines
        for node_label in nodes_dict:
            node = nodes_dict[node_label]
            for connected_node in node.connected_nodes:
                line_label = node_label + connected_node
                line = lines_dict[line_label]
                line.successive[connected_node] = nodes_dict[connected_node]
                node.successive[line_label] = lines_dict[line_label]
        self._connected = True

    def propagate(self, lightpath, occupation=False):
        path = lightpath.path
        start_node = self.nodes[path[0]]
        propagated_lightpath = start_node.propagate(lightpath, occupation)
        return propagated_lightpath

    def optimization(self, lightpath):
        # sets the lightpath power to the optimal \
        # power calculated at each line ( node beginning the line)
        path = lightpath . path
        start_node = self . nodes[path[0]]
        optimized_lightpath = start_node . optimize(lightpath)
        optimized_lightpath . path = path
        return optimized_lightpath


    def set_weighted_paths(self):  # in the pdf he removed the signal power
        if not self.connected:
            self.connect()
        node_labels = self.nodes.keys()
        pairs = []
        for label1 in node_labels:
            for label2 in node_labels:
                if label1 != label2:
                    pairs.append(label1 + label2)

        df = pd.DataFrame()
        paths = []
        latencies = []
        noises = []
        snrs = []

        for pair in pairs:
            for path in self.find_paths(pair[0], pair[1]):
                path_string = ''
                for node in path:
                    path_string += node + '->'
                paths.append(path_string[:-2])

                # Propagation
                lightpath = Lightpath(path) # need to pass more
                lightpath = self.optimization(lightpath)
                lightpath = self.propagate(lightpath, occupation=False)

                latencies.append(lightpath.latency)
                noises.append(lightpath.noise_power)
                snrs.append(10 * np.log10(lightpath.signal_power / lightpath.noise_power))
        df['path'] = paths
        df['latency'] = latencies
        df['noise'] = noises
        df['snr'] = snrs
        self._weighted_paths = df

        route_space = pd.DataFrame()
        route_space['path'] = paths
        for i in range(self.Nch):
            route_space[str(i)] = ['free'] * len(paths)
        self._route_space = route_space

    def available_paths(self, input_node, output_node):  # gives the available paths between two nodes over the all chanels
        if self.weighted_paths is None:
            self.set_weighted_paths()
        all_paths = [path for path in self.weighted_paths.path.values \
                     if ((path[0] == input_node) and (path[-1] == output_node))]
        available_paths = []
        for path in all_paths:
            path_occupancy = self.route_space.loc[self.route_space.path == path].T.values[1:]  # route space è lista di path con una entry per ogni lambda
            if 'free' in path_occupancy:  # se i canali sono tuttio occupati esclude il path
                available_paths.append(path)
        return available_paths

    def find_best_snr(self, input_node, output_node):
        available_paths = self.available_paths(input_node, output_node)
        if available_paths:
            inout_df = self.weighted_paths.loc[self.weighted_paths.path.isin(available_paths)]
            best_snr = np.max(inout_df.snr.values)
            best_path = inout_df.loc[inout_df.snr == best_snr].path.values[0]
        else:
            best_path = None
        return best_path

    def find_best_latency(self, input_node, output_node):
        available_paths = self.available_paths(input_node, output_node)
        if available_paths:
            inout_df = self.weighted_paths.loc[self.weighted_paths.path.isin(available_paths)]
            best_latency = np.min(inout_df.latency.values)
            best_path = inout_df.loc[inout_df.latency == best_latency].path.values[0]
        else:
            best_path = None
        return best_path

    def stream(self, connections, best='latency',  transceiver ='shannon'):
        streamed_connections = []
        for connection in connections:
            input_node = connection.input_node
            output_node = connection.output_node
            signal_power = connection.signal_power
            if best == 'latency':
                path = self.find_best_latency(input_node, output_node)
            elif best == 'snr':
                path = self.find_best_snr(input_node, output_node)
            else:
                print('ERROR: best input not recognized. Value:', best)
                continue
            if path:
                path_occupancy = self.route_space.loc[self.route_space.path == path ].T.values[1:]
                channel = [i for i in range(len(path_occupancy)) \
                       if path_occupancy[i] == 'free'][0]  # uses first free channel
                path = path.replace('->','')
                in_lightpath = Lightpath(path, channel, transceiver=transceiver)
                in_lightpath = self.optimization(in_lightpath)
                #path = in_lightpath.path        # after the debugging
                out_lightpath = self.propagate(in_lightpath, occupation=True)
                out_lightpath.path = path       # after debugging
                self.calculate_bitrate(out_lightpath)

                if out_lightpath.bitrate == 0.0:
                    [self.update_route_space(lp.path, lp.channel, 'free') \
                        for lp in connection.lightpaths ]
                    print("blocked connection ->",input_node, output_node)
                    connection.block_connection()
                else:
                    connection.set_connection(out_lightpath)
                    self.update_route_space( \
                    out_lightpath.path, out_lightpath.channel, 'occupied')
                    if connection.residual_rate_request > 0:
                        self.stream([connection], best, transceiver)
            else:
                [self.update_route_space(lp.path, lp.channel, 'free') for lp in connection.lightpaths ]
                print("blocked connection ->", input_node, output_node)
                connection.block_connection()
            streamed_connections.append(connection)
        return streamed_connections

    @staticmethod
    def path_to_line_set(path):
        path = path.replace('->', '')
        return set([path[i] + path[i + 1] for i in range(len(path) - 1)])


    def update_route_space(self, path, channel, state):
        all_paths = [self.path_to_line_set(p) for p in self.route_space.path.values]
        states = self.route_space[str(channel)]
        lines = self.path_to_line_set(path)
        for i in range(len(all_paths)):
            line_set = all_paths[i]
            if lines.intersection(line_set):
                states[i] = state
        self.route_space[str(channel)] = states

    def calculate_bitrate(self, lightpath, bert=1e-3):
        snr = lightpath.snr
        Bn = 12.5e9
        Rs = lightpath.rs
        if lightpath.transceiver.lower() == 'fixed-rate':
           snrt = 2 * erfcinv(2 * bert) * (Rs / Bn)
           rb = np.piecewise(snr, [snr < snrt, snr >= snrt], [0, 100])
        elif lightpath.transceiver.lower() == 'flex-rate':
           snrt1 = 2 * erfcinv(2 * bert) ** 2 * (Rs / Bn)
           snrt2 = (14 / 3) * erfcinv(3 / 2 * bert) ** 2 * (Rs / Bn)
           snrt3 = 10 * erfcinv(8 / 3 * bert) ** 2 * (Rs / Bn)
           cond1 = (snr < snrt1)
           cond2 = (snr >= snrt1 and snr < snrt2)
           cond3 = (snr >= snrt2 and snr < snrt3)
           cond4 = (snr >= snrt3)
           rb = np.piecewise(snr, [cond1, cond2, cond3, cond4], [0, 100, 200, 400])
        elif lightpath.transceiver.lower() =='shannon':
             rb = 2 * Rs * np.log2(1 + snr * (Rs / Bn)) * 1e-9
        lightpath.bitrate = float(rb)  # set bitrate in lightpath
        return float(rb)

/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/network/Topology.hh"

#include <cassert>

#include "base/trace.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include <iostream>

using namespace std;

const int INFINITE_LATENCY = 10000; // Yes, this is a big hack

// Note: In this file, we use the first 2*m_nodes SwitchIDs to
// represent the input and output endpoint links.  These really are
// not 'switches', as they will not have a Switch object allocated for
// them. The first m_nodes SwitchIDs are the links into the network,
// the second m_nodes set of SwitchIDs represent the the output queues
// of the network.

Topology::Topology(uint32_t num_routers,
                   const vector<BasicExtLink *> &ext_links,
                   const vector<SpuExtLink *> &spu_ext_links,
                   const vector<BasicIntLink *> &int_links)
    : m_nodes(ext_links.size() + spu_ext_links.size()),
      ctrl_nodes(ext_links.size()),
      m_number_of_switches(num_routers),
      m_ext_link_vector(ext_links),
      spu_ext_link_vector (spu_ext_links),
      m_int_link_vector(int_links)
{

    // Total nodes/controllers in network
    assert(m_nodes > 1);

    // analyze both the internal and external links, create data structures.
    // The python created external links are bi-directional,
    // and the python created internal links are uni-directional.
    // The networks and topology utilize uni-directional links.
    // Thus each external link is converted to two calls to addLink,
    // one for each direction.
    //
    // External Links
    for (vector<BasicExtLink*>::const_iterator i = ext_links.begin();
         i != ext_links.end(); ++i) {
        BasicExtLink *ext_link = (*i);
        AbstractController *abs_cntrl = ext_link->params()->ext_node;
        BasicRouter *router = ext_link->params()->int_node;

        int machine_base_idx = MachineType_base_number(abs_cntrl->getType());
        int ext_idx1 = machine_base_idx + abs_cntrl->getVersion(); // input link id
        int ext_idx2 = ext_idx1 + m_nodes; // output link id (different thing taking output?)
        int int_idx = router->params()->router_id + 2*m_nodes; // router id

        // std::cout << "Ext id: " << ext_idx1 << " int id: " << int_idx << " sec ext id: " << ext_idx2 << "\n";
        // create the internal uni-directional links in both directions
        // ext to int
        addLink(ext_idx1, int_idx, ext_link);
        // int to ext
        addLink(int_idx, ext_idx2, ext_link);
    }


	// printf("new m_nodes is %d, ctrl_nodes is %d\n",m_nodes,ctrl_nodes);
    // External Links from SPU
	// printf("Controller nodes: %d\n",ctrl_nodes);
	// TODO: temp hack for now!
	int temp=ctrl_nodes; // can i do num_dirs?
    for (vector<SpuExtLink*>::const_iterator i = spu_ext_links.begin();
         i != spu_ext_links.end(); ++i) {
        SpuExtLink *spu_ext_link = (*i);
        // BasicRouter *router = spu_ext_link->params()->spu_int_node;
		// TODO: use it to get the coreid later on
        // int machine_base_idx = temp;
        // int machine_base_idx = MachineType_base_number(MachineType_Accel);
        int ext_idx1 = temp; // machine_base_idx;
        int ext_idx2 = ext_idx1 + m_nodes;
        // want to use same routers?
        int int_idx = 2*m_nodes + temp - ctrl_nodes; // router->params()->router_id;

        // create the internal uni-directional links in both directions
        // ext to int
        addLink(ext_idx1, int_idx, spu_ext_link);
        // int to ext
        addLink(int_idx, ext_idx2, spu_ext_link);
		temp++; // this makes sure that it increments by 1
    }

    // Internal Links
    for (vector<BasicIntLink*>::const_iterator i = int_links.begin();
         i != int_links.end(); ++i) {
        BasicIntLink *int_link = (*i);
        BasicRouter *router_src = int_link->params()->src_node;
        BasicRouter *router_dst = int_link->params()->dst_node;

        PortDirection src_outport = int_link->params()->src_outport;
        PortDirection dst_inport = int_link->params()->dst_inport;

        // Store the IntLink pointers for later
        m_int_link_vector.push_back(int_link);

        int src = router_src->params()->router_id + 2*m_nodes;
        int dst = router_dst->params()->router_id + 2*m_nodes;

        // create the internal uni-directional link from src to dst
        addLink(src, dst, int_link, src_outport, dst_inport);
    }
}

void
Topology::createLinks(Network *net)
{
    // Find maximum switchID
    SwitchID max_switch_id = 0;
    for (LinkMap::const_iterator i = m_link_map.begin();
         i != m_link_map.end(); ++i) {
        std::pair<SwitchID, SwitchID> src_dest = (*i).first;
        max_switch_id = max(max_switch_id, src_dest.first);
        max_switch_id = max(max_switch_id, src_dest.second);
    }

    // Initialize weight, latency, and inter switched vectors
    int num_switches = max_switch_id+1;

	// Ok this is num_switches = (8+4)*2+4
    // printf("NUM OF SWITCHES IN THE NETWORK: %d\n", num_switches);
    // printf("NUM OF SWITCHES IN THE NETWORK: %d\n", m_number_of_switches);

    Matrix topology_weights(num_switches,
            vector<int>(num_switches, INFINITE_LATENCY));
    Matrix component_latencies(num_switches,
            vector<int>(num_switches, -1));
    Matrix component_inter_switches(num_switches,
            vector<int>(num_switches, 0));

    // Set identity weights to zero
    for (int i = 0; i < topology_weights.size(); i++) {
        topology_weights[i][i] = 0;
    }

    // Fill in the topology weights and bandwidth multipliers
    for (LinkMap::const_iterator i = m_link_map.begin();
         i != m_link_map.end(); ++i) {
        std::pair<int, int> src_dest = (*i).first;
        BasicLink* link = (*i).second.link;
        int src = src_dest.first;
        int dst = src_dest.second;
        // this is 1 for horizontal and 2 for vertical?
        component_latencies[src][dst] = link->m_latency; // 1
        topology_weights[src][dst] = link->m_weight; // 1
     }

    // Walk topology and hookup the links
    Matrix dist = shortest_path(topology_weights, component_latencies,
                                component_inter_switches);
    // std::cout <<  "m_nodes: " << m_nodes << " ctrl_nodes: " << ctrl_nodes << "\n";
    // std::cout << "topology weights size: " << topology_weights.size() << " updated size: " << topology_weights[0].size() << "\n";

    // this is calculated using graph data-structure (flexible)


    /*for(int i = 0; i < nodes; i++) {
        for(int j = 0; j < nodes; j++) {
            std::cout << "i: " << i << " j: " << j << " dist: " << dist[i][j] << "\n";
        }
    }*/

    /*for (int i = 0; i < topology_weights.size(); i++) {
        for (int j = 0; j < topology_weights[i].size(); j++) {
            if(topology_weights[i][j]<INFINITE_LATENCY)
          std::cout <<  "i: " << i << " j: " << j << " topology weights: " << topology_weights[i][j] << "\n" ;
        }
    }*/


    
    /*for (int i = 0; i < topology_weights.size(); i++) {

        for (int j = 0; j < topology_weights[i].size(); j++) {

          std::cout <<  "i: " << i << " j: " << j << " dist weights: " << dist[i][j] << "\n";
        }
    }*/

    // std::cout << "Learning paths\n";

    for (int i = 0; i < topology_weights.size(); i++) {

        for (int j = 0; j < topology_weights[i].size(); j++) {

          int weight = topology_weights[i][j]; // corresponding to original links

            if (weight > 0 && weight != INFINITE_LATENCY) {

                NetDest destination_set =
                        shortest_path_to_node(i, j, topology_weights, dist);

                // std::cout << "making link b/w i: " << i << " j: " << j << "\n";
                // destination set is the set of destinations whose minimum distance passes through i and j
                makeLink(net, i, j, destination_set);
            }
        }
    }
}

void
Topology::addLink(SwitchID src, SwitchID dest, BasicLink* link,
                  PortDirection src_outport_dirn,
                  PortDirection dst_inport_dirn)
{
  // 
    assert(src <= m_number_of_switches+m_nodes+m_nodes);
    assert(dest <= m_number_of_switches+m_nodes+m_nodes);
    // std::cout << "Network graph src: " << src << " dest: " << dest << endl;

    std::pair<int, int> src_dest_pair;
    LinkEntry link_entry;

    src_dest_pair.first = src;
    src_dest_pair.second = dest;
    link_entry.link = link;
    link_entry.src_outport_dirn = src_outport_dirn;
    link_entry.dst_inport_dirn  = dst_inport_dirn;
    m_link_map[src_dest_pair] = link_entry;
}

// spu: how would we make new links here?
// spu:if I use this new convention, I wouldn't need diff makeExtInLink
// FIXME: check if these conditions are correct!
void
Topology::makeLink(Network *net, SwitchID src, SwitchID dest,
                   const NetDest& routing_table_entry)
{
	// printf("START OF MAKE LINK\n");
    // Make sure we're not trying to connect two end-point nodes
    // directly together
	// printf("m_nodes: %u\n",m_nodes);
	// printf("src: %d dest: %d\n",src,dest);
    assert(src >= 2 * m_nodes || dest >= 2 * m_nodes);

    std::pair<int, int> src_dest;
    LinkEntry link_entry;

	// printf("This function should be called 24 times, src: %d dest %d\n",src,dest);
	// printf("m_nodes: %d ctrl_nodes %d\n",m_nodes,ctrl_nodes);
    if (src < m_nodes) { // dest >= 2 * m_nodes
	  // printf("first condition this time\n");
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
		if(src < ctrl_nodes) { // should only come here for 0-15 (instead of 0-16)
	      // std::cout << "cache ext in this time with src: " << src << " and dest: " << dest << "\n"; // issue in this function
          net->makeExtInLink(src, dest - (2 * m_nodes), link_entry.link,
                        routing_table_entry);
		} else {
          // std::cout << "spu ext in this time with src: " << src << " and dest: " << dest << "\n"; // issue in this function
	    // printf("src is external node, dest_id should be varying with switches: %d\n", dest);
        // dest-num_cores+1+i
		  net->makeSpuExtInLink(src, dest - (2 * m_nodes), link_entry.link,
                        routing_table_entry);
		}
    } else if (dest < 2*m_nodes) { // src >= 2 * m_nodes, switch->node
	  // printf("second condition this time\n");
        assert(dest >= m_nodes);
        NodeID node = dest - m_nodes;
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
		// if(dest < m_nodes+ctrl_nodes) { // mem nodes
		if(node < ctrl_nodes) { // mem nodes
          net->makeExtOutLink(src - (2 * m_nodes), node, link_entry.link,
                         routing_table_entry);
		} else {
	  // printf("spu ext out this time\n");
          net->makeSpuExtOutLink(src - (2 * m_nodes), node, link_entry.link,
                         routing_table_entry);
		}
    } else {
        assert((src >= 2 * m_nodes) && (dest >= 2 * m_nodes));
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
        net->makeInternalLink(src - (2 * m_nodes), dest - (2 * m_nodes),
                              link_entry.link,
                              routing_table_entry,
                              link_entry.src_outport_dirn,
                              link_entry.dst_inport_dirn);
    }
	// printf("END OF MAKE LINK\n");
}

// The following all-pairs shortest path algorithm is based on the
// discussion from Cormen et al., Chapter 26.1.
void
Topology::extend_shortest_path(Matrix &current_dist, Matrix &latencies,
    Matrix &inter_switches)
{
    /*

    int num_rows = 4; // m*nodes/2;
    int nodes = current_dist.size();
    // printf("nodes in current dist: %d\n",nodes);
    // TODO: for XY routing, dist should be |X1-X2| + |Y1-Y2|
    // nodes is the number of nodes in the network graph
    // only for internal nodes
    // m_nodes is 16*2+1
    for(int i=2*m_nodes; i<nodes; ++i) {
        int x1=(i-2*m_nodes)%num_rows, y1=(i-2*m_nodes)/num_rows;
        for(int j=2*m_nodes; j<nodes; ++j) {
            int x2=(j-2*m_nodes)%num_rows, y2=(j-2*m_nodes)/num_rows;
            int x_dist = x1-x2; x_dist = x_dist > 0 ? x_dist : -x_dist;
            int y_dist = y1-y2; y_dist = y_dist > 0 ? y_dist : -y_dist;
            current_dist[i][j] = x_dist + y_dist;
            inter_switches[i][j] = current_dist[i][j] + 1;
            latencies[i][j] = current_dist[i][j];
            // std::cout << "Allowed distance b/w i: " << i << " and j: " << j << " is: " << current_dist[i][j] << "\n";
        }
    }

    // for internal to direct external links (ctrl_nodes=17 (m_nodes/2+1))
    int int_node=0; // 80->64 or 80->50
    for(int i=0; i<m_nodes; ++i) {
        // should have dist of 1 from its corresponding internal none
        if(i<ctrl_nodes-1) int_node = 2*m_nodes+i;
        else if(i<ctrl_nodes) int_node = 2*m_nodes;
        else int_node = 2*m_nodes+i-ctrl_nodes;
        // printf("i: %d and int_nodes: %d\n",i,int_node);
        current_dist[i][int_node] = 1;
        inter_switches[i][int_node] = current_dist[i][int_node] + 1;
        latencies[i][int_node] = current_dist[i][int_node];

        current_dist[int_node][m_nodes+i] = 1;
        inter_switches[int_node][m_nodes+i] = current_dist[int_node][m_nodes+i] + 1;
        latencies[int_node][m_nodes+i] = current_dist[int_node][m_nodes+i];
    }

    // external->external
    for(int i=0; i<m_nodes; ++i) { // all spu out nodes
        int int_node1 = 2*m_nodes+i; // corresponding int node
        if(i>=ctrl_nodes) int_node1 -= ctrl_nodes;
        if(i==ctrl_nodes-1) int_node1 = 2*m_nodes; // 66
        for(int j=m_nodes; j<2*m_nodes; ++j) { // all spu in nodes
            int int_node2 = m_nodes+j; // corresponding int node
            if(j>=(m_nodes+ctrl_nodes)) int_node2 -= ctrl_nodes;
            if(j==(m_nodes+ctrl_nodes-1)) int_node2 = 2*m_nodes; // 66 (current_dist[16][51]=2)
            current_dist[i][j] = 1 + current_dist[int_node1][int_node2] + 1;
            inter_switches[i][j] = current_dist[i][j] + 1;
            latencies[i][j] = current_dist[i][j];
        }
    }

    // 16->66, 66->51
    // from internal to other external links
    for(int i=2*m_nodes; i<nodes; ++i) { // int node
        for(int j=m_nodes; j<2*m_nodes; ++j) { // ext node
            int int_node2 = m_nodes+j; // corresponding int node
            if(j>=(m_nodes+ctrl_nodes)) int_node2 -= ctrl_nodes;
            if(j==(m_nodes+ctrl_nodes-1)) int_node2 = 2*m_nodes;
            current_dist[i][j] = current_dist[i][int_node2] + 1;
            inter_switches[i][j] = current_dist[i][j] + 1;
            latencies[i][j] = current_dist[i][j];
        }
    }


    bool change = false;
    */

    bool change = true;
    int nodes = current_dist.size();

      while (change) {
        change = false;
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                int minimum = current_dist[i][j];
                int previous_minimum = minimum;
                int intermediate_switch = -1;
                for (int k = 0; k < nodes; k++) {
                    minimum = min(minimum,
                        current_dist[i][k] + current_dist[k][j]);
                    if (previous_minimum != minimum) {
                        intermediate_switch = k;
                        inter_switches[i][j] =
                            inter_switches[i][k] +
                            inter_switches[k][j] + 1;
                    }
                    previous_minimum = minimum;
                }
                if (current_dist[i][j] != minimum) {
                    change = true;
                    current_dist[i][j] = minimum;
                    assert(intermediate_switch >= 0);
                    assert(intermediate_switch < latencies[i].size());
                    latencies[i][j] = latencies[i][intermediate_switch] +
                        latencies[intermediate_switch][j];
                }
            }
        }
    }
}

Matrix
Topology::shortest_path(const Matrix &weights, Matrix &latencies,
                        Matrix &inter_switches)
{
    Matrix dist = weights;
    extend_shortest_path(dist, latencies, inter_switches);
    return dist;
}

// is src->next comes on the shortest path from src->final
bool
Topology::link_is_shortest_path_to_node(SwitchID src, SwitchID next,
                                        SwitchID final, const Matrix &weights,
                                        const Matrix &dist)
{

    bool is_on_path = weights[src][next] + dist[next][final] == dist[src][final];
    return is_on_path;
    /*
    assert(next>=m_nodes);
    // if external to direct internal link -- push all
    // if internal to direct external link -- only next
    // if internal to internal link -- x,y comparison coded above

    if(src==ctrl_nodes) src=0;
    if(src==m_nodes+ctrl_nodes) src=m_nodes;

    bool is_src_ext = src < m_nodes;
    bool is_next_ext = next < 2*m_nodes;

    assert(is_src_ext || src>=2*m_nodes);
    assert(is_next_ext || next>=2*m_nodes);

    // internal->external
    if(!is_src_ext && is_next_ext) return (next==final);
    // external->internal
    if(is_src_ext && !is_next_ext) return true;

    // check is src->next is a horizontal or vertical link
    assert(src>=2*m_nodes && next>=2*m_nodes);
    // internal->internal
    if(1) { // both are internal nodes
        // if(src>=2*m_nodes && next>=2*m_nodes) { // both are internal nodes
       src -= 2*m_nodes;
       next -= 2*m_nodes;
       is_on_path=false;
       int x1=src%4, y1=src/4;
       int x2=next%4, y2=next/4;
       // dest should be from m_nodes to 2*m_nodes
       final -= m_nodes;
       if(final==ctrl_nodes-1) {
           final=0;
       } else if(final>=ctrl_nodes) {
           final -= ctrl_nodes;
       }
       // 4x4 mesh
       int dest_x=final%4, dest_y=final/4;

       if(x1==x2) { // vertical link
           if(y2>y1) {
               if(dest_y>=y2) is_on_path=true;
           } else {
               if(dest_y<=y1) is_on_path=true;
           }
       } else if(y1==y2) { // horizontal link
           if(x2>x1) {
               if(dest_x>=x2) is_on_path=true;
           } else {
               if(dest_x<=x1) is_on_path=true;
           }
       } else {
           assert(0 && "mesh n/w should only have vert and horizontal links");
       }
    }

   return is_on_path;
     */
}

NetDest
Topology::shortest_path_to_node(SwitchID src, SwitchID next,
                                const Matrix &weights, const Matrix &dist)
{
    NetDest result;

    int d = 0;
    int machines;
    int max_machines;

    machines = MachineType_NUM;
    max_machines = MachineType_base_number(MachineType_NUM);

    // destination could be all nodes from machines to 2*m_machines+i which could be possible dest
    // this is m_nodes to m_nodes
    for (int m = 0; m < machines; m++) {
        for (NodeID i = 0; i < MachineType_base_count((MachineType)m); i++) {
            // we use "d+max_machines" below since the "destination"
            // switches for the machines are numbered
            // [MachineType_base_number(MachineType_NUM)...
            //  2*MachineType_base_number(MachineType_NUM)-1] for the
            // component network

            // std::cout << "src: " << src << " next: " << next << " dest: " << (d+max_machines) << "\n";
            if (link_is_shortest_path_to_node(src, next, d + max_machines,
                    weights, dist)) {
                MachineID mach = {(MachineType)m, i};
                result.add(mach);
            }
            d++;
            // std::cout << "considered location of destinations: " << d << endl;
        }
    }

    DPRINTF(RubyNetwork, "Returning shortest path\n"
            "(src-(2*max_machines)): %d, (next-(2*max_machines)): %d, "
            "src: %d, next: %d, result: %s\n",
            (src-(2*max_machines)), (next-(2*max_machines)),
            src, next, result);

    return result;
}

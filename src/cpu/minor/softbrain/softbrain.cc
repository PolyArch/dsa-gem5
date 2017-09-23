#include <unordered_set>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
 
#include "softbrain.hh"

extern "C" void libsbsim_is_present() {}

using namespace std;

int base_stream_t::ID_SOURCE=0;

void base_stream_t::set_empty(bool b) {
  assert(b && "only goes one way for now");
  assert(!_empty && "can only empty once");

  if(SB_DEBUG::SB_COMMAND_O) {
    if(b == true) {
      std::cout << "COMPLETED: "; 
    } 
    print_status();
  }

  _empty=b;
}


// --------------------------------- CONFIG ---------------------------------------
void soft_config_t::reset() {
  in_ports_active.clear();
  out_ports_active.clear();
  in_port_delay.clear();
  out_ports_lat.clear();
  cgra_in_ports_active.clear();
  input_pdg_node.clear();
  output_pdg_node.clear();
  inst_histo.clear();
}

// ------------------------------ VECTOR PORTS -------------------------------------

void port_data_t::initialize(SbModel* sbconfig, int port, bool isInput) {
  _isInput=isInput;
  _port=port;
}

void port_data_t::reset() {
  assert(_outstanding==0);
  assert(_status==STATUS::FREE);
  //assert(_loc==LOC::NONE); TODO: should this be enforced?
  assert(_mem_data.size()==0); //FIXME: Is this still true?

  assert(_num_in_flight==0);
  assert(_num_ready==0);
  _port_map.clear();

  _total_pushed=0;
}

//total size elements of the vector port
unsigned port_data_t::port_vec_elem() {
  int total=0;
  for(unsigned i = 0; i < port_cgra_elem(); ++i) {
    total+=_port_map[i].second.size();
  }
  return total;
}

void port_data_t::reformat_in() { //rearrange data for CGRA
  
  //resize the data queue to macth the data elements of port vector  
  /*unsigned extra_elem = _mem_data.size() % port_vec_elem();
  if(extra_elem!=0) {
    std::cerr << "Too few elem, zero padding\n";
    _mem_data.resize(_mem_data.size() + port_vec_elem()-extra_elem);
  }*/
  while(_mem_data.size() >= port_vec_elem()) {
    reformat_in_work();
  }
}

void port_data_t::reformat_in_one_vec() { //rearrange data for CGRA
  if(_mem_data.size() >= port_vec_elem() && _num_ready + port_depth() <= CGRA_FIFO_LEN) {
    reformat_in_work();
  }
}

void port_data_t::reformat_in_work() {
  for(unsigned cgra_port = 0; cgra_port < _port_map.size(); ++cgra_port) {
    for(unsigned i = 0; i < _port_map[cgra_port].second.size(); ++i) {
       unsigned ind = _port_map[cgra_port].second[i];
       SBDT val = _mem_data[ind];
       assert(ind < _mem_data.size());
      _cgra_data[cgra_port].push_back(val);
      //cout << dec << ind << " " << std::hex << val << " -> " << cgra_port 
      //     << " (vp" << _port << ")\n";
    }
  }

  _mem_data.erase(_mem_data.begin(),_mem_data.begin()+port_vec_elem());
  _num_ready += port_depth();
}


//pop data from port
SBDT port_data_t::pop_data() {
  SBDT val = _mem_data.front();
  _mem_data.pop_front();
  return val;
}

//pop data from port
SBDT port_data_t::peek_data() {
   SBDT val = _mem_data.front();
  return val;
}

//pop data from port
SBDT port_data_t::peek_data(int i) {
   SBDT val = _mem_data[i];
  return val;
}



//Throw away data in CGRA input ports
void port_data_t::pop(unsigned instances) {
  assert(_num_ready >= instances);
  _num_ready-=instances;
  for(unsigned cgra_port = 0; cgra_port < _port_map.size(); ++cgra_port) {
    assert(_cgra_data[cgra_port].size() >= instances);
    _cgra_data[cgra_port].erase(_cgra_data[cgra_port].begin(),
                                _cgra_data[cgra_port].begin()+instances);
  }
}

//rearrange data from CGRA
void port_data_t::reformat_out() { 
  while(can_output()) {
    reformat_out_work();
  }
}

void port_data_t::reformat_out_one_vec() { 
  if(can_output() && _mem_data.size() + port_vec_elem() <= VP_LEN) {
    assert(_cgra_data[0].size() >= _num_ready);
    assert(_cgra_data[0].size() >= 1);
    reformat_out_work();
  }
}

void port_data_t::reformat_out_work() {
  unsigned cur_mem_data_pos = _mem_data.size(); 
  //_mem_data.insert(_mem_data.begin(),port_vec_elem(),0);
  _mem_data.insert(_mem_data.end(), port_vec_elem(), 0);        //resizing

  unsigned num = 0;
  for(unsigned cgra_port = 0; cgra_port < _port_map.size(); ++cgra_port) {
    assert(num==0 || _cgra_data[cgra_port].size() == num);
    num=_cgra_data[cgra_port].size();
  }

  for(unsigned cgra_port = 0; cgra_port < _port_map.size(); ++cgra_port) {
    for(unsigned i = 0; i < _port_map[cgra_port].second.size(); ++i) {
      unsigned ind = _port_map[cgra_port].second[i];
      _mem_data[cur_mem_data_pos + ind] = _cgra_data[cgra_port].front(); //first element
      assert(_cgra_data[cgra_port].size() > 0);
      _cgra_data[cgra_port].pop_front();
    }
  }
  _num_ready-=port_depth();
}


//Push fake data to each cgra output port
void port_data_t::push_fake(unsigned instances) {
  _num_ready+=instances;
  
  for(unsigned cgra_port = 0; cgra_port < _port_map.size(); ++cgra_port) {
    _cgra_data[cgra_port].insert(_cgra_data[cgra_port].end(),instances,0);
  }
}

// ----------------------------- Port Interface ------------------------------------

void port_interf_t::initialize(SbModel* sbconfig) {
  _in_port_data.resize(NUM_IN_PORTS);
  _out_port_data.resize(NUM_OUT_PORTS);

  for(auto& x : sbconfig->subModel()->io_interf().in_vports) {
    _in_port_data[x.first].initialize(sbconfig,x.first,true);
  }
  
  for(auto& x : sbconfig->subModel()->io_interf().out_vports) {
    _out_port_data[x.first].initialize(sbconfig,x.first,false);
  }
}


// ---------------------------- SOFTBRAIN ------------------------------------------

void softsim_t::step() {
  _ticker->tick();
}

softsim_t::softsim_t(softsim_interf_t* softsim_interf, Minor::LSQ* lsq) : 
  _sim_interf(softsim_interf),
  _lsq(lsq),
  debug(true),
  _dma_c(this),
  _scr_r_c(this,&_dma_c),
  _scr_w_c(this,&_dma_c),
  _port_c(this) {
  _ticker = new ticker_t(this);

  SB_DEBUG::check_env();
 
  int ugh = system("mkdir -p stats/");
  ugh += system("mkdir -p viz/");


  if(SB_DEBUG::VERIF_MEM || SB_DEBUG::VERIF_PORT || SB_DEBUG::VERIF_CGRA || SB_DEBUG::VERIF_SCR ||
     SB_DEBUG::VERIF_CMD) {
    ugh += system("mkdir -p verif/");
    cout << "DUMPING VERIFICATION OUTPUTS (dir: verif/) ... SIMULATION WILL BE SLOWER\n";
    cout << "ALSO, MEMORY ACCESS STATISTICS MAY DIFFER\n";
  }
  if(SB_DEBUG::VERIF_PORT) {
    in_port_verif.open(("verif/" + SB_DEBUG::verif_name + "in_port.txt").c_str(),
                        ofstream::trunc | ofstream::out);
    assert(in_port_verif.is_open());
    out_port_verif.open(("verif/" + SB_DEBUG::verif_name + "out_port.txt").c_str(),
                        ofstream::trunc | ofstream::out);
    assert(out_port_verif.is_open());
  }
  if(SB_DEBUG::VERIF_SCR) {
    scr_rd_verif.open(("verif/" + SB_DEBUG::verif_name + "scr_rd.txt").c_str(),
                        ofstream::trunc | ofstream::out);
    assert(scr_rd_verif.is_open() && scr_rd_verif.good());

    scr_wr_verif.open(("verif/" + SB_DEBUG::verif_name + "scr_wr.txt").c_str(),
                        ofstream::trunc | ofstream::out);
    assert(scr_wr_verif.is_open());
  }
  if(SB_DEBUG::VERIF_CMD) {
    cmd_verif.open(("verif/" + SB_DEBUG::verif_name + "core_trace.txt").c_str(),
                        ofstream::trunc | ofstream::out);
    assert(cmd_verif.is_open());
  }

  const char* sbconfig_file = std::getenv("SBCONFIG");

  if(!SB_DEBUG::SUPRESS_SB_STATS) {
    std::cout << "Loading SB Config (env SBCONFIG):" << sbconfig_file <<"\n";
  }
  _sbconfig = new SbModel(sbconfig_file);
  _port_interf.initialize(_sbconfig);
  scratchpad.resize(SCRATCH_SIZE);


}


void softsim_t::timestamp(){_ticker->timestamp();}
 void softsim_t::run_until(uint64_t i) {_ticker->run_until(i);} 
 void softsim_t::roi_entry(bool enter) {_ticker->roi_entry(enter);}
 void softsim_t::set_not_in_use()  {_ticker->set_not_in_use();}
bool softsim_t::in_use() { return _ticker->in_use();}
bool softsim_t::can_add_stream() {
   _ticker->set_in_use();
   return _in_port_queue.size() < _queue_size && 
                                  _dma_scr_queue.size() < _queue_size;
}
 

//Print out a string on one line idicating hardware status for the previous cycle
// Buffer Sizes                                     |      Bus Activity
// ip 1:5 2:5 7:7; op 1:2 scr_wr:1 cq:1 mem_req:14  | ip: op: scr_rd: scr_wr:   mr: mw: 
void softsim_t::cycle_status() {
  _ticker->timestamp();
  cout << "cq " << _in_port_queue.size() 
                << "," <<_scr_dma_queue.size() << "," <<_dma_scr_queue.size()<<" ";

  cout << "ip "; 
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.in_ports_active[i];
    cout << " " << cur_p << ": "
              << _port_interf.in_port(cur_p).mem_size()  << ","
              << _port_interf.in_port(cur_p).num_ready() <<" ";
  }

  //indirect
  //for(unsigned i = 29; i < 32; ++i) {
  //  unsigned cur_p = i;
  //  cout << " " << cur_p << ": "
  //            << _port_interf.in_port(cur_p).mem_size()  << ","
  //            << _port_interf.in_port(cur_p).num_ready() <<" ";
  //}



  cout << "op "; 
  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.out_ports_active[i];
    cout << " " << cur_p << ": "
              << _port_interf.out_port(cur_p).mem_size()  << ","
              << _port_interf.out_port(cur_p).num_ready() <<" ";  
  }

  //indirect
  //for(unsigned i = 29; i < 32; ++i) {
  //  unsigned cur_p = i;
  //  cout << " " << cur_p << ": "
  //            << _port_interf.out_port(cur_p).mem_size()  << ","
  //            << _port_interf.out_port(cur_p).num_ready() <<" ";  
  //}



  cout << "s_buf:" << _dma_c.scr_buf_size() << " ";
  cout << "m_req:" << _dma_c.mem_reqs() << " ";

  cout << "\t|";
  cout << " m_rd:" << _stat_mem_bytes_rd;
  cout << " m_wr:" << _stat_mem_bytes_wr;
  cout << " s_rd:" << _stat_scr_bytes_rd;
  cout << " s_wr:" << _stat_scr_bytes_wr;

  cout << "\t| ip ";
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.in_ports_active[i];
    if(_port_interf.in_port(cur_p).in_use()) {
      cout << cur_p << " "  << (_port_interf.in_port(cur_p).completed()?"(completed)":"");
    }
  }
  
  //Just the indirect ports
  for(unsigned i = 24; i < 32; ++i) {
    int cur_p=i;
    if(_port_interf.in_port(cur_p).in_use()) {
      cout << cur_p << " "  << (_port_interf.in_port(cur_p).completed()?"(completed)":"");
    }
  }

  cout << "op "; 
  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.out_ports_active[i];
    if(_port_interf.out_port(cur_p).in_use()) {
      cout << cur_p << " "  << (_port_interf.out_port(cur_p).completed()?"(completed)":"");
    }  
  }

  //Just the indirect ports
  for(unsigned i = 24; i < 32; ++i) {
    int cur_p=i;
    if(_port_interf.out_port(cur_p).in_use()) {
      cout << cur_p << " " << (_port_interf.out_port(cur_p).completed()?"(completed)":"");
    }
  }


  clear_cycle();

  cout << "\n";
   //_dma_c.cycle_status();
   //_scr_r_c.cycle_status();
   //_scr_w_c.cycle_status();
   //_port_c.cycle_status();
}

void softsim_t::clear_cycle() {
  //std::map<int,int> _stat_ivp_put;
  //std::map<int,int> _stat_ivp_get;
  //std::map<int,int> _stat_ovp_put;
  //std::map<int,int> _stat_ovp_get;
  //
  //if(_ticker->in_roi()) {
  //  _stat_tot_mem_stored+=_stat_mem_bytes_wr;
  //}

  _stat_mem_bytes_wr=0;
  _stat_mem_bytes_rd=0;
  _stat_scr_bytes_wr=0;
  _stat_scr_bytes_rd=0;
  //_stat_cmds_issued=0;
  //_stat_cmds_complete=0;
}

void softsim_t::print_status() {
   if(done(false,0)) {
     return;
   }
   cout << "---- SOFTBRAIN STATUS ----\n";
   cout << "MEM REQs OUTSTANDING: " << _dma_c.mem_reqs() << "\n";
   cout << "Active SEs:\n";
   _dma_c.print_status();
   _scr_r_c.print_status();
   _scr_w_c.print_status();
   _port_c.print_status();

   cout << "Waiting SEs: (" << _in_port_queue.size() << " " 
                            << _scr_dma_queue.size() << " "
                            << _dma_scr_queue.size() << ")\n";
   for(auto i : _in_port_queue) {i->print_status();}
   for(auto i : _scr_dma_queue) {i->print_status();}
   for(auto i : _dma_scr_queue) {i->print_status();}

   cout << "Ports:\n";
   for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
     unsigned cur_p = _soft_config.in_ports_active[i];
     std::cout << "In Port " << cur_p << ": ";
     std::cout << "  Mem Size: "  << _port_interf.in_port(cur_p).mem_size() <<"";
     std::cout << "  Num Ready: " << _port_interf.in_port(cur_p).num_ready() <<"\n";
   }

//   for(unsigned i = 20; i < 32; ++i) {
//     unsigned cur_p =i;
//     std::cout << "Ind In Port " << cur_p << ": ";
//     std::cout << "  Mem Size: "  << _port_interf.in_port(cur_p).mem_size() <<"";
//     std::cout << "  Num Ready: " << _port_interf.in_port(cur_p).num_ready() <<"\n";
//   }


   for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
     unsigned cur_p = _soft_config.out_ports_active[i];
     std::cout << "Out Port " << cur_p << ": ";
     std::cout << "  In Flight: " <<_port_interf.out_port(cur_p).num_in_flight()<<"";
     std::cout << "  Num Ready: " <<_port_interf.out_port(cur_p).num_ready()<<"";
     std::cout << "  Mem Size: "  <<_port_interf.out_port(cur_p).mem_size()<<"\n";
   }

}

void softsim_t::reset_statistics() { //currently unused
  _stat_comp_instances = 0;
  _stat_scratch_read_bytes = 0;
  _stat_scratch_write_bytes = 0;
  _stat_scratch_reads = 0;
  _stat_scratch_writes = 0;

  _stat_start_cycle = 0;
  _stat_stop_cycle = 0;
  _stat_commands_issued = 0;

  _stat_tot_mem_fetched=0;
  _stat_tot_mem_stored=0;

  _stat_tot_loads=0;
  _stat_tot_stores=0;
  _stat_tot_mem_store_acc=0;
  _stat_tot_mem_load_acc=0;
}


void softsim_t::pedantic_statistics(std::ostream& out) {
   out << "Scratch Read bytes: " << _stat_scratch_read_bytes << "\n";
   out << "Scratch Write bytes:" << _stat_scratch_write_bytes << "\n";

   double scratch_per_cyc = ((double)(_stat_scratch_reads+_stat_scratch_writes))/((double)_roi_cycles);
   out << "Scratch access per cycle: " << scratch_per_cyc << "\n\n";
   double l2_acc_per_cyc = ((double)(_stat_tot_loads+_stat_tot_stores))/((double)_roi_cycles);
   out << "L2 accesses per cycle: " << l2_acc_per_cyc << "\n\n";
   double l2_miss_per_cyc = ((double)(_stat_tot_mem_load_acc+_stat_tot_mem_store_acc))/((double)_roi_cycles);
   out << "L2 misses per cycle:       " << l2_miss_per_cyc << "\n\n";

  out << "CGRA Activity Histogram (inst/switch:times used)\n";
  for(auto i : _total_histo) {
    out << name_of_inst(i.first) << ":" << i.second << "\n"; 
  }
  out << "\n Port Activity Histogram (size:times used)\n ";
  for(auto i : _vport_histo) {
    out << i.first << ":" << i.second << "\n";
  }

  out.flush();
}


void softsim_t::print_statistics(std::ostream& out) {
   if(SB_DEBUG::VERIF_PORT) { //flush all the log files
     in_port_verif.flush();
     out_port_verif.flush();
   }
   if(SB_DEBUG::VERIF_SCR) {
     scr_rd_verif.flush();
     scr_wr_verif.flush();
   }
   if(SB_DEBUG::VERIF_CMD) {
     cmd_verif.flush();
   }

   if(SB_DEBUG::SUPRESS_SB_STATS) {
     return;  // If we don't want to print stats
   }

   out << "\n*** SOFTBRAIN STATISTICS ***\n";
   out.precision(4);
   out << dec;
   //out << "Start Cycle: " << _stat_start_cycle << "\n";
   //out << "Stop  Cycle: " << _stat_stop_cycle << "\n\n";

   out << "Cycles in ROI: " << _roi_cycles << "\n";
   //out << "RISCV insts in ROI: " << timing_model_t::get()->get_insts_in_roi() << "\n";
   out << "Simulator ROI Time: " << ((double)_ticker->elpased_time_in_roi())
                                    /1000000000 << " sec\n";

   out << "Comp. Instances: " << _stat_comp_instances << "\n";
   out << "Commands Issued: " << _stat_commands_issued << "\n\n";

   out << "Activity Ratio: " 
      << ((double)_stat_comp_instances)/((double)_roi_cycles) << "\n";
   out << "SB Insts / Computation Instance: "
      << ((double)_stat_sb_insts)/((double)_stat_comp_instances) << "\n";
   out << "SB Insts / Cycle: "
      << ((double)_stat_sb_insts)/((double)_roi_cycles) << " (overall activity factor)\n";

   out << "\n";
   out << "Avg. Scratch Read Port Req Size:  " 
     << ((double)_stat_scratch_read_bytes)/((double)_stat_scratch_reads) << "\n";
   out << "Avg. Scratch Write Port Req Size: " 
     << ((double)_stat_scratch_write_bytes)/((double)_stat_scratch_writes) << "\n";

   out << "Scratch reads per cycle:  " << ((double)_stat_scratch_reads)/((double)_roi_cycles) << "\n";
   out << "Scratch writes per cycle: " << ((double)_stat_scratch_writes)/((double)_roi_cycles) << "\n";
   out << "\n";


   out << "Avg. L2 Load Port Req Size:  " 
     << ((double)_stat_tot_mem_fetched)/((double)_stat_tot_loads) << "\n";
   out << "Avg. L2 Store Port Req Size: " 
     << ((double)_stat_tot_mem_stored)/((double)_stat_tot_stores) << "\n";

   out << "L2 loads per cycle:    " << ((double)_stat_tot_loads)/((double)_roi_cycles) << "\n";
   out << "L2 stores per cycle:   " << ((double)_stat_tot_stores)/((double)_roi_cycles) << "\n";
   out << "\n";

   out << "L2 load misses per cycle:  " << ((double)_stat_tot_mem_load_acc)/((double)_roi_cycles) << "\n";
   out << "L2 store misses per cycle: " << ((double)_stat_tot_mem_store_acc)/((double)_roi_cycles) << "\n";
   out << "\n";

   out << "Multicore Cycle Estimates: (1-10 cores): ";
   for(int i = 1; i <= 10; ++i) { // estimate performance
     uint64_t multicore_cyc = _roi_cycles / i;
     uint64_t mem_cyc = _stat_tot_mem_load_acc+_stat_tot_mem_store_acc;
     out << std::max(multicore_cyc,mem_cyc) << " ";
   }
   out << "\n";

   out << "Multicore Activity Estimates: (1-10 cores): ";
   for(int i = 1; i <= 10; ++i) { // estimate performance
     uint64_t multicore_cyc = _roi_cycles / i;
     uint64_t mem_cyc = _stat_tot_mem_load_acc+_stat_tot_mem_store_acc;
     uint64_t cycles = std::max(multicore_cyc,mem_cyc);
     double active_ratio = ((double)(_stat_comp_instances)/i)/((double)cycles);
     out << active_ratio << " ";
   }
   out << "\n\n";
}

//wait and print stats
void softsim_t::print_stats() {
  wait_until_done(true);
 
  print_statistics(std::cout);
  print_status();

   ofstream stat_file;
   if(char* name = getenv("SB_RUN_NAME")) {
     stat_file.open(string("stats/")+name+".sb-stats", ofstream::trunc | ofstream::out);
   } else {
     stat_file.open("stats/default.sb-stats", ofstream::trunc | ofstream::out);
   }


   assert(stat_file.is_open());
   print_statistics(stat_file);
   pedantic_statistics(stat_file);


}

uint64_t softsim_t::now() {
 return _ticker->now();
}

// --------------------------SCHEDULE STREAMS ONTO CONTROLLERS-----------------------
// This is essentially the stream dispatcher
void softsim_t::schedule_streams() {
  int str_width=_sbconfig->dispatch_width();
  int str_issued=0;

  bool blocked_ivp[64] = {0}; 
  bool blocked_ovp[64] = {0};
  
  //schedule for ports (these need to go in program order per-vp)
  for(auto i = _in_port_queue.begin(); i!=_in_port_queue.end() && str_issued<str_width;){
    base_stream_t* ip = *i;
    port_data_t* in_vp=NULL;
    port_data_t* out_vp=NULL;
    port_data_t* out_ind_vp=NULL;

    bool scheduled_in=false;
    bool scheduled_out=false;
    bool scheduled_dma_scr=false;
    bool scheduled_scr_dma=false;

    if(auto dma_port_stream = dynamic_cast<dma_port_stream_t*>(ip)) {
      int port = dma_port_stream->_in_port;
      in_vp = &_port_interf.in_port(port);
      if(in_vp->can_take(LOC::DMA) && !blocked_ivp[port] &&
        (scheduled_in = _dma_c.schedule_dma_port(*dma_port_stream)) ) {
        in_vp->set_status(port_data_t::STATUS::BUSY, LOC::DMA);
      }

    } else if(auto scr_port_stream = dynamic_cast<scr_port_stream_t*>(ip)) {
      int port = scr_port_stream->_in_port;
      in_vp = &_port_interf.in_port(port);
      if(in_vp->can_take(LOC::SCR) && !blocked_ivp[port] &&
          (scheduled_in = _scr_r_c.schedule_scr_port(*scr_port_stream)) ) {
        in_vp->set_status(port_data_t::STATUS::BUSY, LOC::SCR);
        _outstanding_scr_read_streams--;
      }      

    } else if(auto port_port_stream = dynamic_cast<port_port_stream_t*>(ip)) {
      int in_port = port_port_stream->_in_port;
      int out_port = port_port_stream->_out_port;
      in_vp = &_port_interf.in_port(in_port);
      out_vp = &_port_interf.out_port(out_port);
      if(in_vp->can_take(LOC::PORT) && out_vp->can_take(LOC::PORT) &&
          !blocked_ivp[in_port] && !blocked_ovp[out_port] && 
          (scheduled_in = _port_c.schedule_port_port(*port_port_stream)) ) {
        scheduled_out = true;
        in_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
        out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
      }

    } else if(auto const_port_stream = dynamic_cast<const_port_stream_t*>(ip)) {
      int port = const_port_stream->_in_port;
      in_vp = &_port_interf.in_port(port);
      if(in_vp->can_take(LOC::PORT) && !blocked_ivp[port] && 
            (scheduled_in = _port_c.schedule_const_port(*const_port_stream) )) {
        in_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
      }
      
    } else if(auto port_dma_stream = dynamic_cast<port_dma_stream_t*>(ip)) {
      int port = port_dma_stream->_out_port;
      out_vp = &_port_interf.out_port(port);
      if(out_vp->can_take(LOC::DMA) && !blocked_ovp[port] &&
           (scheduled_out = _dma_c.schedule_port_dma(*port_dma_stream))) {
        out_vp->set_status(port_data_t::STATUS::BUSY, LOC::DMA);
      }

    } else if(auto port_scr_stream = dynamic_cast<port_scr_stream_t*>(ip)) {
      int port = port_scr_stream->_out_port;
      out_vp = &_port_interf.out_port(port);
      if(out_vp->can_take(LOC::SCR) && !blocked_ovp[port] &&
            (scheduled_out = _scr_w_c.schedule_port_scr(*port_scr_stream))) {
        out_vp->set_status(port_data_t::STATUS::BUSY, LOC::SCR);
      }

    } else if(auto indirect_stream = dynamic_cast<indirect_stream_t*>(ip)) {
      int out_ind_port = indirect_stream->_ind_port; //grab output res
      assert(out_ind_port >= 16); //only allowed to be in this range
      int in_port = indirect_stream->_in_port;

      out_ind_vp = &_port_interf.out_port(out_ind_port); //this is indirect output port
      in_vp = &_port_interf.in_port(in_port);

      if(in_vp->can_take(LOC::PORT) && out_ind_vp->can_take(LOC::PORT)
          && !blocked_ivp[in_port] && !blocked_ovp[out_ind_port] &&
            (scheduled_in = _dma_c.schedule_indirect(*indirect_stream) )) {
        scheduled_out = true;
        in_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
        out_ind_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
      }
    } else if(auto indirect_wr_stream = dynamic_cast<indirect_wr_stream_t*>(ip)) {
      int out_ind_port = indirect_wr_stream->_ind_port; //grab output res
      assert(out_ind_port >= 16); //only allowed to be in this range
      int out_port = indirect_wr_stream->_out_port;

      out_ind_vp = &_port_interf.out_port(out_ind_port); //this is indirect output port
      out_vp = &_port_interf.out_port(out_port);

      if(out_vp->can_take(LOC::PORT) && out_ind_vp->can_take(LOC::PORT)
          && !blocked_ovp[out_port] && !blocked_ovp[out_ind_port] &&
            (scheduled_in = _dma_c.schedule_indirect_wr(*indirect_wr_stream) )) {
        scheduled_out = true;
        out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
        out_ind_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
      }
    } else if(auto dma_scr_stream = dynamic_cast<dma_scr_stream_t*>(ip)) {
      scheduled_dma_scr = _dma_c.schedule_dma_scr(*dma_scr_stream);
    } else if(auto scr_dma_stream = dynamic_cast<scr_dma_stream_t*>(ip)) {
      scheduled_scr_dma = _dma_c.schedule_scr_dma(*scr_dma_stream);
    }


    if(in_vp) blocked_ivp[in_vp->port()] = true; //prevent OOO access
    if(out_vp) blocked_ovp[out_vp->port()] = true;
    if(out_ind_vp) blocked_ovp[out_ind_vp->port()] = true;

    if(scheduled_in || scheduled_out || scheduled_dma_scr || scheduled_scr_dma) {
      str_issued++;
      if(SB_DEBUG::SB_COMMAND_I) {
        _ticker->timestamp();
        cout << " ISSUED:";
        ip->print_status();
      }
      delete ip;
      i=_in_port_queue.erase(i);
      if(_ticker->in_roi()) {
        _stat_commands_issued++;
      }
    } else {
      //we failed to schedule anything!
      if(_sbconfig->dispatch_inorder()) { //if INORDER-dispatch, stop!
        break;
      } else { //if we're in OOO-dispatch mode, just keep going...
        ++i;
      }
    }
  }

  //DMA->Scratch and Scratch->DMA
  //These will only get activated when OOO activation is enabled
  if(_dma_scr_queue.size() > 0 ) {
    bool scheduled_dma_scr = _dma_c.schedule_dma_scr(*_dma_scr_queue.front());
    if(scheduled_dma_scr) {
      if(SB_DEBUG::SB_COMMAND_I) {
        _ticker->timestamp();
        cout << " ISSUED:";
        _dma_scr_queue.front()->print_status();
      }
      if(_ticker->in_roi()) {
        _stat_commands_issued++;
      }

      delete _dma_scr_queue.front();
      _dma_scr_queue.pop_front();
    }
  }

  if(_scr_dma_queue.size() > 0 ) {
    bool scheduled_scr_dma = _dma_c.schedule_scr_dma(*_scr_dma_queue.front());
    if(scheduled_scr_dma) {
      if(SB_DEBUG::SB_COMMAND_I) {
        _ticker->timestamp();
        cout << " ISSUED:";
        _scr_dma_queue.front()->print_status();
      }
      if(_ticker->in_roi()) {
        _stat_commands_issued++;
      }

      delete _scr_dma_queue.front();
      _scr_dma_queue.pop_front();
    }
  }
}

bool dma_controller_t::schedule_dma_port(dma_port_stream_t& new_s) {
  for(auto& s : _dma_port_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}

bool dma_controller_t::schedule_dma_scr(dma_scr_stream_t& new_s) {
  if(_dma_scr_stream.empty()) {
    _dma_scr_stream=new_s;
    return true;
  }
  return false;
}

bool dma_controller_t::schedule_port_dma(port_dma_stream_t& new_s) {
  for(auto& s : _port_dma_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}

bool dma_controller_t::schedule_scr_dma(scr_dma_stream_t& new_s) {
  if(_scr_dma_stream.empty()) {
    _scr_dma_stream=new_s;
    return true;
  }
  return false;
}

bool dma_controller_t::schedule_indirect(indirect_stream_t& new_s) {
  for(auto& s : _indirect_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}

bool dma_controller_t::schedule_indirect_wr(indirect_wr_stream_t& new_s) {
  for(auto& s : _indirect_wr_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}


bool scratch_read_controller_t::schedule_scr_port(scr_port_stream_t& new_s) {
  if(_scr_port_stream.empty()) {
    _scr_port_stream=new_s;
    return true;
  }
  return false;
}
bool scratch_write_controller_t::schedule_port_scr(port_scr_stream_t& new_s) {
  if(_port_scr_stream.empty()) {
    _port_scr_stream=new_s;
    return true;
  }
  return false;
}
bool port_controller_t::schedule_port_port(port_port_stream_t& new_s) {
  for(auto& s : _port_port_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}
bool port_controller_t::schedule_const_port(const_port_stream_t& new_s) {
  for(auto& s : _const_port_streams) {
    if(s.empty()) {
      s=new_s;
      return true;
    }
  }
  return false;
}


/*
void softsim_t::deschedule_streams() {
  _dma_c->deschedule_streams();
  _scr_r_c->deschedule_streams();
  _scr_w_c->deschedule_streams();
  _port_c->deschedule_streams();
}

void dma_controller_t::deschedule_streams() {

}
void scratch_read_controller_t::deschedule_streams() {

}
void scratch_write_controller_t::deschedule_streams() {

}
void port_controller_t::deschedule_streams() {

}*/

#if 0
void dma_controller_t::schedule_streams() {
  //must start at head of queue
  for(int i = 0; i < _dma_port_queue.size(); ++i) {
    auto& new_stream = _dma_port_queue[i];
    port_data_t& in_vp = _sb->port_interf()->in_port(_dma_port_queue[i]._port);
    if(in_vp.in_use()) {
      continue; // don't schedule this one if the vp is in use...
    }

    if(_sb->port_interf().in_port(i)) {
      for(auto& s : _dma_port_streams) {
        if(s.empty()) {
          in_vp.set_in_use(true);
          s=_new_stream;
          _dma_port_queue.pop();
          break;
        }
      }
      break;
    }
  }
  if(_dma_scr_queue.size() > 0) {
    if(_dma_scr_stream.empty()) {
      _dma_scr_stream=_dma_scr_queue.top();
      _dma_scr_queue.pop();
    }
  }

  for(int i = 0; i < _port_dma_queue.size(); ++i) {
    auto& new_stream = _port_dma_queue[i];
    port_data_t& out_vp = _sb->port_interf()->out_port(_port_dma_queue[i]._port);
    if(out_vp.in_use()) {
      continue; // don't schedule this one if the vp is in use...
    }

    if(_sb->port_interf().in_port(i)) {
      for(auto& s : _port_dma_streams) {
        if(s.empty()) {
          out_vp.set_in_use(true);
          s=_new_stream;
          _port_dma_queue.pop();
          break;
        }
      }
      break;
    }
  }

  if(_scr_dma_queue.size() > 0) {
    if(_scr_dma_stream.empty()) {
      _scr_dma_stream=_scr_dma_queue.top();
      _scr_dma_queue.pop();
    }
  }
}

void scratch_read_controller_t::schedule_streams() { 
  if(_scr_port_queue.size() > 0) {
    for(auto& s : _scr_port_streams) {
      if(s.empty()) {
        s=_scr_port_queue.top();
        _scr_port_queue.pop();
      }
    } 
  }
}
#endif

//HACK
 static bool DOUBLE_PORT=true;

void apply_mask(uint64_t* raw_data, vector<bool>  mask, std::vector<SBDT>& data) {
  assert(mask.size()==8); 
  SBDT* u64data = (SBDT*)raw_data;
  for(int i = 0; i < mask.size(); ++i) {
    if(mask[i]) {
      data.push_back(u64data[i]);
    }
  }
}

void apply_map(uint64_t* raw_data, const vector<int>& imap, std::vector<SBDT>& data) {
  //SBDT* u64data = (SBDT*)raw_data;
  assert(imap.size() != 0);
  data.resize(imap.size()); 
  for(int i = 0; i < imap.size(); ++i) {
    data[i] = raw_data[imap[i]];
  }
}


void dma_controller_t::port_resp(unsigned cur_port) {
    if(Minor::LSQ::LSQRequestPtr response = _sb->_lsq->findResponse(cur_port)) {
      auto& in_vp = _sb->port_interf().in_port(cur_port);

      PacketPtr packet = response->packet;
      if(packet->getSize()!=MEM_WIDTH) {
        assert(0 && "weird memory response size");
      }

      vector<SBDT> data;
      if(response->sdInfo->mask.size() > 0) {
        apply_mask(packet->getPtr<uint64_t>(), response->sdInfo->mask, data);
      } else if(response->sdInfo->map.size() > 0) {
        apply_map(packet->getPtr<uint64_t>(),  response->sdInfo->map, data);
      }

      if(in_vp.can_push_vp(data.size())) {
        bool last = response->sdInfo->last;

        if(SB_DEBUG::MEM_REQ) {
          _sb->_ticker->timestamp();
          std::cout << "(real) data into port " << cur_port << ", size: " 
                    << data.size() << " elements" << (last ? "last" : "") << "\n";
        }

        if(_sb->_ticker->in_roi()) {
          _sb->_stat_mem_bytes_rd+=data.size()*DATA_WIDTH;
        }
        in_vp.push_data(data);

        //BEGIN HACK
        if(cur_port==25) { //TRIPPLE PORT : )
          auto& in_vp1 = _sb->port_interf().in_port(cur_port+1);
          auto& in_vp2 = _sb->port_interf().in_port(cur_port+2);
          in_vp1.push_data(data);
          in_vp2.push_data(data);
        }
        if(cur_port==26) { //DOUBLE PORT : )
          DOUBLE_PORT=true;
          auto& in_vp1 = _sb->port_interf().in_port(cur_port+1);
          in_vp1.push_data(data);
        }
        //END HACK

        if(last) {
          auto& in_vp = _sb->port_interf().in_port(cur_port);
          if(SB_DEBUG::VP_SCORE2) {
            cout << "SOURCE: DMA->PORT2\n";
          }

          in_vp.set_status(port_data_t::STATUS::FREE);
        }
        _sb->_lsq->popResponse(cur_port);
        return;
      }
    }
}


// ---------------------STREAM CONTROLLER TIMING ------------------------------------
// If response, can issue load
// Limitations: 1 Response per cycle (512 bits/cycle)
void dma_controller_t::cycle(){
  //Handle response from REAL memory

  //Memory read to config
  if(Minor::LSQ::LSQRequestPtr response =_sb->_lsq->findResponse(CONFIG_STREAM)) {
    PacketPtr packet = response->packet;
    _sb->configure(packet->getAddr(),packet->getSize()/8,packet->getPtr<uint64_t>());
    _sb->_lsq->popResponse(CONFIG_STREAM);
  }

  //memory read to scratch
  if(Minor::LSQ::LSQRequestPtr response =_sb->_lsq->findResponse(SCR_STREAM)) {

    PacketPtr packet = response->packet;
    if(packet->getSize()!=MEM_WIDTH) {
      assert(0 && "weird memory response size");
    }
    vector<SBDT> data;
    apply_mask(packet->getPtr<uint64_t>(), response->sdInfo->mask, data);

    if(_scr_write_buffer.push_data(response->sdInfo->scr_addr, data)) {
      if(SB_DEBUG::MEM_REQ) {
        _sb->_ticker->timestamp();
        std::cout << "data into scratch " << response->sdInfo->scr_addr 
                  << ":" << data.size() << "elements" << "\n";
      }
      if(_sb->_ticker->in_roi()) {
        _sb->_stat_mem_bytes_rd+=data.size();
      }

      //handled_req=true;
      _fake_scratch_reqs--;

      _sb->_lsq->popResponse(SCR_STREAM);
    }
  }


  //Memory Read to Ports
  for(unsigned i = 0; i < _sb->_soft_config.in_ports_active.size(); ++i) {
    int cur_port = _sb->_soft_config.in_ports_active[i];
    port_resp(cur_port);
  }
  for(unsigned i = 28; i < 32; ++i ) {  //ind read ports
    port_resp(i);
  }


  //FIXME: Eliminate fake mem

  //Response from fake memory
  //bool handled_req=false;
  bool blocked_ivp[64] = {0}; 

  for(auto i = _fake_mem.begin(), e = _fake_mem.end(); i!=e; ++i) {
    uint64_t cycle = i->first;
    auto& mem_req = i->second;
    if(cycle <= _sb->now()) {
      //fix this so that we check first
      if(mem_req.port==-1) {
        //Handle DMA->Scratch Case
        if(_scr_write_buffer.push_data(mem_req.scr_addr, mem_req.data)) {
          if(SB_DEBUG::MEM_REQ) {
          _sb->_ticker->timestamp();
          std::cout << "data into scratch " << mem_req.scr_addr << ":" << mem_req.data.size() << "elements" << "\n";
          }
          if(_sb->_ticker->in_roi()) {
            _sb->_stat_mem_bytes_rd+=mem_req.data.size();
          }

          //handled_req=true;
          _fake_mem.erase(i);
          _fake_scratch_reqs--;
          break;
        }
      } else { 
        //Handle DMA->Port
        auto& in_vp = _sb->port_interf().in_port(mem_req.port);
        if(!blocked_ivp[mem_req.port] &&  in_vp.can_push_vp(mem_req.data.size())) {
          if(SB_DEBUG::MEM_REQ) {
          _sb->_ticker->timestamp();
          std::cout << "data into port " << mem_req.port << ":" << mem_req.data.size() << "elements" << "\n";
          }
        
          if(_sb->_ticker->in_roi()) {
            _sb->_stat_mem_bytes_rd+=mem_req.data.size();
          }
          in_vp.push_data(mem_req.data,mem_req.orig_cmd_id);

          //BEGIN HACK
          if(mem_req.port==25) { //TRIPPLE PORT : )
            auto& in_vp1 = _sb->port_interf().in_port(mem_req.port+1);
            auto& in_vp2 = _sb->port_interf().in_port(mem_req.port+2);
            in_vp1.push_data(mem_req.data,mem_req.orig_cmd_id);
            in_vp2.push_data(mem_req.data,mem_req.orig_cmd_id);
          }
          if(mem_req.port==26) { //DOUBLE PORT : )
            DOUBLE_PORT=true;
            auto& in_vp1 = _sb->port_interf().in_port(mem_req.port+1);
            in_vp1.push_data(mem_req.data,mem_req.orig_cmd_id);
          }

          //END HACK

          //handled_req=true;

          if(mem_req.last) {
            auto& in_vp = _sb->port_interf().in_port(mem_req.port);
            if(SB_DEBUG::VP_SCORE2) {
              cout << "SOURCE: DMA->PORT2\n";
            }

            in_vp.set_status(port_data_t::STATUS::FREE);
          }

          _fake_mem.erase(i);
          break;
        } else {
          blocked_ivp[mem_req.port]=true;
        }
      }
    } else {
      break;
    }
  }
  //if(handled_req) {
  //  make_request(_tq_read, _which_read);
  //} else {
  //  make_request(_tq, _which);
  //}
  make_request(0,_tq_read,_which_read); //read request
  make_request(_tq_read,_tq,_which); //write request
}

void dma_controller_t::print_status() {
  for(auto& i : _dma_port_streams) {if(!i.empty()){i.print_status();}} //TODO: maybe optimize this later?
  if(!_dma_scr_stream.empty()) {
    _dma_scr_stream.print_status();
  }
  for(auto& i : _port_dma_streams) {if(!i.empty()){i.print_status();}}
//  _scr_dma_stream.finish_cycle();
}

void dma_controller_t::finish_cycle() {
//  for(auto& i : _dma_port_streams) {i.finish_cycle();} //TODO: maybe optimize this later?
//  _dma_scr_stream.finish_cycle();
//  for(auto& i : _port_dma_streams) {i.finish_cycle();}
//  _scr_dma_stream.finish_cycle();
  _scr_write_buffer.finish_cycle();
  _scr_read_buffer.finish_cycle();
}


void dma_controller_t::make_request(unsigned s, unsigned t, unsigned& which) {
  if(which < s) {
    which=s;
  } 

  //find minimum balance -- this is horribly inneficient.  
  //                           need to redo eventually
  //uint64_t min_balance=-1;
  //if(which < _tq_read) {
  //  for(unsigned i = s; i < t; ++i) {

  //  }

  //  if(_dma_port_streams.size()) { //DMA READ (addr)
  //    
  //  }
  //}

  for(unsigned i = s; i < t; ++i) {
    which=(which+1)==t ? s:which+1; //rolling increment
    std::vector<SBDT> data;

    if(which<_dma_port_streams.size()) { //DMA READ (addr)
      dma_port_stream_t& dma_s = _dma_port_streams[which];

      if(dma_s.stream_active() && _fake_mem.size() < MAX_MEM_REQS) {
        auto& in_vp = _sb->port_interf().in_port(dma_s._in_port);

        if(_sb->_lsq->sd_transfers[dma_s._in_port].unreservedRemainingSpace()>0 
           && _sb->_lsq->canRequest()) {
          _sb->_lsq->sd_transfers[dma_s._in_port].reserve();

          req_read(dma_s,-1/*scratch_addr*/);
          if(dma_s.empty()) {
            if(SB_DEBUG::VP_SCORE2) { cout << "SOURCE: DMA->PORT \n";}
            in_vp.set_status(port_data_t::STATUS::COMPLETE, LOC::DMA);
          }
          return;
        }
      }
    } else if(which==_dma_port_streams.size()) {
      //READ TO SCRATCH
      if(_dma_scr_stream.stream_active() && _fake_mem.size() < MAX_MEM_REQS) {
        if(_sb->_lsq->sd_transfers[SCR_STREAM].unreservedRemainingSpace()>0
          && _sb->_lsq->canRequest()) {
          _sb->_lsq->sd_transfers[SCR_STREAM].reserve();
          int words_read = req_read(_dma_scr_stream,
                                    _dma_scr_stream._scratch_addr);

        //need to update scratch address for next time
          _dma_scr_stream._scratch_addr+= words_read * DATA_WIDTH;

        _fake_scratch_reqs++;

        return;
      }
      }
    } else if(which < _dma_port_streams.size()+1+_indirect_streams.size()) {
      int which_indirect = which - (_dma_port_streams.size()+1);
      indirect_stream_t& ind_s = _indirect_streams[which_indirect];

      auto& in_ind_vp = _sb->port_interf().in_port(ind_s._ind_port);

      if(ind_s.stream_active()) {
        if(in_ind_vp.mem_size()>0 ) {
          auto& in_vp = _sb->port_interf().in_port(ind_s._in_port);
          if(in_vp.num_can_push() > 8) { //make sure vp isn't full

            //pull_data_indirect(ind_s,data,mem_complete_cyc);
            ind_read_req(ind_s,-1/*for scratch*/);

            if(ind_s.empty()) {
              //destination ivp
              if(SB_DEBUG::VP_SCORE2) { cout << "SOURCE: Indirect DMA->PORT \n";}
              in_vp.set_status(port_data_t::STATUS::COMPLETE, LOC::DMA);

              //out_ind_vp is where we keep track of the resource
              auto& out_ind_vp = _sb->port_interf().out_port(ind_s._ind_port);
              if(SB_DEBUG::VP_SCORE2) { cout << "SOURCE: Indirect DMA->PORT \n";}
              out_ind_vp.set_status(port_data_t::STATUS::FREE);
            }

            return;
          }
        }
      }

    //WRITE CASES START HERE
    } else if(which < _tq_read +_port_dma_streams.size()) {
      // PORT->DMA Write
      int which_wr=which-_tq_read;  //TODO: did i f anything up?
      port_dma_stream_t& dma_s = _port_dma_streams[which_wr];

      if(dma_s.stream_active()) {
        port_data_t& out_port = _sb->port_interf().out_port(dma_s._out_port);
        if((out_port.mem_size()>0) && //TODO:  unoptimal if we didn't wait?
           (dma_s._garbage || (_sb->_lsq->canRequest() &&
           _sb->_lsq->sd_transfers[MEM_WR_STREAM].unreservedRemainingSpace()>0))) {

          _sb->_lsq->sd_transfers[MEM_WR_STREAM].reserve();
          if(dma_s._garbage) {
            while(dma_s.stream_active() && out_port.mem_size()>0) {
              out_port.pop_data();//get rid of data
              dma_s.pop_addr(); //get rid of addr
            }
          } else {
            req_write(dma_s,out_port);
            //uint64_t ignored;
            //write_data_port(dma_s,out_port,ignored);
          }
          bool is_empty = dma_s.check_set_empty();
          if(is_empty) {
            if(SB_DEBUG::VP_SCORE2) {cout << "SOURCE: PORT->DMA\n";}
            out_port.set_status(port_data_t::STATUS::FREE);
          }

          return;
        }
      }
    } else if (which == _tq_read + _port_dma_streams.size()) {
      //Scratch DMA -- UNIMPLEMENTED
      /*
      if(_scr_dma_stream.stream_active()) {
        if(_scr_dma_stream.data_ready()) {
          write_data(_scr_dma_stream);
          return;
        }
      }*/
    } else {
      int which_wr=which-_tq_read-1-_port_dma_streams.size(); 
      indirect_wr_stream_t& ind_s = _indirect_wr_streams[which_wr];
      if(ind_s.stream_active()) {
        port_data_t& out_port = _sb->port_interf().out_port(ind_s._out_port);
        port_data_t& ind_port = _sb->port_interf().in_port(ind_s._ind_port);

        if(out_port.mem_size()>0 && ind_port.mem_size()>0) { 
          ind_write_req(ind_s);
          return;
        }
      }
    }
  }
}

int dma_controller_t::req_read(mem_stream_base_t& stream,
                                uint64_t scr_addr) {
  addr_t prev_addr = 0;
  addr_t addr = stream.cur_addr();
  addr_t base_addr = addr & MEM_MASK;  //this is the request address...
  addr_t max_addr = base_addr+MEM_WIDTH;

  assert(addr!=0 && "cannot load address 0x0");
    std::fill(mask.begin(), mask.end(), 0); 
  int words=0;

  while(addr < max_addr && addr > prev_addr  && stream.stream_active()) {
    prev_addr=addr;
      mask[(addr-base_addr)/DATA_WIDTH]=1;
    addr = stream.pop_addr();
    words+=1;
  }

  //TODO: add l2_miss statistics -- probably somewhere else....
  if(_sb->_ticker->in_roi()) {
    _sb->_stat_tot_mem_fetched+=words*DATA_WIDTH;
    _sb->_stat_tot_loads+=1;
  }

  stream.check_set_empty(); //do this first so last is set

  bool last = stream.empty();

  SDMemReqInfoPtr sdInfo = NULL; 
  if(scr_addr==-1) { //READ TO PORTS
    sdInfo = new SDMemReqInfo(scr_addr, stream.in_port(), mask, last);
  } else { //READ TO SCRATCH
    sdInfo = new SDMemReqInfo(scr_addr, SCR_STREAM, mask, last);
}

  //make request
  _sb->_lsq->pushRequest(stream.minst(),true/*isLoad*/,NULL/*data*/,
              MEM_WIDTH/*cache line*/, base_addr, 0/*flags*/, 0 /*res*/,
              sdInfo);

  return words;
}

void dma_controller_t::ind_read_req(indirect_stream_t& stream, 
                                    uint64_t scr_addr) {  //always get the input port if getting data 
  port_data_t& ind_vp = _sb->port_interf().in_port(stream._ind_port);

  cout << stream._ind_port << "\n";

  bool first=true;
  addr_t base_addr=0;
  addr_t max_addr=0;

  vector<int> imap;

  while(ind_vp.mem_size() && stream.stream_active()) {
    addr_t idx  = stream.calc_index(ind_vp.peek_data());
    addr_t addr = stream._index_addr + idx * stream.index_size();

    //cout << "idx:" << idx << "\taddr:" << hex << addr << dec << "\n";
    if(first) {
      first=false;
      base_addr = addr & MEM_MASK;
      max_addr = base_addr+MEM_WIDTH;
    } else { //not first
      if(addr > max_addr || addr < base_addr) {
        break;
      }
    }

    int index = addr - base_addr;
    imap.push_back(index);

    stream.pop_elem();
    ind_vp.pop_data();
  }
  bool last = stream.empty();

  SDMemReqInfoPtr sdInfo = NULL; 
  if(scr_addr==-1) { //READ TO PORTS
    sdInfo = new SDMemReqInfo(scr_addr, stream.in_port(), imap, last);
  } else { //READ TO SCRATCH
    sdInfo = new SDMemReqInfo(scr_addr, SCR_STREAM, imap, last);
  }

  //make request
  _sb->_lsq->pushRequest(stream.minst(),true/*isLoad*/,NULL/*data*/,
              MEM_WIDTH/*cache line*/, base_addr, 0/*flags*/, 0 /*res*/,
              sdInfo);


  if(_sb->_ticker->in_roi()) {
    _sb->_stat_tot_mem_fetched+=imap.size()*DATA_WIDTH; //TODO: fix once the stream size is fixed
    _sb->_stat_tot_loads+=1;
  }

  stream.check_set_empty();
}

void dma_controller_t::ind_write_req(indirect_wr_stream_t& stream) {
  port_data_t& out_vp = _sb->port_interf().out_port(stream._out_port);
  port_data_t& ind_vp = _sb->port_interf().in_port(stream._ind_port);

  bool first=true;
  addr_t base_addr=0;
  addr_t prev_addr=0;

  unsigned bytes_written = 0;

  std::vector<uint8_t> data;
  data.resize(MEM_WIDTH);
  uint8_t* data8 = data.data();
  //uint16_t* data16 = (uint16_t*)data8;
  uint64_t* data64 = (uint64_t*)data8;

  //FIXME: The size of the indirect stream should be made configurable!
  int stream_size = 8;

  int index = 0;
  while(out_vp.mem_size() && ind_vp.mem_size() && stream.stream_active()) {
    addr_t idx  = stream.calc_index(ind_vp.peek_data());
    addr_t addr = stream._index_addr + idx * stream.index_size();

    //cout << "idx:" << idx << "\taddr:" << hex << addr << dec << "\n";

    if(first) {
      first=false;
      base_addr = addr & MEM_MASK;
    } else { //not first
      if(prev_addr + stream_size != addr) { //addr > max_addr || addr < base_addr) {
        break;
      }
    }

    SBDT val = out_vp.peek_data();

    prev_addr=addr;
    ++index;
    data64[++index]=val;

    //_sb->st_mem(addr,val,mem_cycle,stream.minst());
    out_vp.pop_data();
    bytes_written+=sizeof(SBDT);

    out_vp.pop_data();
    bytes_written+=sizeof(SBDT);

    stream.pop_elem();
    ind_vp.pop_data();
  }

  SDMemReqInfoPtr sdInfo = new SDMemReqInfo(-1, MEM_WR_STREAM, 
                                            mask /*N/A*/, false /*N/A*/);

  //make store request
  _sb->_lsq->pushRequest(stream.minst(),false/*isLoad*/, data8,
              bytes_written, base_addr, 0/*flags*/, 0 /*res*/, sdInfo);

  if(_sb->_ticker->in_roi()) {
    _sb->_stat_mem_bytes_wr+=bytes_written;
    _sb->_stat_tot_stores++;
  }

  bool is_empty = stream.check_set_empty();
  if(is_empty) {
    if(SB_DEBUG::VP_SCORE2) {
      cout << "SOURCE: INDIRECT PORT -> DMA\n";
    }
    out_vp.set_status(port_data_t::STATUS::FREE);
    if((stream._ind_port==26&&!DOUBLE_PORT) || stream._ind_port==27 ) {

    } else {
      ind_vp.set_status(port_data_t::STATUS::FREE);
    }
  }

}

//Creates a write request for a contiguous chunk of data smaller than one cache line
void dma_controller_t::req_write(port_dma_stream_t& stream, port_data_t& vp) {
  addr_t addr = stream.cur_addr();
  addr_t init_addr = addr;
  addr_t base_addr = addr & MEM_MASK;
  addr_t max_addr = base_addr+MEM_WIDTH;

  assert(addr!=0 && "cannot store to address 0x0");

  unsigned elem_written = 0;

  //std::fill(mask.begin(), mask.end(), 0); 
  //mask[(addr-base_addr)/DATA_WIDTH]=1;

  int data_width = DATA_WIDTH; 
  if(stream._shift_bytes==2) {
    data_width = 2;
  }
  addr_t prev_addr=addr-data_width;

  std::vector<uint8_t> data;
  data.resize(MEM_WIDTH);
  uint8_t* data8 = data.data();
  uint16_t* data16 = (uint16_t*)data8;
  uint64_t* data64 = (uint64_t*)data8;

  //go while stream and port does not run out
  while(addr < max_addr && (addr == (prev_addr + data_width)) && 
        stream.stream_active() && vp.mem_size()>0) { 
    SBDT val = vp.peek_data();

    if(stream._shift_bytes==2) {
      data16[elem_written++]=(uint16_t)(val&0xFFFF);
    } else { //assuming data_width=64
      data64[elem_written++]=val;
    }
    vp.pop_data(); //pop the one we peeked
    prev_addr=addr;
    addr = stream.pop_addr();
  }

  unsigned bytes_written = elem_written * data_width;
  SDMemReqInfoPtr sdInfo = new SDMemReqInfo(-1, MEM_WR_STREAM, 
                                            mask /*N/A*/, false /*N/A*/);

  //make store request
  _sb->_lsq->pushRequest(stream.minst(),false/*isLoad*/, data8,
              bytes_written, init_addr, 0/*flags*/, 0 /*res*/, sdInfo);


  if(SB_DEBUG::MEM_REQ) {
    _sb->_ticker->timestamp();
    cout << bytes_written << "-byte write request for port->dma\n";
  }

  if(_sb->_ticker->in_roi()) {
    _sb->_stat_mem_bytes_wr+=bytes_written;
    _sb->_stat_tot_stores++;
    _sb->_stat_tot_mem_stored+=bytes_written;

    //bool l2_miss=(cycle_mem_complete-start_cycle)>5; 
    //if(l2_miss) {
    //  _sb->_stat_tot_mem_load_acc++;
    //}
  }
}


//WRITE SCRATCH -> DMA NOT IMPLEMENTED
/*
void dma_controller_t::write_data(scr_dma_stream_t& stream) {
  addr_t addr = stream.cur_addr();
  addr_t base_addr = addr & MEM_MASK;
  addr_t max_addr = base_addr+MEM_WIDTH;

   //go while stream and port does not run out
  while(addr < max_addr && addr!=0 && stream.data_ready()>0) { 
    SBDT val = stream.pop_data();
    _sb->st_mem(addr,val);
    addr = stream.pop_addr();
  }
  stream.check_set_empty();
}
*/


void scratch_read_controller_t::cycle() {
  for(int i=0; i < 2; ++i) {
    if(i==_which) {
      //read to port
      if(_scr_port_stream.stream_active()) {
        addr_t prev_addr = SCRATCH_SIZE;
        addr_t addr = _scr_port_stream.cur_addr(); //this is scratch addr
        addr_t base_addr = addr & SCR_MASK;
        addr_t max_addr = base_addr+SCR_WIDTH;

        if(SB_DEBUG::SCR_BARRIER) {
          _sb->_ticker->timestamp();
          cout << "scr_rd " << hex << addr << " -> " << max_addr << "\n";
        }

        auto& in_vp = _sb->port_interf().in_port(_scr_port_stream._in_port);
        std::fill(mask.begin(), mask.end(), 0); 


        //go while stream and port does not run out
        while(addr < max_addr && (addr > prev_addr || prev_addr == SCRATCH_SIZE)
               && _scr_port_stream.stream_active()) { 
          // keep going while stream does not run out
          SBDT val=0;
          assert(addr + DATA_WIDTH <= SCRATCH_SIZE);
          std::memcpy(&val, &_sb->scratchpad[addr], DATA_WIDTH);
          if(SB_DEBUG::SCR_ACC) {
            cout << "scr_addr:" << hex << addr << " read " << val << "\n";
          }
          in_vp.push_data(val);
          prev_addr=addr;
          mask[(addr-base_addr)/DATA_WIDTH]=1;
          addr = _scr_port_stream.pop_addr();
        }

        if(SB_DEBUG::VERIF_SCR) {
          _sb->scr_rd_verif << hex << setw(8) << setfill('0') << base_addr << " ";
          for(uint64_t i = base_addr; i < max_addr; ++i) {
            _sb->scr_rd_verif << setw(2) << setfill('0') << (unsigned)_sb->scratchpad[i];
          }
          _sb->scr_rd_verif << " ";
          for(unsigned i = 0; i < SCR_WIDTH/DATA_WIDTH; ++i) {
            _sb->scr_rd_verif << setw(1) << setfill('0') << (unsigned)mask[i];
          }
          _sb->scr_rd_verif << "\n";
        }

        bool is_empty = _scr_port_stream.check_set_empty();
        if(is_empty) {
          if(SB_DEBUG::VP_SCORE2) {
            cout << "SOURCE: SCR->PORT\n";
          }
          in_vp.set_status(port_data_t::STATUS::FREE);
        }
        if(_sb->_ticker->in_roi()) {
          _sb->_stat_scr_bytes_rd+=DATA_WIDTH;
          _sb->_stat_scratch_read_bytes+=DATA_WIDTH;
          _sb->_stat_scratch_reads++;
        }

      }
      break;
    } else {
      //read to dma_write
      // THIS IS NOT IMPLEMETNED, SO...
      //          _dma_c->_scr_read_buffer.push_data(val);

    }
  }

  _which=_which?0:1;
}

void scratch_read_controller_t::finish_cycle() {
//  _scr_port_stream.finish_cycle(); //TODO: maybe optimize this later?
}

void scratch_read_controller_t::print_status() {
  if(!_scr_port_stream.empty()){
   _scr_port_stream.print_status();
  }
}


void scratch_write_controller_t::cycle() {
  for(int i=0; i < 2; ++i) {
    if(i==_which) {
      //write from port
      if(_port_scr_stream.stream_active()) {
        addr_t addr = _port_scr_stream._scratch_addr;
        addr_t base_addr = addr & SCR_MASK;
        addr_t max_addr = base_addr+SCR_WIDTH;

        port_data_t& out_vp = _sb->port_interf().out_port(_port_scr_stream._num_bytes);

        //go while stream and port does not run out
        while(addr < max_addr && _port_scr_stream._num_bytes>0 //enough in dest
                              && out_vp.mem_size()) { //enough in source
          SBDT val = out_vp.pop_data(); 
          assert(addr + DATA_WIDTH <= SCRATCH_SIZE);
          std::memcpy(&_sb->scratchpad[addr], &val, sizeof(SBDT));
          addr = _port_scr_stream.pop_addr();

          if(_sb->_ticker->in_roi()) {
            _sb->_stat_scr_bytes_wr+=DATA_WIDTH;
            _sb->_stat_scratch_write_bytes+=DATA_WIDTH;
          }
        }
        if(_sb->_ticker->in_roi()) {
          _sb->_stat_scratch_writes+=1;
        }
          
        bool is_empty = _port_scr_stream.check_set_empty();
        if(is_empty) {
          if(SB_DEBUG::VP_SCORE2) {
            cout << "SOURCE: PORT->SCR\n";
          }
          out_vp.set_status(port_data_t::STATUS::FREE);
        }
        break;
      }     
    } else {

      //write from dma
      if(_dma_c->_scr_write_buffer.data_ready() > 0) {        
        addr_t addr = _dma_c->_scr_write_buffer._scr_address;
        addr_t base_addr = addr & SCR_MASK;
        addr_t max_addr = base_addr+SCR_WIDTH;

        if(SB_DEBUG::SCR_BARRIER) {
          _sb->_ticker->timestamp();
          cout << "scr_wr " << hex << addr << " -> " << max_addr << "\n";
        }


        if(SB_DEBUG::SCR_ACC) {
          cout << "dma->scr buffer ready to write: " << _dma_c->_scr_write_buffer.data_ready() << "to addr:" << hex << addr << dec << "\n"; 
        }


        while(addr < max_addr && _dma_c->_scr_write_buffer.data_ready()>0) { //enough in source
          addr = _dma_c->_scr_write_buffer._scr_address;
          SBDT val = _dma_c->_scr_write_buffer.pop_data();

          if(SB_DEBUG::SCR_ACC) {
            cout << hex << val << dec << "\n";
          }
      
          assert(addr + DATA_WIDTH <= SCRATCH_SIZE);
          std::memcpy(&_sb->scratchpad[addr], &val, sizeof(SBDT));

          if(_sb->_ticker->in_roi()) {
            _sb->_stat_scr_bytes_wr+=DATA_WIDTH;
            _sb->_stat_scratch_write_bytes+=DATA_WIDTH;
          }
        }
        if(_sb->_ticker->in_roi()) {
          _sb->_stat_scratch_writes+=1;
        }


        if(SB_DEBUG::SCR_ACC) {
          cout << "\n";
        }


      }
    }
  }

  _which=_which?0:1;
}

void scratch_write_controller_t::finish_cycle() {
//  _port_scr_stream.finish_cycle(); //TODO: maybe optimize this later?
}

void scratch_write_controller_t::print_status() {
  if(!_port_scr_stream.empty()){ 
    _port_scr_stream.print_status();
  }
}


void port_controller_t::cycle() {
  //Port-Port
  for(unsigned i = 0; i < _port_port_streams.size(); ++i) {
    _which_pp=(_which_pp+1)==_port_port_streams.size() ? 0:_which_pp+1; //rolling incr
    auto& pi=_sb->port_interf();

    auto& stream = _port_port_streams[_which_pp];
    if(!stream.stream_active()) {
      continue;
    }

    port_data_t& vp_out = pi.out_port(stream._out_port);
    port_data_t& vp_in = pi.in_port(stream._in_port);
    if(vp_out.mem_size() && vp_in.mem_size() < VP_LEN ) { // okay go for it
      
      for(int i = 0; i < PORT_WIDTH && vp_in.mem_size() < VP_LEN && 
                         vp_out.mem_size() && stream.stream_active(); ++i) {
        SBDT val = vp_out.pop_data();
        vp_in.push_data(val);
        stream._num_elements--;
      }
      bool is_empty = stream.check_set_empty();
      if(is_empty) {
        if(SB_DEBUG::VP_SCORE2) {
          cout << "SOURCE: PORT->PORT\n";
        }
        port_data_t& in_vp = _sb->port_interf().in_port(stream._in_port);
        in_vp.set_status(port_data_t::STATUS::FREE);

        if(SB_DEBUG::VP_SCORE2) {
          cout << "SOURCE: PORT->PORT\n";
        }
        port_data_t& out_vp = _sb->port_interf().out_port(stream._out_port);
        out_vp.set_status(port_data_t::STATUS::FREE);
      }

      break;
    }
  }

  //Const-Port
  for(unsigned i = 0; i < _const_port_streams.size(); ++i) {
    _which_cp=(_which_cp+1)==_const_port_streams.size() ? 0:_which_cp+1; //rolling incr
    auto& pi=_sb->port_interf();
    auto& stream=_const_port_streams[_which_cp];
    if(!stream.stream_active()) {
      continue;
    }

    port_data_t& vp_in = pi.in_port(stream._in_port);

    if(vp_in.mem_size() < VP_LEN) { // enough space, so go for it
      for(int i = 0; i < PORT_WIDTH && vp_in.mem_size() < VP_LEN 
                  && stream.stream_active(); ++i) {
        vp_in.push_data(stream._constant);
        stream._num_elements--;
      }
      bool is_empty = stream.check_set_empty();
      if(is_empty) {
        port_data_t& in_vp = _sb->port_interf().in_port(stream._in_port);

        if(SB_DEBUG::VP_SCORE2) {
          cout << "SOURCE: CONST->PORT\n";
        }
        in_vp.set_status(port_data_t::STATUS::FREE);
      }

      break;
    }
  }
}

void port_controller_t::finish_cycle() {
//  for(auto& i : _port_port_streams) {i.finish_cycle();}  //TODO: maybe optimize later?
//  for(auto& i : _const_port_streams) {i.finish_cycle();}
}

void port_controller_t::print_status() {
  for(auto& i : _port_port_streams) {if(!i.empty()){i.print_status();}}  
  for(auto& i : _const_port_streams) {if(!i.empty()){i.print_status();}}
}



bool softsim_t::done(bool show,int mask) {
  if(!_dma_c.done(show,mask) || !_scr_r_c.done(show,mask) || 
     !_scr_w_c.done(show,mask) || !_port_c.done(show,mask)) {
    return false;
  }
  
  if(!cgra_done(show,mask)) {
    return false;
  }

  if(mask==0 || mask&WAIT_CMP) {
    if(_in_port_queue.size()) {
      if(show) {
        cout << "Main Queue Not Empty\n";
      }   
      return false;
    }
    if(_scr_dma_queue.size()) {
      if(show) {
        cout << "SCR->DMA Queue Not Empty\n";
      }   
      return false;
    }
  }
  
  if(mask==0 || mask&WAIT_SCR_WR) {
    if(_dma_scr_queue.size()) {
      if(show) {
        cout << "DMA->SCR Queue Not Empty\n";
      }   
      return false;
    }
  }

  if(mask==0 || mask&WAIT_SCR_RD || mask&WAIT_SCR_RD_Q) {
    if(_outstanding_scr_read_streams != 0) {
      if(show) {
        cout << "Number of Outstanding Read Streams is not 0 (its " 
             << _outstanding_scr_read_streams << ")\n";
      }
      return false;
    }
  }

  return true;
}


bool dma_controller_t::done(bool show, int mask) {

  if(mask==0 || mask&WAIT_CMP || mask&WAIT_SCR_WR) {
    if(!_dma_scr_stream.empty()) {
        if(show) {
          cout << "DMA -> SCR Stream Not Empty\n";
        }
      return false;
    }
  }

  if(mask==0 || mask&WAIT_CMP || mask&WAIT_SCR_WR) {
    if(!_scr_write_buffer.empty_buffer()) {
      if(show) {
        _sb->_ticker->timestamp();
        cout << "SCR Write Buffer Not Empty\n";
      }
      return false;
    }
  }

  if(mask==0 || mask&WAIT_CMP) {
    for(auto& i : _dma_port_streams) {
      if(!i.empty()) { 
        if(show) {
          cout << "DMA -> Port Stream Not Empty\n";
        }
        return false;
      }
    } 
  
    for(auto& i : _port_dma_streams) {
      if(!i.empty()) {
        if(show) {
          cout << "PORT -> DMA Stream Not Empty\n";
        }
        return false;
      }
    }
    if(!_fake_mem.empty()) {
      if(show) {
        cout << "Fake Mem contains data\n";
      }
      return false;
    }
  }

  //TODO: Think about scr->DMA when we need it
    /*if(!_scr_dma_stream.empty()) {
      if(show) {
        cout << "SCR -> DMA Stream Not Empty\n";
      }
      return false;
    }*/
  /*
   *    if(!_scr_read_buffer.empty_buffer()) {
      if(show) {
        cout << "SCR Read Buffer Not Empty\n";
      }
      return false;
    }*/

  if((mask & WAIT_SCR_WR) && _fake_scratch_reqs) {
    if(show) {
      cout << "Fake scratch reqs: " << _fake_scratch_reqs 
           << " (is not zero) \n";
    }
    return false;
  }
  
  return true;
}

bool scratch_write_controller_t::done(bool show, int mask) {
  if(mask==0 || mask&WAIT_CMP || mask&WAIT_SCR_WR) {
    if(!_port_scr_stream.empty()) {
      if(show) {
        cout << "PORT -> SCR Stream Not Empty\n";
      }
      return false;
    }
  }
  return true;
}

bool scratch_read_controller_t::done(bool show, int mask) {
  if(mask==0 || mask&WAIT_CMP || mask&WAIT_SCR_RD) {
    if(!_scr_port_stream.empty()) {
      if(show) {
        cout << "SCR -> PORT Stream Not Empty\n";
      }
      return false;
    }
  }
  return true;
}


bool port_controller_t::done(bool show, int mask) {
  if(mask==0 || mask&WAIT_CMP) {
    for(auto& i : _port_port_streams) {
      if(!i.empty()) {
        if(show) {
          cout << "PORT -> PORT Stream Not Empty\n";
        }
        return false;
      } 
    }
    for(auto& i : _const_port_streams) {
      if(!i.empty()) {
        if(show) {
          cout << "CONST -> PORT Stream  Not Empty\n >>>";
          i.print_status();
        }
        return false;
      } 
    }
  }
  return true;
}



bool softsim_t::cgra_done(bool show,int mask) {
  if(mask==0 || mask&WAIT_CMP) {
    for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
      int cur_port = _soft_config.in_ports_active[i];
      auto& in_vp = _port_interf.in_port(cur_port);
      if(in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size()) { 
        if(show) {
          cout << "In VP: " << cur_port << " Not Empty (";
          cout << "in_use: " << in_vp.in_use() << (in_vp.completed()?"(completed)":"");
          cout << " num_rdy: " << in_vp.num_ready();
          cout << " mem_size: " << in_vp.mem_size();
          cout << ") \n";
        }
  
        return false;
      }
    }
  }
  if(mask==0) {
    for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
      int cur_port = _soft_config.out_ports_active[i];
      auto& out_vp = _port_interf.out_port(cur_port);
      if(out_vp.in_use() || out_vp.num_ready() || out_vp.mem_size()) { 
        if(show) {
          cout << "Out VP: " << cur_port << " Not Empty (";
          cout << "in_use: " << out_vp.in_use();
          cout << " num_rdy: " << out_vp.num_ready();
          cout << " mem_size: " << out_vp.mem_size();
          cout << ") \n";
        }
        return false;
      }
    }
  }
  // TODO: check for things passing through CGRA

  return true; 
}


void softsim_t::wait_until_done(bool show, int mask) {
  uint64_t max_wait_for_progress=10000;  

  if(done(false,mask)) {
    //cout << "Done without waiting\n";
    return;
  }

  while(!done(false,mask) && _waiting_cycles < max_wait_for_progress) {
    _ticker->tick();
  }
  done(show,mask);
 
  //print_statistics(std::cout);

  if(_waiting_cycles >= max_wait_for_progress) {
    cout << "FATAL ERROR: SOFTBRAIN WAITING TOO LONG! (";
    cout << dec << "waiting for:" << _waiting_cycles << "cycles)\n";

    print_statistics(cerr);
    print_status();
    assert(0);
  }

}

// ----------------------- SB STREAM COMMANDS ---------------------------------------
#ifdef SB_TIMING

void softsim_t::load_dma_to_scratch(addr_t mem_addr, 
    uint64_t stride, uint64_t access_size, uint64_t num_strides,
    addr_t scratch_addr) {
  dma_scr_stream_t* s = new dma_scr_stream_t();
  s->_mem_addr=mem_addr;
  s->_scratch_addr=scratch_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    _ticker->timestamp();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    return;
  }

  //WAIT_FOR(can_add_dma_scr_stream,1500);

  add_dma_scr_stream(s);
}

void softsim_t::load_dma_to_port(addr_t mem_addr,
     uint64_t stride, uint64_t access_size, uint64_t num_strides,
     int in_port) {
  dma_port_stream_t* s = new dma_port_stream_t();
  s->_mem_addr=mem_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_in_port=in_port;

  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    return;
  }

  //WAIT_FOR(can_add_dma_port_stream,1500);

  add_dma_port_stream(s);
}

void softsim_t::write_dma(int out_port,
    uint64_t stride, uint64_t access_size, uint64_t num_strides,
    addr_t mem_addr, int shift_bytes, int garbage) {
  port_dma_stream_t* s = new port_dma_stream_t();
  s->_mem_addr=mem_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_out_port=out_port;
  s->_shift_bytes=shift_bytes;
  s->_garbage=garbage;


  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    return;
  }

  //WAIT_FOR(can_add_port_dma_stream,1500);

  add_port_dma_stream(s);
}

void softsim_t::load_scratch_to_port(addr_t scratch_addr,
  uint64_t stride, uint64_t access_size, uint64_t num_strides,
  int in_port) {
  scr_port_stream_t* s = new scr_port_stream_t();
  s->_mem_addr=scratch_addr; //NOTE: Here _mem_addr *is* the scratchpad address
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_in_port=in_port;

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    _ticker->timestamp();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    return;
  }

  //WAIT_FOR(can_add_scr_port_stream,1500);

  _outstanding_scr_read_streams++;

  add_scr_port_stream(s);
}

void softsim_t::write_scratchpad(int out_port, 
    addr_t scratch_addr, uint64_t num_bytes) {
  assert(0 && "this not implemented");
  port_scr_stream_t* s = new port_scr_stream_t();
  s->_out_port=out_port;
  s->_scratch_addr=scratch_addr;
  s->_num_bytes=num_bytes;

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    _ticker->timestamp();
    s->print_status(); 
  }

  if(num_bytes==0) {
    return;
  }

  //WAIT_FOR(can_add_port_scr_stream,1500);

  add_port_scr_stream(s);
}

void softsim_t::reroute(int out_port, int in_port, uint64_t num_elem) {
  port_port_stream_t* s = new port_port_stream_t();
  s->_out_port=out_port;
  s->_in_port=in_port;
  s->_num_elements=num_elem;

  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status();
  }

  if(num_elem==0) {
    return;
  }

  //WAIT_FOR(can_add_port_port_stream,1500);

  add_port_port_stream(s);
}

//Configure an indirect stream with params
void softsim_t::indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
    uint64_t num_elem) {
  indirect_stream_t* s = new indirect_stream_t();
  s->_ind_port=ind_port;
  s->_type=ind_type;
  s->_in_port=in_port;
  s->_index_addr=index_addr;
  s->_index_in_word=0;
  s->_num_elements=num_elem;

  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status();
  }

  if(num_elem==0) {
    return;
  }

  //WAIT_FOR(can_add_indirect_stream,1500);

  add_indirect_stream(s);
}

//Configure an indirect stream with params
void softsim_t::indirect_write(int ind_port, int ind_type, int out_port, 
    addr_t index_addr, uint64_t num_elem) {
  indirect_wr_stream_t* s = new indirect_wr_stream_t();
  s->_ind_port=ind_port;
  s->_type=ind_type;
  s->_out_port=out_port; //comes from sb_dma_addr
  s->_index_addr=index_addr;
  s->_index_in_word=0;
  s->_num_elements=num_elem;

  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status();
  }

  //WAIT_FOR(can_add_indirect_stream,1500);

  add_indirect_stream(s);
}

void softsim_t::write_constant(int in_port, SBDT constant, uint64_t num_elem) {
  const_port_stream_t* s = new const_port_stream_t();
  s->_in_port=in_port;
  s->_constant=constant;
  s->_num_elements=num_elem; 

  if(debug && SB_DEBUG::SB_COMMAND) {
    _ticker->timestamp();
    s->print_status();
  }

  if(num_elem==0) {
    return;
  }

  //WAIT_FOR(can_add_const_port_stream,1500);

  add_const_port_stream(s);
}


#else
// --------------------- DEPRICATED NON-TIMING VERSIONS ---------------------------

void softsim_t::load_dma_to_scratch() {
  addr_t cur_stride_addr = _mem_addr;
  addr_t cur_scratch_addr = _scratch_addr;

  //total strides
  for(unsigned i = 0; i < _num_strides; ++i, cur_stride_addr+=_stride) {
    addr_t cur_access_addr = cur_stride_addr;

    //for each stride -- get data by DATA_WIDTH
    //needs 2 data loads of 8B for one scratchpad line
    for(; cur_access_addr < cur_stride_addr + _access_size; cur_access_addr += DATA_WIDTH, cur_scratch_addr += DATA_WIDTH) {
      SBDT val = LD_MEM(cur_access_addr);
      std::memcpy(scratchpad.data() + cur_scratch_addr, &val, sizeof(SBDT));      //DATA_WIDTH bytes copied
      assert(cur_scratch_addr + DATA_WIDTH <= SCRATCH_SIZE);
    }
  }

  _port_interf.reformat_in(_in_port);         //why do we need this here ??
}

void softsim_t::load_scratch_to_port() {
  SBDT val;

  for(unsigned i = 0; i < _num_strides * scratch_line_size; i+=DATA_WIDTH) {
    std::memcpy(&val, scratchpad.data() + _scratch_addr + i, sizeof(SBDT));        //8B
    _port_interf.push_data(val,_in_port); 

  }

  if(_sb->_ticker->in_roi()) {
    _stat_scratch_read_bytes += _num_strides * scratch_line_size;
  }
  _port_interf.reformat_in(_in_port);
  do_cgra();
}

//each offset element in vector port is 8B or SBDT
void softsim_t::load_dma_to_port() { //iterate over _stride and _access_size
  addr_t cur_stride_addr = _mem_addr;

  for(unsigned i = 0; i < _num_strides; ++i, cur_stride_addr+=_stride) {
    addr_t cur_access_addr = cur_stride_addr;
    for(; cur_access_addr < cur_stride_addr + _access_size; cur_access_addr += DATA_WIDTH) {
      SBDT mem_data = LD_MEM(cur_access_addr);
      //cout << "PULLED: " << hex << mem_data << " @ " << cur_access_addr << dec << "\n";
      _port_interf.push_data(mem_data, _in_port);              //8B data 
    }
  }

  _port_interf.reformat_in(_in_port);
  do_cgra();
}

//write to scratchpad
//stride size == vector_port elements size ??
void softsim_t::write_scratchpad() {
  unsigned stride_size = _port_interf.out_port(_out_port).port_vec_elem();        //total elems in vector ports
  assert( _port_interf.out_port(_out_port).mem_size() >= _num_bytes );

  for(unsigned i = 0; i < _num_bytes; i+=DATA_WIDTH) {
    SBDT val = _port_interf.out_port(_out_port).pop_data();
    unsigned cur_scratch_addr = _scratch_addr + i;
    assert(cur_scratch_addr + DATA_WIDTH <= SCRATCH_SIZE);
    std::memcpy(scratchpad.data() + cur_scratch_addr, &val, sizeof(SBDT));

    if(_sb->_ticker->in_roi()) {
      _stat_scratch_write_bytes += sizeof(SBDT);
    }

  }
  if(_sb->_ticker->in_roi() ) {
    _stat_scratch_writes +=1;
  }

}

//Write data to DMA engine
//TODO: write in vec_size? or something else?
void softsim_t::write_dma() {
  unsigned stride_size = _port_interf.out_port(_out_port).port_vec_elem();
  assert(_port_interf.out_port(_out_port).mem_size() >= _num_strides * stride_size &&
         "Data Words Ready >= Data Words Requested");

  std::cout << "Elem Requested: " << _num_strides * stride_size << "\n";
  std::cout << "stride_size: " << stride_size << "\n";

  for(unsigned i = 0; i < _num_strides * stride_size * DATA_WIDTH; i+=DATA_WIDTH) {
    SBDT val = _port_interf.out_port(_out_port).pop_data();
    addr_t cur_addr = _mem_addr + i;
    ST_MEM(cur_addr,val);
    std::cout << "m[" << i << "]=" << std::hex<< val << "\n";
  }
}

void softsim_t::reroute()  {
  unsigned stride_size = _port_interf.out_port(_out_port).port_vec_elem();
  assert(_port_interf.out_port(_out_port).mem_size() >= _num_strides * stride_size);

  if(_num_strides * stride_size > fifo_depth) {
    std::cerr << "too many recurrence vals: " 
              << _num_strides * stride_size << "\n";
  }

  for(unsigned i = 0; i < _num_strides * stride_size; i+=1) {
    SBDT val = _port_interf.out_port(_out_port).pop_data();
    _port_interf.push_data(val, _in_port); 
  }

  _port_interf.reformat_in(_in_port);
  do_cgra();
}

void softsim_t::write_constant() { //Write constants onto a port
  for(unsigned i = 0; i < _num_strides; ++i) {
    _port_interf.push_data(_constant, _in_port); 
  }

  _port_interf.reformat_in(_in_port);
  do_cgra();
}

#endif


void softsim_t::cycle_in_interf() {
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    int cur_port = _soft_config.in_ports_active[i];
    _port_interf.in_port(cur_port).reformat_in_one_vec();
  }
}

void softsim_t::cycle_out_interf() {
  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    int cur_port = _soft_config.out_ports_active[i];
    _port_interf.out_port(cur_port).reformat_out_one_vec();
  }
}


void softsim_t::cycle_cgra() {
  if(!_pdg) return;
  //do_cgra();
  //return; 

  uint64_t cur_cycle = now();

  //Detect if we are ready to fire
  unsigned min_ready=10000000;
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    int cur_port = _soft_config.in_ports_active[i];
    min_ready = std::min(_port_interf.in_port(cur_port).num_ready(), min_ready);
  }

  //Now fire on all cgra ports
  if(min_ready > 0) {
    forward_progress();
    execute_pdg(0);

    if(_ticker->in_roi()) {
      _stat_sb_insts+=_pdg->num_insts();
    }
    //pop the elements from inport as they have been processed
    for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
      _port_interf.in_port(_soft_config.in_ports_active[i]).pop(1);
    }

    //cout << "issued " << now();
    for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
       unsigned out_port_num =  _soft_config.out_ports_active[i];
       _port_interf.out_port(out_port_num).set_in_flight();
       int lat = _soft_config.out_ports_lat[out_port_num];
       //cout  << " lat: " << lat << " ";
       _cgra_output_ready[cur_cycle + lat].push_back(out_port_num);
    }

    //cout << "min_ready" << min_ready << "\n";

  }

  if(!_cgra_output_ready.empty()) {
    auto iter =  _cgra_output_ready.begin();
    if(cur_cycle >= iter->first) {
      for(auto& out_port_num : iter->second) {
        _port_interf.out_port(out_port_num).set_out_complete();
      }
      _cgra_output_ready.erase(iter); //delete from list
    }
  }
}


// DEPRICATED NON-TIMING CODE
void softsim_t::do_cgra() {
  //1. Iterate over all active input ports, find the minimum filled
  //2. For each ready instance
  //   a. Send each fifo's data to corresponding pdg input
  //   b. perform computation
  //   c. Send outputs to corresponding output fifo
  //3. Pop from CGRA inputs
  //4. Reformat data into stream

  unsigned min_ready=10000000;
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    int cur_port = _soft_config.in_ports_active[i];
    min_ready = std::min(_port_interf.in_port(cur_port).num_ready(), min_ready);

    if(min_ready == 0) {
      return;
    }
  }

#ifdef FAKE_CGRA
  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
     int cur_port = _soft_config.out_ports_active[i];
     _port_interf.out_port(cur_port).push_fake(min_ready);          //fake data to cgra out port
  }
#else
  for(unsigned instance = 0; instance < min_ready; ++instance) {      //actual computation
    forward_progress();
    execute_pdg(instance);
  }
#endif
 
  //pop the elements from inport as they have been processed
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    _port_interf.in_port(_soft_config.in_ports_active[i]).pop(min_ready);
  }

  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
     _port_interf.out_port(_soft_config.out_ports_active[i]).reformat_out();
  }

}//end do cgra



void softsim_t::execute_pdg(unsigned instance) {
  if(SB_DEBUG::SB_COMP) {
    std::cout << "inputs: ";
  }

  //send each fifo's data to corresponding pdg input
  for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    auto& cur_in_port = _port_interf.in_port(_soft_config.in_ports_active[i]);

    //for each active vector port
    for(unsigned port_idx = 0; port_idx < cur_in_port.port_cgra_elem(); ++port_idx) {
      int cgra_port = cur_in_port.cgra_port_for_index(port_idx);
      if(_soft_config.cgra_in_ports_active[cgra_port]==false) {
        continue;
      }
     
      //get the data of the instance of CGRA FIFO
      SBDT val = cur_in_port.value_of(port_idx, instance);

      //for each cgra port and associated pdg input
      _soft_config.input_pdg_node[i][port_idx]->set_value(val);  //value of the node
      
      if(SB_DEBUG::SB_COMP) {
        std::cout << std::hex << val << ", " << std::dec;
      }
      if(SB_DEBUG::VERIF_PORT) {
        in_port_verif << hex << setw(16) << setfill('0') << val << " ";
      }
    }
  }

  if(_ticker->in_roi()) {  
    for(auto i : _soft_config.inst_histo) {
     _total_histo[i.first]+=i.second;
    }
    for(unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
       unsigned cur_p = _soft_config.in_ports_active[i];
      _vport_histo[_port_interf.in_port(cur_p).port_cgra_elem()]++;
    }
    for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
       unsigned cur_p = _soft_config.out_ports_active[i];
      _vport_histo[_port_interf.out_port(cur_p).port_cgra_elem()]++;
    }
  }


  bool print = false;
  if(SB_DEBUG::SB_COMP) {
    print = true;
    std::cout <<"\n";
  }

  if(SB_DEBUG::VERIF_PORT) {
    in_port_verif << "\n";
  }

  //perform computation
  _pdg->compute(print,SB_DEBUG::VERIF_CGRA);

  //send outputs to corresponding output fifo
  for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    auto& cur_out_port = _port_interf.out_port(_soft_config.out_ports_active[i]);

    for(unsigned port_idx = 0; port_idx < cur_out_port.port_cgra_elem(); ++port_idx) {
      //int cgra_port = cur_out_port.cgra_port_for_index(port_idx);

      uint64_t val = _soft_config.output_pdg_node[i][port_idx]->retrieve();
      cur_out_port.push_val(port_idx, val);       //retreive the val from last inst

      if(SB_DEBUG::VERIF_PORT) {
        out_port_verif << hex << setw(16) << setfill('0') << val << " ";
      }

    }
    cur_out_port.inc_ready(1); //we just did one instance
  }

  if(SB_DEBUG::VERIF_PORT) {
    out_port_verif << "\n";
  }

  if(_ticker->in_roi()) {
    _stat_comp_instances += 1;  
  }
}

void softsim_t::req_config(addr_t addr, int size) {
  assert(_in_config ==false);
  _in_config=true;

  _ticker->set_in_use();

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    _ticker->timestamp();
    cout << "SB_CONFIGURE(request): " << "0x" << std::hex << addr  << " " 
                                      << std::dec << size << "\n";
  }
  SDMemReqInfoPtr sdInfo = new SDMemReqInfo(CONFIG_STREAM);

  _lsq->pushRequest(_cur_minst,true/*isLoad*/,NULL/*data*/,
              size*8/*cache line*/, addr, 0/*flags*/, 0 /*res*/,
              sdInfo);
}

// Configure once you get all the bits
void softsim_t::configure(addr_t addr, int size, uint64_t* bits) {
  //Slice 0: In Ports Activge
  //Slice 1: Out Ports Active
  //2,3,4: Reserved for delay  (4 bits each)
  //5+: switch/fu configuration
  //size - num of 64bit slices

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    _ticker->timestamp();
    cout << "SB_CONFIGURE(response): " << "0x" << std::hex << addr << " " 
                                       << std::dec << size << "\n";
    //for(int i = 0; i < size/8; ++i) {
    //  cout << "0x" << std::hex << bits[i] << " ";
    //}
    //cout << "\n";
  }

   assert(_in_config==true);
  _in_config=false;

  _soft_config.reset(); //resets to no configuration
  _port_interf.reset();

  if(_sched) {
    _sched->clear_sbpdg();
    delete _sched;
  }
  _sched = new Schedule(_sbconfig);
  //assert(_sched);
  
  for(int i = 0; i < size; ++i) { //load in 64bit slices
    _sched->slices().write(i,bits[i]);           
  }
 
  _soft_config.inst_histo = _sched->interpretConfigBits();

  _pdg=_sched->sbpdg(); //now we have the pdg!

  //Lets print it for debugging purposes
  std::ofstream ofs("viz/reconstructed.dot", std::ios::out);
  if(!ofs.good()) {
    cerr << "WARNING: viz/ folder not created\n";
  }
  _pdg->printGraphviz(ofs);

  _soft_config.out_ports_lat.resize(64); // make this bigger if we need

  _soft_config.cgra_in_ports_active.resize(128);

  // Associating the PDG Nodes from the configuration bits, with the vector ports defined by the hardware
  for(auto& port_pair : _sbconfig->subModel()->io_interf().in_vports) {
    int i = port_pair.first; //index of port
    //std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;

    if(_sched->slices().read_slice(0,i,i)) {
      _soft_config.in_ports_active.push_back(i); //activate input vector port
      port_data_t& cur_in_port = _port_interf.in_port(i);
 
      SbPDG_Vec* vec_in = _sched->vportOf(make_pair(true/*input*/,i));
      assert(vec_in);
      vector<bool> mask = _sched->maskOf(vec_in);

      cur_in_port.set_port_map(_sbconfig->subModel()->io_interf().in_vports[i],mask);

      //cout << "added ivp" << i << "mask size: " << mask.size() << "\n";

      //find corresponding pdg nodes
      std::vector<SbPDG_Input*> pdg_inputs;

      //for each mapped cgra port
      for(unsigned port_idx = 0; port_idx < cur_in_port.port_cgra_elem(); ++port_idx) {
        int cgra_port_num = cur_in_port.cgra_port_for_index(port_idx);
        
        //look up the dynode for the input cgra_port_num
        sbinput* sbin =_sbconfig->subModel()->get_input(cgra_port_num);
        SbPDG_Node* pdg_node = _sched->pdgNodeOf(sbin);
        if(pdg_node) {
          SbPDG_Input* pdg_in = static_cast<SbPDG_Input*>(pdg_node);
          pdg_inputs.push_back(pdg_in);                 //get all the pdg inputs for ports
          _soft_config.cgra_in_ports_active[cgra_port_num]=true;
        } else {
          //TODO: is there something I SHOULD DO INCASE input DOESN'T EXIST?
        }
      }
      _soft_config.input_pdg_node.push_back(pdg_inputs);
    }
  }


  for(auto& port_pair : _sbconfig->subModel()->io_interf().out_vports) {
    int i = port_pair.first; //index of port
    //std::vector<std::pair<int, std::vector<int> > >& port_m = port_pair.second;

    if(_sched->slices().read_slice(1,i,i)) { //activate output port
      _soft_config.out_ports_active.push_back(i);
      auto& cur_out_port = _port_interf.out_port(i);

      SbPDG_Vec* vec_out = _sched->vportOf(make_pair(false/*input*/,i));
      assert(vec_out);
      vector<bool> mask = _sched->maskOf(vec_out);

      //port mapping of 1 vector port - cgra_port_num: vector offset elements
      cur_out_port.set_port_map(_sbconfig->subModel()->io_interf().out_vports[i],mask);

      //find corresponding pdg nodes
      std::vector<SbPDG_Output*> pdg_outputs;

      int max_lat=0;
      for(unsigned port_idx = 0; port_idx < cur_out_port.port_cgra_elem(); ++port_idx){
        int cgra_port_num = cur_out_port.cgra_port_for_index(port_idx);
        //look up the dynode for the input cgra_port_num
        sboutput* sbout =_sbconfig->subModel()->get_output(cgra_port_num);
        SbPDG_Node* pdg_node = _sched->pdgNodeOf(sbout);
        assert(pdg_node);
        SbPDG_Output* pdg_out = static_cast<SbPDG_Output*>(pdg_node);
        max_lat=std::max(_sched->latOf(pdg_out),max_lat);
        pdg_outputs.push_back(pdg_out);
      }
      _soft_config.out_ports_lat[i]=max_lat;
      if(SB_DEBUG::SB_OVP_LAT) {
        cout << "softbrain ovp" << i << " has latency:" << max_lat << "\n";
      }
      _soft_config.output_pdg_node.push_back(pdg_outputs);
    }
  }

  ofs.close();
}




#include <unordered_set>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <assert.h>

#include "ssim.hh"
#include "../cpu.hh"

using namespace std;

// Vector-Stream Commands (all of these are context-dependent)

ssim_t::ssim_t(Minor::LSQ* lsq) :
  NUM_ACCEL(getenv("LANES") ? std::stoi(getenv("LANES")) : 8), _lsq(lsq) {

  const char *req_core_id_str = std::getenv("DBG_CORE_ID");
  if (req_core_id_str != nullptr) {
    _req_core_id = atoi(req_core_id_str);
  }

  SS_DEBUG::check_env(debug_pred());

  accel_arr.resize(NUM_ACCEL_TOTAL);
  for(int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    accel_arr[i] = new accel_t(i, this);
  }
  //accel_arr[SHARED_SP] = new accel_t(lsq, SHARED_SP, this);
  //TODO: inform accel_arr

  // set indirect ports to be 1-byte by default
  for(int i=START_IND_PORTS; i<STOP_IND_PORTS; ++i) {
    port_data_t& cur_out_port = accel_arr[0]->_port_interf.out_port(i);
    cur_out_port.set_port_width(8);

    port_data_t& cur_in_port = accel_arr[0]->_port_interf.in_port(i);
    cur_in_port.set_port_width(8);
  }

  for (int i = 0; i < DSARF::TOTAL_REG; ++i) {
    rf[i].value = REG_DEFAULT[i];
    rf[i].sticky = REG_STICKY[i];
  }

}

void ssim_t::LoadBitStream() {
  auto context = rf[DSARF::TBC].value;
  auto addr = rf[DSARF::CSA].value;
  auto size = rf[DSARF::CFS].value;

  LOG(CONFIG) << addr << " " << size;

  if(addr == 0) {
    if (size == 0) {
      LOG(COMMAND_I) << "Complete reset request issued";
      //This is the reset_data case!
      accel_arr[0]->request_reset_data();
      return;
    }
    if(size == 1) {
      LOG(COMMAND_I) << "RESET stream request triggered";
      accel_arr[0]->switch_stream_cleanup_mode_on();
      return;
    }
  }

  set_in_use();  //lets get going then..

  for (int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if ((context >> i) & 1) {
      CHECK(!accel_arr[i]->_in_config);
      accel_arr[i]->set_in_config();
    }
  }

  LOG(COMMAND) << "dsaURE(request): 0x" << std::hex << addr << " " << std::dec << size << "\n";

  SSMemReqInfoPtr sdInfo = new SSMemReqInfo(-4, context, CONFIG_STREAM);

  _lsq->pushRequest(cur_minst(), true /*isLoad*/, NULL /*data*/,
                    size * 8 /*cache line*/, addr, 0 /*flags*/, 0 /*res*/,
                    nullptr /*atom op*/, std::vector<bool>() /*byte enable*/,
                    sdInfo);

}

Minor::LSQ *ssim_t::lsq() {
  return _lsq;
}

void ssim_t::resetNonStickyState() {
  for (int i = 0; i < DSARF::TOTAL_REG; ++i) {
    if (!rf[i].sticky) {
      rf[i].value = REG_DEFAULT[i];
    }
  }
  for (int i = 0; i < DSA_MAX_IN_PORTS; ++i) {
    vps[i] = dsa::sim::IVPState();
  }
}

const std::function<LinearStream*(int word, bool is_mem, dsa::sim::ConfigState *rf)>
CONSTRUCT_LINEAR_STREAM[] = {
  [](int word, bool is_mem, dsa::sim::ConfigState *rf) -> LinearStream* {
    rf += DSARF::SAR;
    return new Linear1D(word, rf[0].value, rf[1].value, rf[2].value, is_mem);
  },
  [](int word, bool is_mem, dsa::sim::ConfigState *rf) -> LinearStream* {
    rf += DSARF::SAR;
    return new Linear2D(word,
                        rf[0].value, rf[1].value, rf[2].value,
                        rf[3].value, rf[4].value, rf[5].value,
                        is_mem);
  },
  [](int word, bool is_mem, dsa::sim::ConfigState *rf) -> LinearStream* {
    rf += DSARF::SAR;
    return new Linear3D(word,
                        rf[0].value, rf[1].value, rf[2].value, rf[3].value, rf[4].value,
                        rf[5].value, rf[6].value, rf[7].value, rf[8].value, rf[9].value,
                        rf[10].value, rf[11].value, is_mem);
  }
};

BuffetEntry *FindBuffetEntry(int begin, int end, std::vector<BuffetEntry> &bes) {
  auto iter = std::find_if(bes.begin(), bes.end(),
                           [begin, end](const BuffetEntry &be) {
                             return be.begin == begin && be.end == end; });
  if (iter == bes.end()) {
    bes.emplace_back(begin, end);
    return &(*(bes.end() - 1));
  }
  return &(*iter);
}

void ssim_t::LoadMemoryToPort(int port, int source, int dim, int padding) {
  std::vector<PortExecState> pes = gatherBroadcastPorts(port);
  assert(dim < 3);
  auto addressable = (1 << (rf[DSARF::CSR].value & 3)) * spec.memory_addressable;
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, true, rf);
  auto* s = new LinearReadStream(source == 0 ? LOC::DMA : LOC::SCR, ls, pes, padding);
  if (rf[DSARF::BR].value != -1) {
    int begin = rf[DSARF::BR].value & 65535, end = (rf[DSARF::BR].value >> 16) & 65535;
    s->be = FindBuffetEntry(begin, end, bes);
    s->be->use = s;
    LOG(BUFFET) << "Allocate Buffet: " << s->be->toString();
  }
  add_bitmask_stream(s);
}

void ssim_t::WritePortToMemory(int port, int operation, int dst, int dim) {
  assert(dim < 3);
  auto addressable = (1 << (rf[DSARF::CSR].value & 3)) * spec.memory_addressable;
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, true, rf);
  auto unit = (dst == 0) ? LOC::DMA : LOC::SCR;
  if (operation != 1) {
    CHECK(unit == LOC::SCR) << "Only scratch pad supports in-situ computing!";
  }
  auto* s = new LinearWriteStream(unit, ls, port, operation);
  if (rf[DSARF::BR].value != -1) {
    int begin = rf[DSARF::BR].value & 65535, end = (rf[DSARF::BR].value >> 16) & 65535;
    s->be = FindBuffetEntry(begin, end, bes);
    s->be->load = s;
    LOG(BUFFET) << "Allocate Buffet: " << s->be->toString();
  }
  add_bitmask_stream(s);
}


void ssim_t::ConstStream(int port, int dim) {
  CHECK(dim < 3);
  auto addressable = (1 << (rf[DSARF::CSR].value & 3)) * spec.memory_addressable;
  auto dtype = (1 << ((rf[DSARF::CSR].value >> 2) & 3)) * spec.memory_addressable;
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, false, rf);
  auto s = new ConstPortStream(gatherBroadcastPorts(port), ls);
  s->set_data_width(dtype);
  add_bitmask_stream(s);
}


void ssim_t::IndirectMemoryToPort(int port, int source, int ind, int lin) {
  // TODO(@were): Use the linear and indirect flags!
  auto irs =
    new IndirectReadStream(source == 0 ? LOC::DMA : LOC::SCR,
                           gatherBroadcastPorts(port), rf[DSARF::INDP].value & 127,
                           rf[DSARF::SAR].value, rf[DSARF::L1D].value);
  irs->set_data_width(1 << ((rf[DSARF::CSR].value >> 4) & 3));
  add_bitmask_stream(irs);
}

std::vector<PortExecState> ssim_t::gatherBroadcastPorts(int port) {
  std::vector<PortExecState> pes;
  for (int i = 0; i < DSA_MAX_IN_PORTS; ++i) {
    if (vps[i].broadcast || i == port) {
      pes.emplace_back(vps[i], i);
      LOG(PORTS) << pes.back().toString();
    }
  }
  return pes;
}

int64_t ssim_t::CurrentCycle() {
  return now() - _orig_stat_start_cycle;
}

//The reroute function handles either local recurrence, or remote data transfer
void ssim_t::Reroute(int oport, int iport) {
  // TODO(@were): Strided padding.
  // TODO(@were): Inter-lane rerouting.
  auto ips = gatherBroadcastPorts(iport);
  int dtype = (1 << (rf[DSARF::CSR].value & 3)) * spec.memory_addressable;
  auto n = rf[DSARF::L1D].value;
  auto pps = new RecurrentStream(dtype, oport, ips, n);
  add_bitmask_stream(pps);
}

// receive network message at the given input port id
void ssim_t::push_in_accel_port(int accel_id, int8_t* val, int num_bytes, int in_port) {
  assert(accel_id<NUM_ACCEL_TOTAL);
  accel_arr[accel_id]->receive_message(val, num_bytes, in_port);
}

// SPU has only 1 DGRA in a core
void ssim_t::push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc) {
  accel_arr[0]->push_atomic_update_req(scr_addr, opcode, val_bytes, out_bytes, inc);
}

void ssim_t::push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    accel_arr[0]->push_ind_rem_read_req(is_remote, req_core, request_ptr, addr, data_bytes, reorder_entry);
}

void ssim_t::push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    accel_arr[0]->push_ind_rem_read_data(data, request_ptr, addr, data_bytes, reorder_entry);
}

bool ssim_t::can_add_stream() {
  auto context = rf[DSARF::TBC].value;
  for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i,b<<=1) {
    if(context & b) {
      if(!accel_arr[i]->can_add_stream()) {
        return false;
      }
    }
  }
  return true;
}

bool ssim_t::done(bool show, int mask) {
  bool is_done=true;
  auto context = rf[DSARF::TBC].value;
  for(int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if(context >> i & 1) {
      is_done &= accel_arr[i]->done(show, mask);
    }
  }
  return is_done;
}

bool ssim_t::is_in_config() {
  bool in_config=true;
  auto context = rf[DSARF::TBC].value;
  for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
    if(context & b) {
      in_config &= accel_arr[i]->is_in_config();
    }
  }
  return in_config;
}

void ssim_t::cycle_shared_busses() {
}

void ssim_t::step() {
  if (!_in_use) {
    return;
  }
  cycle_shared_busses();
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      accel_arr[i]->tick();
    }
  }
  shared_acc()->tick();
}

void ssim_t::print_stats() {
   auto& out = std::cout;
   out.precision(4);
   out << dec;

   out << "\n*** ROI STATISTICS for CORE ID: " << _lsq->getCpuId() << " ***\n";
   out << "Simulator Time: " << ((double)elpased_time_in_roi())
                                    / 1000000000 << " sec\n";

   out << "Cycles: " << roi_cycles() << "\n";
   out << "Number of coalesced SPU requests: " << accel_arr[0]->_stat_num_spu_req_coalesced << "\n";
   out << "Control Core Insts Issued: " << control_core_insts() << "\n";
   out << "Control Core Discarded Insts Issued: " << control_core_discarded_insts() << "\n";
   out << "Control Core Discarded Inst Stalls: " << ((double)control_core_discarded_insts()/(double)control_core_insts()) << "\n";
   // out << "Control Core Bubble Insts Issued: " << control_core_bubble_insts() << "\n";
   out << "Control Core Config Stalls: "
       << ((double)config_waits()/roi_cycles()) << "\n";
   out << "Control Core Wait Stalls:   ";
   for(auto& i : _wait_map) {
     out << ((double)i.second/roi_cycles()) << " (";
     uint64_t mask = i.first;
     if(mask & GLOBAL_WAIT) {
       out << "GLOBAL_WAIT";
     }
     if(mask == 0) {
       out << "ALL";
     }
     if(mask & WAIT_SCR_RD) {
       out << "SCR_RD";
     }
     if(mask & WAIT_SCR_WR) {
       out << "SCR_WR";
     }
     if(mask & STREAM_WAIT) {
       out << "STREAM_WAIT";
     }
     out << ")  ";
   }
   out << "\n";

  int cores_used=0;
  for (int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if (_ever_used_bitmask >> i & 1) {
      accel_arr[i]->print_stats();
      if(i != NUM_ACCEL_TOTAL) {
        cores_used++;
      }
    }
  }

  std::cout << "\nCores used: " << cores_used << "bitmask: "
            << std::hex << _ever_used_bitmask << std::dec << "\n";

  uint64_t total_mem_accesses=0;
  uint64_t max_comp_instances=0;
  for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      total_mem_accesses+=
        accel_arr[i]->_bw_map[make_pair(LOC::DMA,LOC::TOTAL)].first;
      max_comp_instances=std::max(max_comp_instances,
         accel_arr[i]->_stat_comp_instances);
    }
  }

  std::cout << "Total atomic updates: " << accel_arr[0]->_stat_tot_updates << std::endl;

  std::cout << "Total Memory Activity: " << (double) total_mem_accesses / roi_cycles() << std::endl;

  std::cout << "\n -- Parallelization Estimates --\n";

  std::cout << "Multi-Pipeline Cycles (cores:cycles): ";
  for(int i = 1; i * cores_used <= 18; ++i) { // estimate performance
    std::cout << i*cores_used << ":";
    uint64_t multicore_cyc = roi_cycles() / i;
    uint64_t mem_cyc = total_mem_accesses;
    std::cout << std::max(multicore_cyc,mem_cyc) << " ";
  }
  std::cout << "\n";

  std::cout << "Multi-Pipeline Activity: (cores:cycles)";
  for(int i = 1; i * cores_used <= 18; ++i) { // estimate performance
    std::cout << i*cores_used << ":";
    uint64_t multicore_cyc = roi_cycles() / i;
    uint64_t mem_cyc = total_mem_accesses;
    uint64_t cycles = std::max(multicore_cyc,mem_cyc);
    double active_ratio = ((double)(max_comp_instances)/i)/((double)cycles);
    std::cout << active_ratio << " ";
  }
  std::cout << "\n" << std::endl;


}

uint64_t ssim_t::forward_progress_cycle() {
  uint64_t r = _global_progress_cycle;
  for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      r = std::max(accel_arr[i]->forward_progress_cycle(),r);
    }
  }
  return r;
}

void ssim_t::set_memory_map_config(base_stream_t* s, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type) {
  if(partition_size!=0) {
    s->set_part_size(partition_size);
    s->set_active_core_bv(active_core_bitvector);
    s->set_mapping_type(mapping_type);
    for(int i=0; i<64; ++i) {
      if((active_core_bitvector >> i) & 1) {
        s->push_used_core(i);
        // std::cout << "number of used cores identifed as i: " << i << "\n";
      }
    }
    s->set_dist_cores();
  }
}

void ssim_t::add_bitmask_stream(base_stream_t* s, uint64_t ctx) {
  //patch with implicit stuff
  s->set_context_offset(_context_offset);
  s->set_mem_map_config();

  //Check if not active!
  if(debug && (SS_DEBUG::COMMAND)) {
    timestamp_context();
    std::cout << "id:" << s->id() << " ";
    s->print_status();
  }

  if(!s->stream_active()) {
    if(debug && (SS_DEBUG::COMMAND)  ) {
      timestamp_context();
      std::cout << " ---    and this stream is being deleted for being inactive!\n";
    }
    delete s;
    return;
  }

  shared_ptr<base_stream_t> s_shr(s);

  for(uint64_t i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if(ctx >> i & 1) {
      if(auto is = dynamic_cast<IPortStream*>(s)) {
        CHECK(!is->pes.empty());
        auto& in_vp = accel_arr[i]->port_interf().in_port(is->pes[0].port);
        s->set_data_width(in_vp.get_port_width()); // added for dgra
      } else if (s->out_port() != -1) {
        auto& out_vp = accel_arr[i]->port_interf().out_port(s->out_port());
        s->set_data_width(out_vp.get_port_width()); // added for dgra
      } else {
        s->set_data_width(8);
      }
      accel_arr[i]->add_stream(s_shr);
    }
  }

}

void ssim_t::add_bitmask_stream(base_stream_t* s) {
  auto context = rf[DSARF::TBC].value;
  add_bitmask_stream(s, context);
}


// ------------------------Time Stuffs-------------------------------------
void ssim_t::timestamp() {
  std::cout << std::dec << now() - _orig_stat_start_cycle << "\t";
}

void ssim_t::timestamp_index(int i) {
  timestamp();
  std::cout << "context " << i << " ";
}

void ssim_t::timestamp_context() {
  timestamp();
  auto context = rf[DSARF::TBC].value;

  std::cout << "context ";
  for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
    if(context & b) {
      std::cout << i <<" ";
    }
  }
  std::cout << "\t";
}

// ----------------------- STREAM COMMANDS ------------------------------------
void ssim_t::set_context(uint64_t context, uint64_t offset) {
  if(debug && (SS_DEBUG::CONTEXT)  ) {
    std::cout << "Set Context: " << std::hex << context << std::dec << "\n";
  }

  _context_offset = offset;
  _ever_used_bitmask |= context;
}

// it is not a stream; just a linear write (just push data into buf?)
void ssim_t::write_remote_banked_scratchpad(uint8_t* val, int num_bytes, uint16_t scr_addr) {
  // TODO: add a check for full buffer to apply backpressure to network
  accel_arr[0]->push_scratch_remote_buf(val, num_bytes, scr_addr); // hopefully, we use single accel per CC
}

void ssim_t::atomic_update_hardware_config(int addr_port, int val_port, int out_port) {

    // set up the associated stream here
  accel_arr[0]->_scr_w_c.set_atomic_cgra_addr_port(addr_port);
  accel_arr[0]->_scr_w_c.set_atomic_cgra_val_port(val_port);
  accel_arr[0]->_scr_w_c.set_atomic_cgra_out_port(out_port);
  accel_arr[0]->_scr_w_c.set_atomic_addr_bytes(2); // get_bytes_from_type(addr_type));

  port_data_t& value_port = accel_arr[0]->_port_interf.in_port(val_port);
  accel_arr[0]->_scr_w_c.set_atomic_val_bytes(value_port.get_port_width());
  // std::cout << " Addr port: " << addr_port << " val port: " << val_port << " out port: " << out_port << "\n";

}

// command decode for atomic stream update
void ssim_t::atomic_update_scratchpad(uint64_t offset, uint64_t iters, int addr_port, int inc_port, int value_type, int output_type, int addr_type, int opcode, int val_num, int num_updates, bool is_update_cnt_port, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type) {
    atomic_scr_stream_t* s = new atomic_scr_stream_t();
    
    s->_mem_addr = offset;
    s->_num_strides = iters;
    s->_out_port = addr_port;
    s->_val_port = inc_port;
    s->_op_code = opcode;
    s->_value_type=value_type;
    s->_output_type=output_type;
    s->_addr_type=addr_type;

    if(val_num!=0) { // no config specified
      s->_val_num=val_num;
      s->_sstream_left=val_num;
      s->_num_updates=num_updates;
      s->_val_sstream_left=num_updates;
      s->_is_update_cnt_port=is_update_cnt_port;
      if(is_update_cnt_port) {
        s->_num_update_port = num_updates;
        s->_num_updates=-1;
        s->_val_sstream_left=-1;
      }
    }

    // std::cout << "Atomic scr initialized with sstream size: " << val_num << "\n";

    s->set_orig();

    set_memory_map_config(s, partition_size, active_core_bitvector, mapping_type);
    add_bitmask_stream(s);

}

// TODO: make it neater
void ssim_t::multicast_remote_port(uint64_t num_elem, uint64_t mask, int out_port, int rem_port, bool dest_flag, bool spad_type, int64_t stride, int64_t access_size) {
    // remote_port_multicast_stream_t* s = NULL;
    // 0 means port->port stream
    if (dest_flag==0) {
      remote_port_multicast_stream_t* s = new remote_port_multicast_stream_t();
      s->_core_mask = mask;
      s->_num_elements = num_elem;
      s->_out_port = out_port;
      s->_remote_port = rem_port;
      // s->_unit=LOC::SCR; (probably add a flag for the destination to save a new opcode)
      s->set_orig();
      if(SS_DEBUG::NET_REQ){
        printf("Remote stream initialized");
        s->print_status();
      }
       add_bitmask_stream(s);
      } else {
        if(rem_port!=0) { // I hope it can never be 0
          remote_scr_stream_t* s = new remote_scr_stream_t();
          s->_num_elements = num_elem;
          s->_core_mask = -1;
          s->_out_port = out_port;
          s->_remote_port = -1;
          s->_addr_port = rem_port;
          s->_remote_scr_base_addr = mask; // this would now be scratch_base_addr
          s->_scr_type = spad_type;
          s->set_orig();
          add_bitmask_stream(s);
        } else { // inherited by the affine_base_stream
          direct_remote_scr_stream_t* s = new direct_remote_scr_stream_t(mask, access_size, stride);
          s->_scr_type = 0; // spad_type; // this should not be required now?
          s->_num_elements = num_elem; // this is num_strides
          s->_out_port = out_port;
          s->set_orig();
          add_bitmask_stream(s);
        }
    }
}

// Configure an indirect stream with params
// void ssim_t::indirect(
//     int ind_port, int ind_type, int in_port, addr_t index_addr,
//     uint64_t num_elem, int repeat, int repeat_str, uint64_t offset_list,
//     int dtype, uint64_t ind_mult, bool scratch, bool is_2d_stream, int sstride, int sacc_size, int sn_port, int val_num, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type) {
//   indirect_stream_t* s = new indirect_stream_t();
//   s->_ind_port=ind_port;
//   s->_ind_type=ind_type;
//   s->add_in_port(in_port);
//   s->_index_addr=index_addr; // so this is the offset
//   s->_num_elements=num_elem;
//   s->_repeat_in=repeat;
//   s->_repeat_str=repeat_str;
//   s->_offset_list=offset_list;
//   s->_dtype=dtype;
//   s->_ind_mult=ind_mult;
//   s->_is_2d_stream=is_2d_stream;
//   s->_val_num=val_num;
//   s->_ind_mult *= val_num; // for wide accesses (real mult requires more bits)
//   if(is_2d_stream) { // set sub-stream parameters here (read from ss_stride)
//     s->_sstride = sstride;
//     s->_sacc_size = sacc_size;
//     s->_sn_port = sn_port;
//   }
// 
//   // std::cout << "Came here for part size: " << partition_size << " bitvector: " << active_core_bitvector << " mapping type: " << mapping_type << "\n";
// 
//   if(scratch) s->_unit=LOC::SCR;
//   else s->_unit=LOC::DMA;
//   s->set_orig();
// 
//   set_memory_map_config(s, partition_size, active_core_bitvector, mapping_type);
//   add_bitmask_stream(s);
// 
// }

//Configure an indirect stream with params
void ssim_t::indirect_write(int ind_port, int ind_type, int out_port,
    addr_t index_addr, uint64_t num_elem, uint64_t offset_list,
    int dtype, uint64_t ind_mult, bool scratch, bool is_2d_stream, int sstride, int sacc_size, int sn_port, int val_num) {


  if(SS_DEBUG::MEM_WR || SS_DEBUG::SCR_ACC) {
    std::cout << "Identified an indirect write stream, scratch? " << scratch << " 2d: " << is_2d_stream << " num_elem port: " << sn_port << " stride: " << sstride << " access size: " << sacc_size << "\n";
  }

  indirect_wr_stream_t* s = new indirect_wr_stream_t();
  s->_ind_port=ind_port;
  s->_ind_type=ind_type;
  s->_out_port=out_port; //comes from sb_dma_addr
  s->_index_addr=index_addr;
  s->_num_elements=num_elem;
  s->_dtype=dtype;
  s->_ind_mult=ind_mult;

  s->_is_2d_stream=is_2d_stream;
  s->_val_num=val_num;
  s->_ind_mult *= val_num; // for wide accesses (real mult requires more bits)
  if(is_2d_stream) { // set sub-stream parameters here (read from ss_stride)
    s->_sstride = sstride;
    s->_sacc_size = sacc_size;
    s->_sn_port = sn_port;
  }

  if(scratch) s->_unit=LOC::SCR;
  else s->_unit=LOC::DMA;
  s->set_orig();

  add_bitmask_stream(s);
}

// -------------------------------------------------------------------------------
bool ssim_t::CanReceive(int imm) {
  imm >>= 1;
  int dtype = imm & 3;
  imm >>= 2;
  imm >>= 1;
  int port = imm;
  dtype = 1 << dtype;
  return CanReceive(port, dtype);
}

//These two functions just return the first core from 0
bool ssim_t::CanReceive(int port, int dtype) {
  auto context = rf[DSARF::TBC].value;
  CHECK((context & -context) == context)
    << "More than one accelerator to receive!";
  CHECK(context) << "No accelerator to receive";
  for(int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if(context >> i & 1) {
      auto &ovp = accel_arr[i]->port_interf().out_port(port);
      CHECK(dtype % ovp.get_port_width() == 0);
      if (ovp.stream) {
        return false;
      }
      return dtype / ovp.get_port_width() <= ovp.mem_size();
    }
  }
  return false;
}

uint64_t ssim_t::Receive(int port, int dtype) {
  auto context = rf[DSARF::TBC].value;
  CHECK((context & -context) == context)
    << "More than one accelerator to receive!";
  CHECK(context) << "No accelerator to receive";
  for (int i = 0; i < NUM_ACCEL_TOTAL; ++i) {
    if(context >> i & 1) {
      port_data_t &out_vp = accel_arr[i]->port_interf().out_port(port);
      CHECK(dtype % out_vp.get_port_width() == 0);
      std::vector<uint8_t> raw;
      for (int i = 0; i < dtype; i += out_vp.get_port_width()) {
        auto val = out_vp.pop_out_data();
        auto *ptr = (uint8_t*)&val;
        raw.insert(raw.end(), ptr, ptr + out_vp.get_port_width());
      }
      CHECK(raw.size() <= 8);
      raw.resize(8, 0);
      auto res = *reinterpret_cast<uint64_t*>(&raw[0]);
      LOG(RECV)
        << "SS_RECV value: " << res
        << " on port" << out_vp.port() << " " << out_vp.mem_size()
        << " on core: " << lsq()->getCpuId();
      return res;
    }
  }
  return -1;
}


void ssim_t::InsertBarrier(uint64_t mask) {
  Barrier* s = new Barrier();
  s->_mask = mask;
  s->set_orig();
  add_bitmask_stream(s);
}

// TODO:FIXME: see if we need new mask
void ssim_t::insert_df_barrier(int64_t num_scr_wr, bool spad_type) {
  Barrier* s = new Barrier();
  s->_mask = 64;
  // s->_num_remote_writes = num_scr_wr;
  s->_scr_type = spad_type; // TODO: Where to use this?
  s->set_orig();
  // printf("df count read from register is: %ld\n",num_scr_wr);
  accel_arr[0]->_scr_w_c.set_df_count(num_scr_wr);
  add_bitmask_stream(s);
}

uint64_t ssim_t::now() {
  return curTick();
}

void ssim_t::update_stat_cycle() {
  assert(in_roi());
  if (_stat_start_cycle == ~0ull) {
    _stat_start_cycle = now();
  } else {
    _stat_stop_cycle = now();
  }
}

void ssim_t::setup_stat_cycle() {
  _stat_start_cycle = ~0ull;
  _roi_enter_cycle = now();
}

void ssim_t::cleanup_stat_cycle() {
  if (_stat_start_cycle == ~0ull) {
    _stat_start_cycle = _roi_enter_cycle;
    _stat_stop_cycle = now();
  }
}

// ------------------------- TIMING ---------------------------------
void ssim_t::roi_entry(bool enter) {
  if(enter) {
    if(debug && (SS_DEBUG::COMMAND || SS_DEBUG::ROI)  ) {
      timestamp();
      cout << "Entering ROI ------------\n";
    }

    if(_orig_stat_start_cycle == 0) {
      _orig_stat_start_cycle = now();
    }
    setup_stat_cycle();
    clock_gettime(CLOCK_REALTIME,&_start_ts);
    _in_roi=true;
    _times_roi_entered+=1;
  } else {
    clock_gettime(CLOCK_REALTIME,&_stop_ts);
    _elapsed_time_in_roi += 1000000000 * (_stop_ts.tv_sec - _start_ts.tv_sec) +
                                          _stop_ts.tv_nsec - _start_ts.tv_nsec;

    if(debug && (SS_DEBUG::COMMAND || SS_DEBUG::ROI)) {
      timestamp();
      std::cout << "Exiting ROI ------------";
      std::cout << "(" << _stat_start_cycle << "to" << _stat_stop_cycle << ")\n";
    }

    _roi_cycles += _stat_stop_cycle - _stat_start_cycle;
    _in_roi=false;
  }
}

void ssim_t::instantiate_buffet(int repeat, int repeat_str) {
  // int start = (stream_stack[0] >> 32) & 0xFFFFFFFF;
  // int buffer_size = stream_stack[0] & 0xFFFFFFFF;
  // uint64_t total = stream_stack[1];
  // int src_port = stream_stack[2];

  // stream_stack.erase(stream_stack.begin(), stream_stack.begin() + 3);

  // int shadow_port = stream_stack[stream_stack.size() - 3] & 31;
  // int shadow_dtype = stream_stack[stream_stack.size() - 3] >> 5 & 7;
  // if(shadow_dtype) {
  //   //case 0: width=8;
  //   //case 1: width=4;
  //   //case 2: width=2;
  //   //case 3: width=1;
  //   shadow_dtype = 1 << (3 - (shadow_dtype - 1));
  // } else { // assume the width of the corresponding input port
  //   port_data_t& cur_out_port = accel_arr[0]->_port_interf.out_port(shadow_port);
  //   shadow_dtype = cur_out_port.get_port_width();
  // }

  // int in_port = stream_stack[stream_stack.size() - 1];
  // int inport_dtype = stream_stack[stream_stack.size() - 3] >> 8 & 7;
  // if(inport_dtype) {
  //   //case 0: width=8;
  //   //case 1: width=4;
  //   //case 2: width=2;
  //   //case 3: width=1;
  //   inport_dtype = 1 << (3 - (inport_dtype - 1));
  //   port_data_t& cur_out_port = accel_arr[0]->_port_interf.in_port(in_port);
  //   assert(inport_dtype % cur_out_port.get_port_width() == 0);
  // } else { // assume the width of the corresponding input port
  //   port_data_t& cur_out_port = accel_arr[0]->_port_interf.in_port(in_port);
  //   inport_dtype = cur_out_port.get_port_width();
  // }

  // stream_stack[stream_stack.size() - 3] = 0;
  // BuffetStream *buffet = new BuffetStream(start, buffer_size, src_port, total, LOC::SCR,
  //                                         stream_stack, {in_port}, repeat, repeat_str,
  //                                         shadow_port, shadow_dtype, inport_dtype);

  // add_bitmask_stream(buffet);
  // stream_stack.clear();
}

int ssim_t::get_bytes_from_type(int t) {
  switch(t) {
    case T64: return 8;
    case T32: return 4;
    case T16: return 2;
    case T08: return 1;
    default: assert(0);
  }
}

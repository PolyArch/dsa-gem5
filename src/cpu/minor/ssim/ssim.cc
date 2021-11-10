#include <unordered_set>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <assert.h>

#include "json/json.h"
#include "dsa/core/utils.h"
#include "json/value.h"
#include "ssim.hh"
#include "../cpu.hh"
#include "../exec_context.hh"
#include "./ism.h"

using namespace std;

// Vector-Stream Commands (all of these are context-dependent)

ssim_t::ssim_t(Minor::LSQ *lsq_) : lsq_(lsq_), statistics(*this) {

  const char *req_core_id_str = std::getenv("DBG_CORE_ID");
  if (req_core_id_str != nullptr) {
    _req_core_id = atoi(req_core_id_str);
  }

  lanes.resize(spec.num_of_lanes + 1);
  for(int i = 0; i < (int) lanes.size(); ++i) {
    lanes[i] = new accel_t(i, this);
  }
  //lanes[SHARED_SP] = new accel_t(lsq, SHARED_SP, this);
  //TODO: inform lanes

  // set indirect ports to be 1-byte by default
  for (int i = 0; i < DSARF::TOTAL_REG; ++i) {
    rf[i].value = REG_DEFAULT[i];
    rf[i].sticky = REG_STICKY[i];
  }

  if (std::getenv("DSA_SPEC")) {
    std::string dsa_spec(std::getenv("DSA_SPEC"));
    Json::Value raw_json = dsa::core::utils::LoadJsonFromFile(dsa_spec);
    #define SPEC_ATTR(TY, ID, VAL) if (raw_json.isMember(#ID)) { spec.ID = raw_json.as<TY>(); }
    #include "dsa-ext/spec.attr"
    #undef SPEC_ATTR
  }

}

void ssim_t::LoadBitstream() {
  auto context = rf[DSARF::TBC].value;
  auto addr = rf[DSARF::CSA].value;
  auto size = rf[DSARF::CFS].value;

  DSA_LOG(CONFIG) << addr << " " << size;

  if(addr == 0) {
    if (size == 0) {
      DSA_LOG(COMMAND_I) << "Complete reset request issued";
      //This is the reset_data case!
      lanes[0]->request_reset_data();
      return;
    }
    if(size == 1) {
      DSA_LOG(COMMAND_I) << "RESET stream request triggered";
      lanes[0]->switch_stream_cleanup_mode_on();
      return;
    }
  }

  set_in_use();  //lets get going then.

  for (int i = 0; i < (int) lanes.size(); ++i) {
    if ((context >> i) & 1) {
      CHECK(lanes[i]->statistics.blame != dsa::stat::Accelerator::CONFIGURE) << i;
      lanes[i]->statistics.blame = dsa::stat::Accelerator::CONFIGURE;
    }
  }

  DSA_LOG(COMMAND)
    << now() << ": Request 0x" << std::hex << addr
    << ", " << std::dec << size << " to configure";

  SSMemReqInfoPtr sdInfo = new SSMemReqInfo(-4, context, CONFIG_STREAM);

  lsq()->pushRequest(inst, true /*isLoad*/, NULL /*data*/,
                    size * 8 /*cache line*/, addr, 0 /*flags*/, 0 /*res*/,
                    nullptr /*atom op*/, std::vector<bool>() /*byte enable*/,
                    sdInfo);

}

Minor::LSQ *ssim_t::lsq() {
  return lsq_;
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
  ++statistics.ctrl_intrinsics;
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

void ssim_t::LoadMemoryToPort(int port, int source, int dim, int padding) {
  std::vector<PortExecState> pes = gatherBroadcastPorts(port);
  assert(dim < 3);
  auto addressable = 1 << (rf[DSARF::CSR].value & 3);
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, true, rf);
  auto* s = new LinearReadStream(source == 0 ? LOC::DMA : LOC::SCR, rf[DSARF::TBC].value,
                                 ls, pes, padding, rf[DSARF::BMSS].value);
  s->dtype = s->ls->word_bytes();
  s->inst = inst;
  if (rf[DSARF::BR].value != -1) {
    int begin = rf[DSARF::BR].value & 65535, end = (rf[DSARF::BR].value >> 16) & 65535;
    RegisterBuffet(s, begin, end);
  }
  DSA_LOG(COMMAND) << now() << ": Linear Read Stream: " << s->toString();
  cmd_queue.push_back(s);
}

void ssim_t::WritePortToMemory(int port, int operation, int dst, int dim) {
  assert(dim < 3);
  auto addressable = (1 << (rf[DSARF::CSR].value & 3));
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, true, rf);
  auto unit = (dst == 0) ? LOC::DMA : LOC::SCR;
  if (operation != 1) {
    CHECK(unit == LOC::SCR) << "Only scratch pad supports in-situ computing!";
  }
  auto* s = new LinearWriteStream(unit, rf[DSARF::TBC].value, ls, port, operation);
  s->dtype = s->ls->word_bytes();
  s->inst = inst;
  if (rf[DSARF::BR].value != -1) {
    int begin = rf[DSARF::BR].value & 65535, end = (rf[DSARF::BR].value >> 16) & 65535;
    RegisterBuffet(s, begin, end);
  }
  DSA_LOG(COMMAND) << now() << ": Linear Write Stream: " << s->toString();
  cmd_queue.push_back(s);
}


void ssim_t::ConstStream(int port, int dim) {
  CHECK(dim < 3);
  auto addressable = (1 << (rf[DSARF::CSR].value & 3));
  auto dtype = (1 << ((rf[DSARF::CSR].value >> 2) & 3));
  LinearStream *ls = CONSTRUCT_LINEAR_STREAM[dim](addressable, false, rf);
  auto s = new ConstPortStream(rf[DSARF::TBC].value, gatherBroadcastPorts(port), ls);
  s->dtype = dtype;
  s->inst = inst;
  DSA_LOG(COMMAND) << now() << ": Const Stream: " << s->toString();
  cmd_queue.push_back(s);
}

dsa::sim::stream::IndirectFSM
initializeIndirectFSM(dsa::sim::ConfigState *rf, int ind, int dim, int penetrate, int associate,
                      const std::vector<int> &PORT_SHIFT,
                      const std::vector<int> &TYPE_SHIFT) {
  CHECK(PORT_SHIFT.size() == TYPE_SHIFT.size());
  auto ctx = rf[DSARF::TBC].value;
  using Attr = dsa::sim::stream::IndirectFSM::FSMAttr;
  dsa::sim::stream::IndirectFSM fsm(dim + 1, rf[DSARF::SAR].value, rf[DSARF::E2D].value,
                                    rf[DSARF::I2D].value, rf[DSARF::I1D].value,
                                    penetrate, associate);
  for (int i = 0; i < PORT_SHIFT.size(); ++i) {
    fsm.attrs.emplace_back();
    if ((ind >> i) & 1) {
      fsm.attrs[i].port = (rf[DSARF::INDP].value >> PORT_SHIFT[i]) & 127;
    }
    fsm.attrs[i].dtype =  1 << ((rf[DSARF::CSR].value >> TYPE_SHIFT[i]) & 3);
  }
  // Make the inner-most stream a dummy and empty stream,
  // which forces the outer dimensions to refresh.
  fsm.idx().stream = new SentinelStream(false);

  // If 2D, we use L2D. O.w, we use 1.
  int64_t outer = dim == 1 ? rf[DSARF::L2D].value : (int64_t)1;
  auto fPortOrConst = [ctx, outer, dim](Attr &attr, int64_t const_value) {
    if (attr.port == -1) {
      auto l1d = new Linear1D(attr.dtype, const_value, 0, outer, 0);
      attr.stream = new ConstPortStream(ctx, {}, l1d);
      attr.stream->dtype = attr.dtype;
    } else {
      CHECK(dim == 1) << "Only 2D stream can have inner dimension from a port!";
      attr.stream = new RecurrentStream(ctx, attr.port, {}, outer);
    }
  };
  fPortOrConst(fsm.length(), rf[DSARF::L1D].value);
  fPortOrConst(fsm.array(), 0);
  return fsm;
}

void ssim_t::IndirectMemoryToPort(int port, int source, int ind, int dim, bool penetrate, bool associate) {
  auto ctx = rf[DSARF::TBC].value;
  // INDP[0:7]: Index Port; INDP[14:20]: Start Address; INDP[21:27]: Length
  //  CSR[4:5]: Index Port;    CSR[8:9]: Start Address;  CSR[10:11]: Length
  auto fsm = initializeIndirectFSM(rf, ind, dim, penetrate, associate, {0, 7, 14}, {4, 8, 10});
  fsm.attrs.emplace_back();
  fsm.value().stream = new SentinelStream(false);
  CHECK(fsm.attrs.size() == 4);
  auto ports = gatherBroadcastPorts(port);
  auto irs = new IndirectReadStream(source == 0 ? LOC::DMA : LOC::SCR, ctx, ports, fsm);
  irs->bmss = rf[DSARF::BMSS].value;
  irs->dtype = (1 << ((rf[DSARF::CSR].value >> 0) & 3));
  irs->inst = inst;
  cmd_queue.push_back(irs);
  DSA_LOG(COMMAND) << now() << ": Indirect Read Stream: " << irs->toString();
}

void ssim_t::AtomicMemoryOperation(int port, int mem, int operation, int ind, int dim, bool penetrate, bool associate) {
  auto ctx = rf[DSARF::TBC].value;
  // INDP[0:7]: Index Port; INDP[14:20]: Start Address; INDP[21:27]: Length;
  //  CSR[4:5]: Index Port;    CSR[8:9]: Start Address;  CSR[10:11]: Length;
  int dtype = 1 << ((rf[DSARF::CSR].value >> 0) & 3);
  auto fsm = initializeIndirectFSM(rf, ind | 8, dim, penetrate, associate, {0, 7, 14}, {4, 8, 10});
  fsm.attrs.emplace_back();
  fsm.value().port = port;
  fsm.value().stream = new SentinelStream(false);
  fsm.value().dtype = dtype;
  auto ias = new IndirectAtomicStream(mem == 0 ? LOC::DMA : LOC::SCR, ctx, fsm);
  ias->dtype = dtype;
  ias->inst = inst;
  ias->mo = (MemoryOperation) operation;
  cmd_queue.push_back(ias);
  DSA_LOG(COMMAND) << now() << ": Indirect Write Stream: " << ias->toString();
}

std::vector<PortExecState> ssim_t::gatherBroadcastPorts(int port) {
  std::vector<PortExecState> pes;
  for (int i = 0; i < DSA_MAX_IN_PORTS; ++i) {
    if (vps[i].broadcast || i == port) {
      pes.emplace_back(vps[i], i);
      DSA_LOG(PORTS) << pes.back().toString();
    }
  }
  return pes;
}

int64_t ssim_t::CurrentCycle() {
  return now() - _orig_stat_start_cycle;
}

void ssim_t::DispatchStream() {

  struct DispatchChecker : dsa::sim::stream::Functor {
    DispatchChecker(accel_t *accel_) : accel(accel_) {}

    /*!
     * \brief Check the avaiability of the ports of this stream.
     * \param is The input stream to be scheduled.
     */
    bool IVPAvailable(const std::vector<PortExecState> &pes) {
      for (auto &elem : pes) {
        auto &in = accel->input_ports[elem.port];
        if (in.stream) {
          DSA_LOG(CMD_BLOCK) << "ivp" << elem.port << " serving stream " << in.stream->toString();
          return false;
        }
        if (std::find(iports.begin(), iports.end(), elem.port) != iports.end()) {
          DSA_LOG(CMD_BLOCK) << "ivp" << elem.port << " is enforced by previous conflict port!";
          return false;
        }
        if (!in.buffer.empty()) {
          DSA_LOG(CMD_BLOCK)
            << "ivp" << elem.port << " buffer not empty! Buffer: "
            << in.buffer.size();
          return false;
          // FIXME(@were): This can be more aggressive, but what should I do?
          // if (elem.repeat != in.pes.repeat ||
          //     elem.period != in.pes.period ||
          //     elem.stretch != in.pes.stretch) {
          //   return false;
          // }
        }
      }
      return true;
    }

    bool OVPAvailable(const std::vector<int> &ports) {
      for (auto port : ports) {
        auto &out = accel->output_ports[port];
        if (out.stream) {
          DSA_LOG(CMD_BLOCK) << "ovp" << port << " is serving " << out.stream->toString();
          return false;
        }
        if (std::find(oports.begin(), oports.end(), port) != oports.end()) {
          DSA_LOG(CMD_BLOCK) << "ovp" << port << " is enforced by conflict streams!";
          return false;
        }
      }
      return true;
    }

    /*!
     * \brief Enforce the barrier.
     */
    void Visit(Barrier *bar) override {
      // Add the bits to be blocked.
      barrier_mask |= bar->_mask;
      // A barrier indicates to enforce the order of execution of streams
      // (before, after, and bar itself) that have bit masks in common.
      // If there are streams (either active or wait in FIFO) before
      // this barrier are seperated by this barrier, this barrier should not retire.
      if (bar->_mask & barred) {
        DSA_LOG(BAR) << "Cannot retire because of prior streams.";
        return;
      }
      // If there are active stream affected by this barrier,
      // this barrier should not retire.
      for (auto &i : accel->bsw.iports()) {
        auto &in = accel->input_ports[i.port];
        if (in.stream && (in.stream->barrier_mask() & bar->_mask)) {
          DSA_LOG(BAR) << "Cannot retire because of active streams.";
          return;
        }
      }
      for (auto &i : accel->bsw.oports()) {
        auto &out = accel->output_ports[i.port];
        if (out.stream && (out.stream->barrier_mask() & bar->_mask)) {
          DSA_LOG(BAR) << "Cannot retire because of active streams.";
          return;
        }
      }
      // If no stream is affected by this barrier, retire.
      retire = true;
    }

    /*!
     * \brief Schedule the read stream.
     */
    void Visit(IPortStream *ips) override {
      barred |= ips->barrier_mask();
      // (not blocked by barrier) && (input ports available)
      retire = !(ips->barrier_mask() & barrier_mask) && IVPAvailable(ips->pes);
      std::for_each(ips->pes.begin(), ips->pes.end(),
                    [this] (const PortExecState &pes) { iports.push_back(pes.port); });
    }

    /*!
     * \brief Schedule the write stream.
     */
    void Visit(OPortStream *ops) override {
      barred |= ops->barrier_mask();
      // (not blocked by barrier) && (output ports available)
      retire = !(ops->barrier_mask() & barrier_mask) && OVPAvailable(ops->oports());
      oports.insert(oports.end(), ops->oports().begin(), ops->oports().end());
    }

    /*!
     * \brief Schedule a port to port stream.
     */
    void Visit(PortPortStream *pps) override {
      barred |= pps->barrier_mask();
      // (not blocked by barrier) && (output ports available) && (input ports available)
      retire = !(pps->barrier_mask() & barrier_mask) &&
               OVPAvailable(pps->oports) &&
               IVPAvailable(pps->pes);
      std::for_each(pps->pes.begin(), pps->pes.end(),
                    [this] (const PortExecState &pes) { iports.push_back(pes.port); });
      oports.insert(oports.end(), pps->oports.begin(), pps->oports.end());
    }

    /*!
     * \brief Reset the status after each scheduling.
     */
    void Reset() {
      retire = false;
    }

    /*!
     * \brief The mask of blocking.
     */
    uint64_t barrier_mask{0};
    /*!
     * \brief This accelerator.
     */
    accel_t *accel;
    /*!
     * \brief If this scheduled instruction should be removed from the FIFO.
     */
    bool retire{false};
    /*!
     * \brief The barrier scoreboard.
     */
    uint64_t barred{0};
    /*!
     * \brief The ports occupied by prior streams. For example, both indirect streams
              and port-port streams involve both input and output ports. These stream
              can only be scheduled when both input and output ports are available.
              Partial avaiability may affect the port-enforced stream order.
     */
    std::vector<int> iports, oports;
  };

  struct StreamDispatcher : dsa::sim::stream::Functor {
    StreamDispatcher(accel_t *accel_) : accel(accel_) {}

    /*!
     * \brief Update the status of ports involved by the scheduled stream.
     * \param is The scheduled stream.
     */
    void BindStreamToPorts(const vector<PortExecState> &pes, base_stream_t *stream) {
      for (auto &elem : pes) {
        auto &ivp = accel->input_ports[elem.port];
        ivp.bindStream(stream);
        ivp.pes = elem;
      }
    }
  
    void BindStreamToPorts(const vector<int> &ports, base_stream_t *stream) {
      for (auto &elem : ports) {
        accel->output_ports[elem].bindStream(stream);
      }
    }

    void debugLog(base_stream_t *s0, base_stream_t *s1) {
      DSA_LOG(DISPATCH)
        << "[" << accel->get_ssim()->CurrentCycle() << "]: [" << s0
        << "] Issue to Accel" << accel->accel_index() << " " << s1->toString();
    }

    /*!
     * \brief Schedule the read stream.
     */
    void Visit(IPortStream *ips) override {
      auto cloned = ips->clone(accel);
      BindStreamToPorts(cloned->pes, cloned);
      debugLog(ips, cloned);
    }

    /*!
     * \brief Schedule the write stream.
     */
    void Visit(OPortStream *ops) override {
      auto cloned = ops->clone(accel);
      BindStreamToPorts(ops->oports(), cloned);
      debugLog(ops, cloned);
    }

    /*!
     * \brief Schedule a port to port stream.
     */
    void Visit(PortPortStream *pps) override {
      int port = pps->oports[0];
      auto cloned = pps->clone(accel);
      BindStreamToPorts(pps->pes, cloned);
      auto &out = accel->output_ports[port];
      out.bindStream(cloned);
      debugLog(pps, cloned);
    }

    /*!
     * \brief This accelerator.
     */
    accel_t *accel;
  };

  std::vector<DispatchChecker> dc;
  std::vector<StreamDispatcher> sd;

  for (int i = 0; i < lanes.size(); ++i) {
    dc.emplace_back(lanes[i]);
    sd.emplace_back(lanes[i]);
  }

  for (int i = 0; i < (int) cmd_queue.size(); ++i) {
    bool retire = true;
    for (int j = 0; j < (int) lanes.size(); ++j) {
      if (cmd_queue[i]->context >> j & 1) {
        cmd_queue[i]->Accept(&dc[j]);
        retire = retire && dc[j].retire;
        if (!retire) {
          DSA_LOG(CMD_BLOCK) << "Cannot Issue Stream: " << cmd_queue[i]->toString();
          break;
        }
      }
    }
    if (retire) {
      for (int j = 0; j < (int) lanes.size(); ++j) {
        if (cmd_queue[i]->context >> j & 1) {
          cmd_queue[i]->Accept(&sd[j]);
        }
      }
    }
    for (int j = 0; j < (int) lanes.size(); ++j) {
      if (cmd_queue[i]->context >> j & 1) {
        dc[j].Reset();
      }
    }
    if (retire) {
      cmd_queue.erase(cmd_queue.begin() + i);
      --i;
    }
  }

}

//The reroute function handles either local recurrence, or remote data transfer
void ssim_t::Reroute(int oport, int iport) {
  // TODO(@were): Strided padding.
  // TODO(@were): Inter-lane rerouting.
  auto ips = gatherBroadcastPorts(iport);
  int dtype = (1 << (rf[DSARF::CSR].value & 3));
  auto n = rf[DSARF::L1D].value;
  auto pps = new RecurrentStream(rf[DSARF::TBC].value, oport, ips, n);
  pps->dtype = dtype;
  cmd_queue.push_back(pps);
  DSA_LOG(COMMAND) << now() << ": Recur: "<< pps->toString();
}

// receive network message at the given input port id
void ssim_t::push_in_accel_port(int accel_id, int8_t* val, int num_bytes, int in_port) {
  assert(accel_id < lanes.size() && accel_id >= 0);
  lanes[accel_id]->receive_message(val, num_bytes, in_port);
}

// SPU has only 1 DGRA in a core
void ssim_t::push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc) {
  // lanes[0]->push_atomic_update_req(scr_addr, opcode, val_bytes, out_bytes, inc);
}

void ssim_t::push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    lanes[0]->push_ind_rem_read_req(is_remote, req_core, request_ptr, addr, data_bytes, reorder_entry);
}

void ssim_t::push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    lanes[0]->push_ind_rem_read_data(data, request_ptr, addr, data_bytes, reorder_entry);
}

bool ssim_t::StreamBufferAvailable() {
  return cmd_queue.size() < spec.cmd_queue_size;
}

bool ssim_t::done(bool show, int mask) {
  bool is_done=true;
  auto context = rf[DSARF::TBC].value;
  for(int i = 0; i < (int) lanes.size(); ++i) {
    if(context >> i & 1) {
      is_done &= lanes[i]->done(show, mask);
    }
  }
  return is_done;
}

bool ssim_t::is_in_config() {
  for(int i = 0; i < (int) lanes.size(); ++i) {
    if (lanes[i]->statistics.blame == dsa::stat::Accelerator::CONFIGURE) {
      return true;
    }
  }
  return false;
}

void ssim_t::cycle_shared_busses() {
}

void ssim_t::step() {
  if (!_in_use) {
    return;
  }
  cycle_shared_busses();
  DispatchStream();
  for(int i = 0; i < (int) (lanes.size() - 1); ++i) {
    if(_ever_used_bitmask >> i & 1) {
      lanes[i]->tick();
    }
  }
  // shared_acc()->tick();
}

void ssim_t::print_stats() {
   auto& out = std::cout;
   out.precision(4);
   out << dec;

   out << "\n*** ROI STATISTICS for CORE ID: " << lsq()->getCpuId() << " ***\n";
   out << "Simulator Time: " << statistics.timeElapsed() << " seconds" << std::endl;

   out << "Cycles: " << (int) statistics.cycleElapsed() << "\n";
   out << "Number of coalesced SPU requests: " << lanes[0]->_stat_num_spu_req_coalesced << "\n";
   out << "Control Core Insts Issued: " << statistics.insts_issued << "\n";
   out << "Control Core Discarded Insts Issued: " << statistics.insts_discarded << "\n";
   out << "Control Core Intrinsics/Instruction Issued: " << statistics.ctrl_intrinsics << "/" << statistics.ctrl_instructions << "\n";

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
  for (int i = 0; i < (int) lanes.size(); ++i) {
    if (_ever_used_bitmask >> i & 1) {
      lanes[i]->print_stats();
      if(i != (int) lanes.size()) {
        cores_used++;
      }
    }
  }

  std::cout << "\nCores used: " << cores_used << "bitmask: "
            << std::hex << _ever_used_bitmask << std::dec << "\n";

  uint64_t total_mem_accesses=0;
  uint64_t max_comp_instances=0;
  for(int i = 0; i < lanes.size(); ++i) {
    if(_ever_used_bitmask >> i & 1) {
      total_mem_accesses+=
        lanes[i]->_bw_map[make_pair(LOC::DMA,LOC::TOTAL)].first;
      max_comp_instances=std::max(max_comp_instances,
         lanes[i]->_stat_comp_instances);
    }
  }

  std::cout << "Total atomic updates: " << lanes[0]->_stat_tot_updates << std::endl;

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
  for(int i = 0; i < (int) lanes.size(); ++i) {
    if(_ever_used_bitmask >> i & 1) {
      r = std::max(lanes[i]->forward_progress_cycle(),r);
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
  for(int i = 0; i < (int) lanes.size(); ++i) {
    if(context >> i & 1) {
      std::cout << i <<" ";
    }
  }
  std::cout << "\t";
}

// it is not a stream; just a linear write (just push data into buf?)
void ssim_t::write_remote_banked_scratchpad(uint8_t* val, int num_bytes, uint16_t scr_addr) {
  // TODO: add a check for full buffer to apply backpressure to network
  lanes[0]->push_scratch_remote_buf(val, num_bytes, scr_addr); // hopefully, we use single accel per CC
}

void ssim_t::atomic_update_hardware_config(int addr_port, int val_port, int out_port) {

  // set up the associated stream here
  // lanes[0]->_scr_w_c.set_atomic_cgra_addr_port(addr_port);
  // lanes[0]->_scr_w_c.set_atomic_cgra_val_port(val_port);
  // lanes[0]->_scr_w_c.set_atomic_cgra_out_port(out_port);
  // lanes[0]->_scr_w_c.set_atomic_addr_bytes(2); // get_bytes_from_type(addr_type));

  // port_data_t& value_port = lanes[0]->port_interf().in_port(val_port);
  // lanes[0]->_scr_w_c.set_atomic_val_bytes(value_port.get_port_width());
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
    // BroadcastStream(s);

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
      DSA_LOG(NET_REQ) << s->toString();
      // BroadcastStream(s);
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
        // BroadcastStream(s);
      } else { // inherited by the affine_base_stream
        direct_remote_scr_stream_t* s = new direct_remote_scr_stream_t(mask, access_size, stride);
        s->_scr_type = 0; // spad_type; // this should not be required now?
        s->_num_elements = num_elem; // this is num_strides
        s->_out_port = out_port;
        s->set_orig();
        // BroadcastStream(s);
      }
    }
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
  for(int i = 0; i < (int) lanes.size(); ++i) {
    if(context >> i & 1) {
      auto &ovp = lanes[i]->output_ports[port];
      CHECK(dtype % ovp.scalarSizeInBytes() == 0);
      if (ovp.stream) {
        break;
      }
      if (ovp.raw.size() >= dtype) {
        DSA_LOG(RECV) << now() << ": Receive " << dtype << " bytes from " << port;
        return true;
      } else {
        break;
      }
    }
  }
  // DSA_LOG(RECV) << "CANNOT receive " << dtype << " bytes from " << port;
  return false;
}

uint64_t ssim_t::Receive(int port, int dtype) {
  auto context = rf[DSARF::TBC].value;
  CHECK((context & -context) == context)
    << "More than one accelerator to receive!";
  CHECK(context) << "No accelerator to receive";
  for (int i = 0; i < (int) lanes.size(); ++i) {
    if(context >> i & 1) {
      auto &ovp = lanes[i]->output_ports[port];
      std::vector<uint8_t> raw(ovp.raw.begin(), ovp.raw.begin() + dtype);
      ovp.pop(dtype);
      CHECK(raw.size() <= 8);
      raw.resize(8, 0);
      auto res = *reinterpret_cast<uint64_t*>(&raw[0]);
      DSA_LOG(RECV)
        << "SS_RECV value: " << res
        << " on port" << ovp.id() << " " << ovp.raw.size()
        << " on core: " << lsq()->getCpuId();
      return res;
    }
  }
  return -1;
}


void ssim_t::InsertBarrier(uint64_t mask) {
  Barrier* s = new Barrier(rf[DSARF::TBC].value, mask);
  s->set_orig();
  // BroadcastStream(s);
}

// TODO:FIXME: see if we need new mask
void ssim_t::insert_df_barrier(int64_t num_scr_wr, bool spad_type) {
  Barrier* s = new Barrier(0, 0);
  s->_mask = 64;
  // s->_num_remote_writes = num_scr_wr;
  s->_scr_type = spad_type; // TODO: Where to use this?
  s->set_orig();
  // printf("df count read from register is: %ld\n",num_scr_wr);
  lanes[0]->_scr_w_c.set_df_count(num_scr_wr);
  // BroadcastStream(s);
}

uint64_t ssim_t::now() {
  return curTick(); //lsq()->get_cpu().curCycle();
}

void ssim_t::update_stat_cycle() {
  assert(in_roi());
  if (_stat_start_cycle == ~0ull) {
    _stat_start_cycle = now();
  } else {
    _stat_stop_cycle = now();
  }
}

// ------------------------- TIMING ---------------------------------
void ssim_t::roi_entry(bool enter) {
  if(enter) {
    if(_orig_stat_start_cycle == 0) {
      _orig_stat_start_cycle = now();
    }
    _in_roi=true;
    _times_roi_entered+=1;
  } else {
    _roi_cycles += _stat_stop_cycle - _stat_start_cycle;
    _in_roi=false;
  }
}

int ssim_t::get_bytes_from_type(int t) {
  CHECK(false);
  return 0;
}

#include <fstream>
#include <iomanip>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <sstream>
#include <numeric>
#include <functional>

#include "dsa-ext/rf.h"
#include "dsa/dfg/utils.h"

#include "../cpu.hh"
#include "./bitstream.h"
#include "./ssim.hh"
#include "./arbiter.h"
#include "./request.h"
#include "./ism.h"

using namespace std;

const char *BLAME_NAME[] = {
#define MACRO(x) #x,
#include "./blame.def"
#undef MACRO
};

std::vector<uint8_t> apply_mask(const uint8_t *raw_data, vector<bool> mask) {
  std::vector<uint8_t> res;
  for (int i = 0; i < mask.size(); ++i) {
    if (mask[i]) {
      res.push_back(raw_data[i]);
    }
  }
  return res;
}

int accel_t::get_cur_cycle() {
  return now() - _ssim->roi_enter_cycle();
}

dsa::sim::Port *accel_t::port(bool isInput, int id) {
  if (isInput) {
    CHECK(id >= 0 && id < (int) input_ports.size());
    return input_ports.data() + id;
  }
  CHECK(id >= 0 && id < (int) output_ports.size());
  return output_ports.data() + id;
}


void accel_t::sanity_check_stream(base_stream_t *s) {

  struct SanityChecker : dsa::sim::stream::Functor {
    void CheckIt(bool is_input, int port) {
      auto name = accel->bsw.name(is_input, port);
      if (name.find("port: ") == 0) {
        DSA_WARNING << name << " has no name!";
      }
    }
    void Visit(IPortStream *is) {
      for (auto elem : is->pes) {
        CheckIt(true, elem.port);
      }
    }

    void Visit(OPortStream *os) {
      for (auto elem : os->oports()) {
        CheckIt(false, elem);
      }
    }

    void Visit(PortPortStream *pps) {
      for (auto elem : pps->pes) {
        CheckIt(true, elem.port);
      }
      for (auto elem : pps->oports) {
        CheckIt(false, elem);
      }
    }

    SanityChecker(accel_t *accel_) : accel(accel_) {}

    accel_t *accel;
  };

  SanityChecker sc(this);
  s->Accept(&sc);
}

void accel_t::request_reset_data() {
  _cleanup_mode = true;
  reset_data();
  DSA_LOG(COMMAND_O) << "Complete reset request served except deleting requested memory reads/writes: "
                 << lsq()->getCpuId() << "\n";
}

// Let's not consider input ports here
bool accel_t::all_ports_empty() {

  // It should check only those output ports which are busy
  for (auto &elem : bsw.oports()) {
    int cur_port = elem.port;
    auto &out_vp = output_ports[cur_port];
    // if(out_vp.in_use() && out_vp.mem_size()) {
    if(out_vp.empty()) {
      std::cout << "Output port not empty: " << cur_port << " at core id: " << lsq()->getCpuId() << endl;
      return false;
    }
  }

  std::cout << "Output ports empty\n";

  return true;
}

void accel_t::request_reset_streams() {

  if(!all_ports_empty()) return;

  DSA_LOG(COMMAND_O) << "RESET STREAM REQUEST RELEASED for core: " << lsq()->getCpuId();

  // free ports and streams (not input ports?)
  // reset_data();
  for (auto &elem : bsw.oports()) {
    int cur_port = elem.port;
    auto &out_vp = output_ports[cur_port];
    out_vp.reset();
  }

  for (auto &elem : bsw.iports()) {
    int cur_port = elem.port;
    auto &in_vp = input_ports[cur_port];
    in_vp.reset();
  }

  _dma_c.reset_data();
  _scr_w_c.reset_data();
  _net_c.reset_data();

  _stream_cleanup_mode=false;
  _cleanup_mode = true; // it should wait for outstanding mem req to be done (wait on o/p ports and then cleanup memory)
}

void accel_t::switch_stream_cleanup_mode_on() {
  _stream_cleanup_mode = true;
}

void accel_t::reset_data() {
  DSA_LOG(COMMAND) << now() << ": Reset data requested";

  for (int i = 0; i < 2; ++i) {
    for (auto &elem : bsw.ports[i]) {
      auto &vp = input_ports[elem.port];
      vp.reset();
    }
  }

  _dma_c.reset_data();
  _scr_w_c.reset_data();
  _net_c.reset_data();

}

// ------------------------------ VECTOR PORTS

// ---------------------------- ACCEL ------------------------------------------
uint64_t accel_t::now() {
  return get_ssim()->now();
}

Minor::LSQ *accel_t::lsq() {
  return get_ssim()->lsq();
}

accel_t::accel_t(int i, ssim_t *ssim)
    : arbiter(new dsa::sim::RoundRobin()),
      statistics(*this),
      _ssim(ssim), _accel_index(i), _accel_mask(1 << i),
      _dma_c(this, &_scr_r_c, &_scr_w_c, &_net_c), _scr_r_c(this, &_dma_c),
      _scr_w_c(this, &_dma_c), _net_c(this, &_dma_c) {
  spads.emplace_back(8, 32, SCRATCH_SIZE, 1, new dsa::sim::InputBuffer(4, 16, 1));

  ENFORCED_SYSTEM("mkdir -p stats/");
  ENFORCED_SYSTEM("mkdir -p viz/");

  const char *fifo_len_str = std::getenv("FU_FIFO_LEN");
  if (fifo_len_str != nullptr) {
    _fu_fifo_len = atoi(fifo_len_str);
  }

  const char *ind_rob_size = std::getenv("IND_ROB_SIZE");
  if (ind_rob_size != nullptr) {
    _ind_rob_size = atoi(ind_rob_size);
  }

  assert(_ind_rob_size<64 && "ind rob size exceeded the bits allocated in SPU-net transaction");

  std::string ssconfig_file = std::getenv("SBCONFIG");
 
  const char *back_cgra_str = std::getenv("BACKCGRA");
  if (back_cgra_str != NULL) {
    _back_cgra = true;
  }
 
  const char *linear_spad_str = std::getenv("LINEAR_SCR");
  if (linear_spad_str != NULL) {
    _linear_spad = true;
  }
  _banked_spad_mapping_strategy = std::getenv("MAPPING");

  if (!get_ssim()->spec.adg_file.empty()) {
    ssconfig_file = get_ssim()->spec.adg_file;
  }
  _ssconfig = new SSModel(ssconfig_file.c_str());
  // cout << "Came here to create a new configuration for a new accel\n";
  _ssconfig->setMaxEdgeDelay(_fu_fifo_len);
  // TODO(@were): This is hacky.
  int buffer_size = get_ssim()->spec.dma_bandwidth * 2;
  auto in_list = _ssconfig->subModel()->input_list();
  for (int i = 0; i < (int) in_list.size(); ++i) {
    input_ports.emplace_back(this, buffer_size, i);
  }
  auto out_list = _ssconfig->subModel()->output_list();
  for (int i = 0; i < (int) out_list.size(); ++i) {
    output_ports.emplace_back(this, buffer_size, i);
  }

  scratchpad.resize(SCRATCH_SIZE);
  if (_linear_spad) {
    scratchpad.resize(SCRATCH_SIZE + LSCRATCH_SIZE);
    // scratchpad_readers.resize(SCRATCH_SIZE+LSCRATCH_SIZE);
    // scratchpad_writers.resize(SCRATCH_SIZE+LSCRATCH_SIZE);
  }

  if(lsq()->getCpuId() == 0) { // single-core simulation
    _ssim->set_num_active_threads(1);
  }

  // optionally used -- this is expensive
  // TODO: make it compat with linear scr case
  scratchpad_readers.resize(SCRATCH_SIZE);
  scratchpad_writers.resize(SCRATCH_SIZE);

  // FIXME(@were): Implicit streams for remote spad access.
  // if (i == 0) { // required only in 1st accel
  //   remote_core_net_stream_t *implicit_stream = new remote_core_net_stream_t();

  //   port_data_t *in_addr = &_port_interf.in_port(NET_ADDR_PORT);
  //   port_data_t *in_val = &_port_interf.in_port(NET_VAL_PORT);
  //   _scr_w_c.schedule_network_stream(*implicit_stream);
  //   in_addr->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
  //   in_val->set_status(port_data_t::STATUS::BUSY, LOC::PORT);

  //   // push_net_in_cmd_queue(implicit_stream);
  // }
}

void accel_t::timestamp() {
  _ssim->timestamp();
  std::cerr << "acc" << _accel_index << " ";
}

bool accel_t::in_use() { return _ssim->in_use(); }

bool accel_t::in_roi() { return _ssim->in_roi(); }

// The job of whos_to_blame(group) is to determine the reason of whether
// a group is able to issue or not.
// It's not possible to determine if
// there is data/stream for one port, but no data/stream for another, why
// that is (is it scatchpad or core?), so that's not answered here.
pipeline_stats_t::PIPE_STATUS accel_t::whos_to_blame(int group) {
  return pipeline_stats_t::NOT_IN_USE;
  // bool any_empty_fifo = false, any_scr = false, any_dma = false,
  //      any_const = false, any_rec = false;

  // auto &active_ports = _soft_config.in_ports_active_group[group];
  // auto &active_out_ports = _soft_config.out_ports_active_group[group];

  // if (active_ports.size() == 0)
  //   return pipeline_stats_t::NOT_IN_USE;

  // if (_cgra_issued_group[group])
  //   return pipeline_stats_t::ISSUED;

  // // Iterate over inputs
  // bool any_input_activity = false;
  // bool all_inputs_inactive = true;
  // int num_unknown_input = 0;
  // for (unsigned i = 0; i < active_ports.size(); ++i) {
  //   auto &in_vp = _port_interf.in_port(active_ports[i]);
  //   bool assigned_ivp = in_vp.in_use() || in_vp.completed();
  //   bool empty_fifo = in_vp.any_data() == 0;
  //   any_empty_fifo |= empty_fifo;
  //   if (empty_fifo && assigned_ivp) {
  //     switch (in_vp.loc()) {
  //     case LOC::DMA:
  //       any_dma = true;
  //       break;
  //     case LOC::SCR:
  //       any_scr = true;
  //       break;
  //     case LOC::PORT:
  //       any_rec = true;
  //       break;
  //     case LOC::CONST:
  //       any_const = true;
  //       break;
  //     default:
  //       break;
  //     }
  //   }
  //   num_unknown_input += (empty_fifo && !assigned_ivp);
  //   any_input_activity |= assigned_ivp || !empty_fifo;
  //   all_inputs_inactive = all_inputs_inactive && empty_fifo && !assigned_ivp;
  // }

  // if (num_unknown_input == 0) {
  //   if (!any_empty_fifo)
  //     return pipeline_stats_t::CGRA_BACK;
  //   if (any_const)
  //     return pipeline_stats_t::CONST_FILL;
  //   if (any_scr)
  //     return pipeline_stats_t::SCR_FILL;
  //   if (any_dma)
  //     return pipeline_stats_t::DMA_FILL;
  //   if (any_rec)
  //     return pipeline_stats_t::REC_WAIT;
  // }

  // for (unsigned i = 0; i < active_out_ports.size(); ++i) {
  //   // This is just a guess really, but things really are bad
  //   // if memory write is the bottleneck...
  //   auto &out_vp = _port_interf.out_port(active_out_ports[i]);
  //   if (out_vp.any_data()) {
  //     if (!lsq()->sd_transfers[MEM_WR_STREAM].canReserve()) {
  //       return pipeline_stats_t::DMA_WRITE;
  //     }
  //   }
  // }

  // bool bar_scratch_read = false; // These are set by scratch
  // bool bar_scratch_write = false;
  // bool bar_remote_scratch_write = false;
  // std::set<int> scratch_waiters;
  // std::set<int> waiters;
  // for (auto i = _cmd_queue.begin(); i != _cmd_queue.end(); ++i) {
  //   base_stream_t *ip = i->get();
  //   if (auto stream = dynamic_cast<Barrier *>(ip)) {
  //     bar_scratch_read |= stream->bar_scr_rd();
  //     bar_scratch_write |= stream->bar_scr_wr();
  //     bar_remote_scratch_write |= stream->bar_scr_wr_df(); // it has no waiters
  //   }
  //   if (auto is = dynamic_cast<IPortStream*>(ip)) {
  //     for (auto &port : is->pes) {
  //       port_data_t &in_vp = _port_interf.in_port(port.port);
  //       auto it = std::find(active_ports.begin(), active_ports.end(), port.port);
  //       if (it == active_ports.end())
  //         continue;

  //       if (!(in_vp.in_use() || in_vp.completed() || in_vp.any_data())) {
  //         if (bar_scratch_write && ((ip->src() & LOC::SCR) != LOC::NONE)) {
  //           scratch_waiters.insert(port.port);
  //         } else {
  //           waiters.insert(port.port);
  //         }
  //       }
  //     }
  //   }
  // }

  // // if all of our unknowns are waiting for us, then maybe its scratch or cmd
  // // queue
  // unsigned total_waiters = scratch_waiters.size() + waiters.size();
  // if (total_waiters == num_unknown_input) {
  //   if (scratch_waiters.size()) {
  //     return pipeline_stats_t::SCR_BAR_WAIT;
  //   }
  //   return pipeline_stats_t::CMD_QUEUE;
  // }
  // if (scratch_waiters.size() && _cmd_queue.size() >= _queue_size) {
  //   return pipeline_stats_t::SCR_BAR_WAIT;
  // }

  // any_input_activity |= (total_waiters); // waiters counts as input activity

  // bool any_out_activity = false, any_cgra_activity = false;
  // for (unsigned i = 0; i < active_out_ports.size(); ++i) {
  //   auto &out_vp = _port_interf.out_port(active_out_ports[i]);
  //   bool assigned_ovp = out_vp.in_use() || out_vp.completed();
  //   any_cgra_activity |= out_vp.num_in_flight();
  //   any_out_activity |= assigned_ovp || out_vp.any_data();
  // }

  // // No activity on this group, so ignore it
  // if (!any_input_activity && !any_cgra_activity && !any_out_activity) {
  //   return pipeline_stats_t::NOT_IN_USE;
  // }

  // if (!any_input_activity) { // no input, but at least some cgra/out
  //   return pipeline_stats_t::DRAIN;
  // }

  // // some input activity

  // return pipeline_stats_t::CORE_WAIT;
  // cout << num_unassigned << "/" << total_ivps << "-" <<
  // num_unassigned_queued; done(true,0);
}

// Figure out who is to blame for a given cycle's not issuing
// void accel_t::whos_to_blame(
//     std::vector<pipeline_stats_t::PIPE_STATUS> &blame_vec,
//     std::vector<pipeline_stats_t::PIPE_STATUS> &group_vec) {
//   return;
//   if (_soft_config.in_ports_active.size() == 0) {
//     blame_vec.push_back(pipeline_stats_t::NOT_IN_USE);
//     return;
//   }
// 
//   bool draining = false, cgra_back = false;
// 
//   // Temp vec are for reasons that we consider to have priority
//   // when considering blame over draining and backpressure.
//   std::vector<pipeline_stats_t::PIPE_STATUS> temp_vec;
// 
//   for (int g = 0; g < NUM_GROUPS; ++g) {
//     if (_dfg->meta[g].is_temporal)
//       continue;
// 
//     pipeline_stats_t::PIPE_STATUS blame = whos_to_blame(g);
//     group_vec.push_back(blame);
//     switch (blame) {
//     case pipeline_stats_t::DRAIN:
//       draining = true;
//       break;
//     case pipeline_stats_t::CGRA_BACK:
//       cgra_back = true;
//       break;
//     case pipeline_stats_t::NOT_IN_USE:
//       break;
//     default:
//       temp_vec.push_back(blame);
//       break;
//     }
//   }
// 
//   if (_dedicated_cgra_issued > 1) {
//     blame_vec.push_back(pipeline_stats_t::ISSUED_MULTI);
//     return;
//   }
//   if (_dedicated_cgra_issued == 1) {
//     blame_vec.push_back(pipeline_stats_t::ISSUED);
//     return;
//   }
// 
//   if (_backcgra_issued != 0) {
//     blame_vec.push_back(pipeline_stats_t::TEMPORAL_ONLY);
//     return;
//   }
// 
//   if (temp_vec.size() == 0) {
//     if (draining) {
//       blame_vec.push_back(pipeline_stats_t::DRAIN);
//       return;
//     }
//     if (cgra_back) {
//       blame_vec.push_back(pipeline_stats_t::CGRA_BACK);
//       return;
//     }
//     blame_vec.push_back(pipeline_stats_t::CORE_WAIT);
//     return;
//   }
// 
//   for (auto i : temp_vec) {
//     blame_vec.push_back(i);
//   }
// 
// }

struct StreamExecutor : dsa::sim::stream::Functor {

  /*!
   * \brief Pad the operand data by the generated bitmask.
   * \param data The data to be padded.
   * \param info The cacheline information to which the bitmask belongs.
   */
  static std::vector<uint8_t> padData(const std::vector<uint8_t> &data,
                                      const dsa::sim::stream::LinearStream::LineInfo &info) {
    auto res = data;
    std::vector<uint8_t> before(info.start - info.linebase, 0);
    res.insert(res.begin(), before.begin(), before.end());
    std::vector<uint8_t> after(info.mask.size() - res.size(), 0);
    res.insert(res.end(), after.begin(), after.end());
    CHECK(res.size() == info.mask.size());
    return res;
  }

  /*!
   * \brief If we can request data from the given component.
   * \param loc Either DMA or SPAD.
   * \param port It is to comply the LSQ interface.
   */
  int canRequest(LOC loc, int port) {
    if (loc == LOC::DMA) {
      if (accel->lsq()->sd_transfers[port].unreservedRemainingSpace() &&
          accel->lsq()->canRequest()) {
        return accel->get_ssim()->spec.dma_bandwidth;
      }
    } else {
      CHECK(loc == LOC::SCR);
      if (accel->spads[0].rb->Available()) {
        return accel->spads[0].bandwidth();
      }
    }
    return -1;
  }

  void makeMemoryRequest(base_stream_t *s, const std::vector<uint8_t> &data, const std::vector<int> &ports,
                         const dsa::sim::stream::LinearStream::LineInfo &info, MemoryOperation mo) {
    int read = mo == DMO_Read;
    if (read) {
      int to_reserve =
        std::accumulate(info.mask.begin(), info.mask.end(), (int) 0, std::plus<int>());
      reserveBuffers(ports, to_reserve);
    }
    if (s->side(read) == LOC::DMA) {
      makeDMARequest(s->id(), s->inst, mo, ports, data, info);
    } else {
      auto padded = data;
      if (!padded.empty()) {
        padded = padData(data, info);
      }
      accel->spads[0].rb->Decode(ports[0], mo, info, padded);
    }
  }

  void makeDMARequest(int sid, Minor::MinorDynInstPtr inst, MemoryOperation op,
                      const std::vector<int> &ports,
                      const std::vector<uint8_t> &data,
                      const dsa::sim::stream::LinearStream::LineInfo &info) {
    bool read = op == MemoryOperation::DMO_Read;
    auto *sdInfo = new SSMemReqInfo(sid, accel->accel_index(), ports,
                                    read ? info.mask : std::vector<bool>(),
                                    accel->now(), info.as);
    if (read) {
      accel->lsq()->sd_transfers[ports[0]].reserve();
    }
    int size = read ? info.mask.size() : data.size();
    auto address = read ? info.linebase : info.start;
    // make request
    accel->lsq()->pushRequest(inst, /*isLoad*/op == MemoryOperation::DMO_Read,
                              /*data*/read ? nullptr : const_cast<uint8_t*>(data.data()),
                              /*size in bytes*/size, /*addr*/address, /*flags*/0,
                              /*res*/0, /*atomic op*/nullptr, /*byte enable*/std::vector<bool>(),
                              sdInfo);
    DSA_LOG(MEM_REQ) << (read ? "Read" : "Write") << " request: " << info.linebase << ", " << info.start;
  }

  void Visit(ConstPortStream *cps) override {
    auto &stream = *cps;
    int pushed = 0;
    while (stream.stream_active() && pushed < accel->get_ssim()->spec.const_bandwidth &&
           bufferAvailable(cps->pes, cps->dtype) >= cps->dtype) {
      int64_t scalar = stream.ls->poll(false);
      std::vector<uint8_t> data((uint8_t*) &scalar, ((uint8_t*) &scalar) + cps->dtype);
      for (auto &elem : stream.pes) {
        auto &in = accel->input_ports[elem.port];
        in.push(data, sim::stream::AffineStatus(), true);
      }
      auto value = stream.ls->poll(true);
      DSA_LOG(CONST) << "Const value pushed: " << value;
      DSA_LOG(CONST) << stream.toString();
      pushed += stream.data_width();
    }
    accel->statistics.countDataTraffic(true, cps->src(), pushed);
    if (!stream.stream_active()) {
      accel->process_stream_stats(stream);
      for (auto &elem : stream.pes) {
        accel->input_ports[elem.port].freeStream();
      }
    }
  }


  void Visit(LinearReadStream *lrs) override {
    auto &stream = *lrs;
    CHECK(stream.stream_active());
    int cacheline = canRequest(stream.src(), lrs->pes[0].port);
    // If the request unit is not available just return.
    if (cacheline == -1) {
      return;
    }
    int available = cacheline;
    if (stream.be) {
      available = std::min(stream.be->occupied, available);
      stream.be->mo = DMO_Read;
    }
    auto info = stream.ls->cacheline(cacheline, available, stream.be, DMO_Read, stream.src());
    info.as.padding = (Padding) stream.padding;
    std::vector<bool> bm = info.mask;
    std::vector<int> ports;
    for (auto &elem : stream.pes) {
      ports.push_back(elem.port);
    }
    makeMemoryRequest(lrs, {}, ports, info, DMO_Read);
    accel->statistics.countDataTraffic(true, stream.unit(), info.bytes_read());
    int total_bytes = std::accumulate(bm.begin(), bm.end(), 0);
    DSA_LOG(MEM_REQ)
      << "read request: " << info.linebase << ", " << info.start
      << " for " << stream.toString() << " in total " << total_bytes << " bytes";
  }

  void Visit(IndirectReadStream *irs) override {
    CHECK(irs->oports.size() == 1);
    if (irs->fsm.hasNext(accel) != 1) {
      return;
    }
    std::vector<int> ports;
    for (auto &elem : irs->pes) {
      ports.push_back(elem.port);
    }
    if (canRequest(irs->src(), irs->pes[0].port) == -1) {
      return;
    }
    int n = irs->src() == LOC::DMA ? 1 : accel->spads[0].bandwidth() / irs->fsm.idx().dtype;
    std::vector<int64_t> addrs;
    for (int i = 0; i < n; ++i) {
      if (irs->fsm.hasNext(accel) != 1) {
        break;
      }
      addrs.push_back(irs->fsm.poll(accel, true)[0]);
    }
    if (irs->src() == LOC::DMA) {
      uint64_t cacheline = accel->get_ssim()->spec.dma_bandwidth;
      std::vector<bool> bm(cacheline, false);
      auto addr = addrs[0];
      int linebase = addr & ~(cacheline - 1);
      for (int i = 0; i < irs->data_width(); ++i) {
        CHECK(i + addr % cacheline < bm.size());
        bm[i + addr % cacheline] = true;
      }
      dsa::sim::stream::LinearStream::LineInfo info(linebase, addr, bm, 0);
      reserveBuffers(ports, irs->data_width());
      info.as.stream_last = !irs->fsm.hasNext(accel);
      info.as.padding = DP_NoPadding;
      makeDMARequest(irs->id(), irs->inst, DMO_Read, ports, {}, info);
      accel->statistics.countDataTraffic(true, irs->unit(), info.bytes_read());
    } else {
      // TODO(@were): Merge this to the indirect atomic write to a unified function.
      std::vector<dsa::sim::Request> requests;
      dsa::sim::stream::LinearStream::LineInfo meta(0, 0, {}, 0);
      for (int i = 0; i < addrs.size(); ++i) {
        auto addr = addrs[i];
        requests.emplace_back(irs->pes[0].port, addr, irs->dtype, MemoryOperation::DMO_Read);
        auto &mask = requests.back().mask;
        mask.resize(accel->spads[0].bank_width, 0);
        int offset = addr % accel->spads[0].bank_width;
        CHECK(offset % irs->dtype == 0) << addr << " is not aligned with " << irs->dtype;
        for (int j = 0; j < irs->dtype; ++j) {
          CHECK(offset + j < mask.size()) << "No bank straddle is allowed.";
          mask[offset + j] = 1;
        }
        reserveBuffers(ports, irs->dtype);
        meta.mask.insert(meta.mask.end(), mask.begin(), mask.end());
        DSA_LOG(MEM_REQ)
          << " [Indirect Read Request] addr: " << addr
          << ", memory op: " << MemoryOperation::DMO_Read;
      }
      meta.as.stream_last = !irs->fsm.hasNext(accel);
      accel->spads[0].rb->Decode(requests, meta);
    }
  }

  void Visit(IndirectAtomicStream *ias) override {
    if (ias->fsm.hasNext(accel) != 1) {
      return;
    }
    if (canRequest(ias->dest(), MEM_WR_STREAM) == -1) {
      return;
    }
    int n = ias->unit() == LOC::DMA ? 1 : accel->spads[0].bandwidth() / ias->dtype;
    std::vector<int64_t> addrs;
    std::vector<int64_t> data;
    for (int i = 0; i < n; ++i) {
      if (ias->fsm.hasNext(accel) != 1) {
        DSA_LOG(ATOMIC) << "No element to write!";
        break;
      }
      auto buffer = ias->fsm.poll(accel, true);
      addrs.push_back(buffer[0]);
      data.push_back(buffer[1]);
      DSA_LOG(ATOMIC) << buffer[0] << " " << buffer[1];
    }
    if (ias->unit() == LOC::DMA) {
      uint64_t cacheline = accel->get_ssim()->spec.dma_bandwidth;
      std::vector<bool> bm(cacheline, false);
      auto addr = addrs[0];
      auto linebase = addr & ~(cacheline - 1);
      for (int i = 0; i < ias->data_width(); ++i) {
        bm[i + addr % cacheline] = true;
      }
      dsa::sim::stream::LinearStream::LineInfo info(linebase, addr, bm, 0);
      info.as.stream_last = !ias->stream_active();
      makeDMARequest(
        ias->id(), ias->inst, DMO_Write, {MEM_WR_STREAM},
        std::vector<uint8_t>((uint8_t*) &data[0], (uint8_t*)&data[0] + ias->data_width()), info);
      accel->statistics.countDataTraffic(true, ias->unit(), info.bytes_read());
      if (!ias->stream_active()) {
        DSA_LOG(STREAM) << ias->toString() << " freed!";
        for (auto elem : ias->oports()) {
          accel->output_ports[elem].freeStream();
        }
      }
    } else {
      std::vector<dsa::sim::Request> requests;
      dsa::sim::stream::LinearStream::LineInfo meta(0, 0, {}, 0);
      for (int i = 0; i < addrs.size(); ++i) {
        auto addr = addrs[i];
        requests.emplace_back(MEM_WR_STREAM, addr, ias->dtype, ias->mo);
        auto &mask = requests.back().mask;
        mask.resize(accel->spads[0].bank_width, 0);
        int offset = addr % accel->spads[0].bank_width;
        CHECK(offset % ias->dtype == 0) << addr << " is not aligned with " << ias->dtype;
        for (int j = 0; j < ias->dtype; ++j) {
          CHECK(offset + j < mask.size()) << "No bank straddle is allowed.";
          mask[offset + j] = 1;
        }
        meta.mask.insert(meta.mask.end(), mask.begin(), mask.end());
        requests.back().operand =
          std::vector<uint8_t>((uint8_t*)&data[i], (uint8_t*)&data[i] + ias->dtype);
        DSA_LOG(MEM_REQ)
          << " [SPAD Atomic Request] addr: " << addr << ", data: " << data[i]
          << ", memory op: " << ias->mo;
      }
      if (!ias->stream_active()) {
        DSA_LOG(STREAM) << ias->toString() << " freed!";
        for (auto elem : ias->oports()) {
          accel->output_ports[elem].freeStream();
        }
      }
      meta.as.stream_last = !ias->stream_active();
      accel->spads[0].rb->Decode(requests, meta);
    }
  }

  void Visit(LinearWriteStream *lws) override {
    auto &stream = *lws;
    CHECK(stream.stream_active()) << "Inactive stream should be freed!";
    auto &out_port = accel->output_ports[stream.port()];
    if (!out_port.canPop(stream.dtype)) {
      return;
    }
    if (stream.garbage()) {
      while (stream.stream_active() && out_port.canPop(stream.dtype)) {
        out_port.pop(stream.dtype);
        int cacheline = accel->get_ssim()->spec.dma_bandwidth;
        stream.ls->cacheline(cacheline, stream.dtype,
                             nullptr, DMO_Write, stream.dest());
      }
      return;
    }
    // Check if the write unit is available.
    int memory_bw = canRequest(lws->dest(), MEM_WR_STREAM);
    if (memory_bw == -1) {
      return;
    }
    // Prepare the data according to the port data availability and memory bandwidth
    auto &ovp = out_port;
    int port_available = std::min(ovp.scalarSizeInBytes() * ovp.vectorLanes(), (int) ovp.raw.size());
    // Make sure the value requested does not exceed the buffet buffer.
    if (lws->be) {
      port_available = std::min(lws->be->SpaceAvailable(), port_available);
      stream.be->mo = DMO_Write;
    }
    DSA_LOG(MEM_REQ) << "write bandwidth: " << memory_bw;
    auto info = stream.ls->cacheline(memory_bw, std::min(memory_bw, port_available), stream.be,
                                     DMO_Write, stream.dest());
    int to_pop = std::accumulate(info.mask.begin(), info.mask.end(), (int) 0, std::plus<int>());
    CHECK(to_pop % ovp.scalarSizeInBytes() == 0)
      << to_pop << " % " << ovp.scalarSizeInBytes() << " != 0";
    auto data = ovp.poll(to_pop);
    ovp.pop(to_pop);
    makeMemoryRequest(lws, data, {MEM_WR_STREAM}, info, DMO_Write);
    accel->statistics.countDataTraffic(false, stream.unit(), info.bytes_read());
    if (!stream.stream_active()) {
      DSA_LOG(STREAM) << stream.toString() << " freed!";
      out_port.freeStream();
    }
    DSA_LOG(MEM_REQ)
      << "write request: " << info.linebase << ", " << info.start
      << " for " << stream.toString() << " " << stream.stream_active();
    if (lws->be) {
      DSA_LOG(MEM_REQ) << "write to buffet: " << lws->be->toString();
    }
    std::ostringstream oss;
    for (int i = 0; i < data.size(); ++i) {
      oss << " " << (int) data[i];
    }
    DSA_LOG(MEM_REQ)
      << data.size() << "/" << std::min(port_available, memory_bw) << " bytes: " << oss.str();
  }

  /*!
   * \brief If all the port buffers are available to push data.
   * \param pes The ports to be checked.
   * \param bw The most bandwidth this request can sustain.
   */
  int bufferAvailable(const std::vector<PortExecState> &pes, int bw) {
    for (auto elem : pes) {
      auto &ivp = accel->input_ports[elem.port];
      bw = std::min(ivp.canPush(false), bw);
    }
    return bw;
  }

  /*!
   * \brief Reserve ongoing buffer space for the data to push.
   *        Once this value is pushed, ongoing reservation will be freed.
   */
  bool reserveBuffers(const std::vector<int> &iports, int bytes) {
    for (auto elem : iports) {
      accel->input_ports[elem].ongoing += bytes;
      DSA_LOG(PORT) << "Register " << bytes << " ongoing bytes for port " << elem;
    }
    return true;
  }

  void Visit(RecurrentStream *pps) override {
    CHECK(pps->oports.size() == 1);
    auto &ovp = accel->output_ports[pps->oports[0]];
    CHECK(pps->dtype % ovp.scalarSizeInBytes() == 0)
      << "Recurrence Element Bytes: " << pps->dtype
      << ", Port Scalar Bytes: " << ovp.scalarSizeInBytes();
    int cnt = 0;
    while (pps->stream_active() && ovp.canPop(pps->dtype) &&
           bufferAvailable(pps->pes, pps->dtype) >= pps->dtype) {
      auto data = ovp.poll(pps->dtype);
      ovp.pop(pps->dtype);
      for (auto &elem : pps->pes) {
        auto &ivp = accel->input_ports[elem.port];
        ivp.push(data, sim::stream::AffineStatus(), true);
        DSA_LOG(RECUR)
          << ivp.id() << ": " << "Packet Buffer: " << ivp.buffer.size()
          << ", Ready: " << ivp.lanesReady();
      }
      ++pps->i;
      ++cnt;
    }
    if (cnt) {
      DSA_LOG(RECUR) << accel->now() << ": " << cnt << " recur pushed " << pps->toString();
    }
    if (!pps->stream_active()) {
      DSA_LOG(STREAM) << pps->toString() << " freed!";
      ovp.freeStream();
      for (auto &elem : pps->pes) {
        accel->input_ports[elem.port].freeStream();
      }
    }
    accel->statistics.countDataTraffic(false, pps->unit(), cnt * pps->dtype);
  }

  StreamExecutor(accel_t *accel_) : accel(accel_) {}

  accel_t *accel;
};

namespace dsa {
namespace sim {

struct SPADResponser : dsa::sim::stream::Functor {

  template<typename StreamT>
  void VisitImpl(StreamT *stream) {
    is_is = true;
    std::vector<uint8_t> data;
    data = apply_mask(response.raw.data(), response.info.mask);
    DSA_LOG(MEM_REQ)
      << "SPAD response stream " << stream->id() << ", "
      << "base: " << response.info.linebase << ", "
      << "start: " << response.info.start << ", "
      << "data: " << data.size() << " bytes, "
      << "shrink: " << response.info.shrink
      << (response.info.as.stream_last ? " last!" : "");
    DSA_LOG(MEM_REQ) << stream->toString();
    for (auto &elem : stream->pes) {
      auto &ivp = accel->input_ports[elem.port];
      ivp.push(data, response.info.as, false);
      if (response.info.as.stream_last) {
        ivp.freeStream();
      }
    }
  }

  void Visit(IPortStream *ips) override {
    VisitImpl<IPortStream>(ips);
  }

  void Visit(IndirectReadStream *ips) override {
    VisitImpl<IndirectReadStream>(ips);
    if (response.info.as.stream_last) {
      auto &idxp = accel->output_ports[ips->idx_port()];
      idxp.freeStream();
    }
  }

  void Visit(LinearReadStream *lrs) override {
    Visit(static_cast<IPortStream*>(lrs));
    if (lrs->be) {
      auto new_addr = std::max(lrs->be->address, response.info.shrink);
      auto to_pop = new_addr - lrs->be->address;
      if (to_pop) {
        DSA_LOG(BUFFET) << "Buffet shrink " << lrs->be->address << " -> " << new_addr;
      }
      lrs->be->Shrink(to_pop);
    }
  }

  SPADResponser(const dsa::sim::Response &response_, accel_t *accel_) :
    response(response_), accel(accel_) {}

  bool is_is{false};
  const dsa::sim::Response &response;
  accel_t *accel;
};

}
}

void accel_t::tick() {
  if (statistics.blame != dsa::stat::Accelerator::Blame::CONFIGURE) {
    statistics.blame = dsa::stat::Accelerator::Blame::UNKNOWN;
  }

  _cgra_issued = 0; // for statistics reasons
  _dedicated_cgra_issued = 0;
  _backcgra_issued = 0;

  for (int i = 0; i < NUM_GROUPS; ++i) {
    _cgra_issued_group[i] = false;
  }

  auto scheduled = arbiter->Arbit(this);
  for (auto stream : scheduled) {
    StreamExecutor se(this);
    stream->Accept(&se);
  }

  auto spad_response = spads[0].Step();
  if (spad_response.id != -1) {
    DSA_LOG(MEM_REQ) << "Responding port: " << spad_response.id;
    if (spad_response.op == MemoryOperation::DMO_Read) {
      auto &vp = this->input_ports[spad_response.id];
      CHECK(vp.stream)
        << "Stream affiliated with the port is no longer active! " << vp.id();
      dsa::sim::SPADResponser pp(spad_response, this);
      vp.stream->Accept(&pp);
      CHECK(pp.is_is) << "Not a input stream!";
    }
  }

  _dma_c.cycle();

  // HACK: logic such that only atomic or rd/wr occurs in 1 cycle
  bool performed_atomic_scr = false;
  bool performed_read = false;

  if (_scr_ctrl_turn == 0) {
    _scr_w_c.cycle(true, performed_atomic_scr);
  }
  if (_scr_ctrl_turn == 1) {
    _scr_w_c.cycle(!performed_read, performed_atomic_scr);
  }
  _scr_ctrl_turn = (_scr_ctrl_turn + 1) % 2;
  // TODO: remove this after removing the above logic

  _net_c.cycle();

  for (auto &elem : bsw.iports()) {
    input_ports[elem.port].tick();
  }

  cycle_cgra();

  uint64_t cur_cycle = now();

  for (int i = 0; i < NUM_GROUPS; ++i) {
    std::vector<bool> &prev_issued_group = _cgra_prev_issued_group[i];
    if (prev_issued_group.size() > 0) {
      int mod_index = cur_cycle % prev_issued_group.size();
      prev_issued_group[mod_index] = _cgra_issued_group[i];
    }
  }

  if (in_roi()) {
    get_ssim()->update_stat_cycle();
  }

  for (auto &elem : bsw.oports()) {
    output_ports[elem.port].tick();
  }
  schedule_streams();
  _waiting_cycles++;

  _dma_c.finish_cycle();
  _scr_w_c.finish_cycle();
  _net_c.finish_cycle();

  if(_stream_cleanup_mode) {
    request_reset_streams();
  }

  statistics.blameCycle();
}

bool accel_t::is_shared() { return _accel_index == get_ssim()->lanes.size() - 1; }

// forward from indirect inputs to indirect outputs -- this makes the protocol
// simpler than it used to be

// Simulate the portion of the dataflow graph which requires either
// 1. backpressure, or 2. temporal sharing.
// Ports relevant for this simulation this are called "active_in_ports_bp"

std::string dumpPredicatedValues(const std::vector<uint64_t> &a, const std::vector<bool> &b) {
  CHECK(a.size() == b.size());
  ostringstream oss;
  for (int i = 0; i < (int) a.size(); ++i) {
    oss << " " << a[i] << "(" << b[i] << ")";
  }
  return oss.str();
}

void accel_t::cycle_cgra_backpressure() {

  int num_computed = 0;

  auto vps = bsw.sched->ssModel()->subModel()->vport_list();
  for (auto &elem : bsw.iports()) {
    int port_index = elem.port;
    auto &cur_in_port = input_ports[port_index];

    auto vp_iter = std::find_if(vps.begin(), vps.end(), [port_index] (ssvport *vp) {
      return vp->port() == port_index && vp->in_links().empty();
    });
    CHECK(vp_iter != vps.end());
    ssvport *vp = *vp_iter;
    auto *vec_in = dynamic_cast<dsa::dfg::InputPort*>(bsw.sched->dfgNodeOf(vp));

    if (cur_in_port.lanesReady()) {

      assert(vec_in != NULL && "input port pointer is null\n");

      if (vec_in->can_push()) {
        forward_progress();

        CHECK(cur_in_port.vectorLanes() == vec_in->values.size())
          << cur_in_port.vectorLanes() << " " << vec_in->vectorLanes()
          << " ( " << vec_in->name() << " )";

        auto data = cur_in_port.poll();
        CHECK(!data.empty()) << cur_in_port.vectorLanes();
        bool valid = false;
        std::vector<uint64_t> values;
        std::vector<bool> predicates;
        for (int i = 0; i < cur_in_port.vectorLanes(); ++i) {
          vec_in->values[i].push(data[i].value, data[i].valid, 0);
          valid |= data[i].valid;
          values.push_back(data[i].value);
          predicates.push_back(data[i].valid);
        }
        if (auto tag_port = cur_in_port.affine_tag) {
          uint8_t tag = 0;
          for (int i = 0; i < cur_in_port.vectorLanes(); ++i) {
            tag |= data[i].tag;
          }
          CHECK(tag_port->vp->values.size() == 1);
          tag_port->vp->values[0].push(tag, valid, 0);
          DSA_LOG(COMP)
            << "Push tag " << tag_port->vp->name() << ": " << std::bitset<8>(tag).to_string();
        }
        // TODO(@were): Move repeat port stuff to port pop.
        cur_in_port.pop();

        DSA_LOG(COMP)
          << now() << ": In Port: " << vec_in->name() << " allowed to push "
          << data.size() << " input(s): " << dumpPredicatedValues(values, predicates);

      }

    }
  }

  auto _dfg = bsw.sched->ssdfg();
  // calling with the default parameters for now
  num_computed = _dfg->forward(_back_cgra);
  if (num_computed) {
    statistics.blame = dsa::stat::Accelerator::Blame::COMPUTING;
  }

  if (statistics.roi()) {
    statistics.dynamic_instructions += num_computed;
  }

  _cgra_issued += _dfg->total_dyn_insts(0) + _dfg->total_dyn_insts(1);
  _dedicated_cgra_issued += _dfg->total_dyn_insts(0);
  _backcgra_issued += _dfg->total_dyn_insts(1);

  if (in_roi()) {
    _stat_ss_insts += num_computed;
    _stat_ss_dfg_util += (double)num_computed / _dfg->instructions.size();
  }

  for (auto &elem : bsw.oports()) {
    int port_index = elem.port; 
    auto &cur_out_port = output_ports[(port_index)];

    auto vp_iter = std::find_if(vps.begin(), vps.end(), [port_index] (ssvport *vp) {
      return vp->port() == port_index && vp->out_links().empty();
    });
    CHECK(vp_iter != vps.end());
    ssvport* vp = *vp_iter;
    auto *vec_output = dynamic_cast<dsa::dfg::OutputPort*>(bsw.sched->dfgNodeOf(vp));

    assert(vec_output != NULL && "output port pointer is null\n");

    if (vec_output->can_pop()) {
      forward_progress();
      vector<SBDT> data;
      vector<bool> data_valid;
      vec_output->pop(data, data_valid);

      if (in_roi()) {
        _stat_comp_instances += 1;
      }

      int dtype = cur_out_port.scalarSizeInBytes();
      for (int j = 0; j < (int) data.size(); ++j) {
        // push the data to the CGRA output port only if discard is not 0
        if (data_valid[j]) {
          std::vector<uint8_t> raw((uint8_t*)&data[j], (uint8_t*)&data[j] + dtype);
          cur_out_port.push(raw);
        } 
      }
      DSA_LOG(COMP)
        << now() << ": outvec[" << vec_output->name() << "] allowed to pop output: "
        << dumpPredicatedValues(data, data_valid)
        << ", buffered " << cur_out_port.raw.size() << " byte(s)";
    }
  }

}

void accel_t::cycle_cgra() {
  // printf("ACCEL ID: %d and cycle: %d\n", _lsq->getCpuId(), get_cur_cycle());
  if (!bsw.sched)
    return;

  // cout << "issuing cycle cgra backpressure\n";

  cycle_cgra_backpressure();

  if (in_roi()) {
    _stat_cgra_busy_cycles += (_cgra_issued > 0);
  }

  if (_cgra_issued > 0) {
    DSA_LOG(NET_REQ) << "ACCEL ID: " << lsq()->getCpuId();
  }
}

// Print out a string on one line indicating hardware status for the previous
// cycle
// Buffer Sizes                                     |      Bus Activity
// ip 1:5 2:5 7:7; op 1:2 scr_wr:1 cq:1 mem_req:14  | ip: op: scr_rd: scr_wr:
// mr:
void accel_t::cycle_status() {
  if (!bsw.sched) {
    return;
  }

  // timestamp();
  // cout << "cq" << _cmd_queue.size();

  // for (int group = 0; group < NUM_GROUPS; ++group) {
  //   auto &active_ports = _soft_config.in_ports_active_group[group];
  //   if (active_ports.size()) {
  //     cout << "|";
  //     for (unsigned i = 0; i < active_ports.size(); ++i) {
  //       unsigned cur_p = active_ports[i];
  //       cout << "i" << cur_p << ":" << _port_interf.in_port(cur_p).mem_size()
  //            << "," << _port_interf.in_port(cur_p).num_ready();
  //       auto &in_port = _port_interf.in_port(active_ports[i]);
  //       if (in_port.in_use()) {
  //         cout << LOC_NAME[in_port.loc()];
  //         if (in_port.completed()) {
  //           cout << "#";
  //         }
  //       }
  //       cout << " ";
  //     }
  //   }
  // }

  // cout << "\t";
  // for (int group = 0; group < NUM_GROUPS; ++group) {
  //   auto &active_ports = _soft_config.out_ports_active_group[group];
  //   if (active_ports.size()) {
  //     cout << "|";
  //     for (unsigned i = 0; i < active_ports.size(); ++i) {
  //       unsigned cur_p = active_ports[i];
  //       auto &out_port = _port_interf.out_port(cur_p);
  //       cout << "o" << cur_p << ":" << out_port.num_in_flight() << "-"
  //            << out_port.num_ready() << "," << out_port.mem_size();
  //       if (out_port.in_use()) {
  //         cout << LOC_NAME[out_port.loc()];
  //       }
  //       cout << " ";
  //     }
  //   }
  // }

  // cout << "m_req:" << _dma_c.mem_reqs() << " ";

  // cout << "\t|";

  // cout << "s_rd" << _stat_scr_bytes_rd << " s_wr:" << _stat_scr_bytes_wr
  //      << " m_rd:" << _stat_mem_bytes_rd << " m_wr:" << _stat_mem_bytes_wr
  //      << " ";

  // // Just the indirect ports
  // //  for(unsigned i = 24; i < 32; ++i) {
  // //    int cur_p=i;
  // //    if(_port_interf.in_port(cur_p).in_use()) {
  // //      cout << cur_p << " "  <<
  // //      (_port_interf.in_port(cur_p).completed()?"(completed)":"");
  // //    }
  // //  }

  // // Just the indirect ports
  // //  for(unsigned i = 24; i < 32; ++i) {
  // //    int cur_p=i;
  // //    if(_port_interf.out_port(cur_p).in_use()) {
  // //      cout << cur_p << " " <<
  // //      (_port_interf.out_port(cur_p).completed()?"(completed)":"");
  // //    }
  // //  }

  // clear_cycle();
}

void accel_t::cycle_status_backcgra() {
  // if (_soft_config.in_ports_active_backcgra.size() == 0) {
  //   return;
  // }

  // timestamp();
  // cout << "cq" << _cmd_queue.size();
  // auto &active_in_ports = _soft_config.in_ports_active_backcgra;

  // if (active_in_ports.size()) {
  //   cout << "|";
  //   for (unsigned i = 0; i < active_in_ports.size(); ++i) {
  //     unsigned cur_p = active_in_ports[i];
  //     cout << "i" << cur_p << ":" << _port_interf.in_port(cur_p).mem_size()
  //          << "," << _port_interf.in_port(cur_p).num_ready();
  //     auto &in_port = _port_interf.in_port(active_in_ports[i]);
  //     if (in_port.in_use()) {
  //       cout << LOC_NAME[in_port.loc()];
  //       if (in_port.completed()) {
  //         cout << "#";
  //       }
  //     }
  //     cout << " ";
  //   }
  // }

  // cout << "\t";
  // auto &active_out_ports = _soft_config.out_ports_active;
  // if (active_out_ports.size()) {
  //   cout << "|";
  //   for (unsigned i = 0; i < active_out_ports.size(); ++i) {
  //     unsigned cur_p = active_out_ports[i];
  //     auto &out_port = _port_interf.out_port(cur_p);
  //     cout << "o" << cur_p << ":" << out_port.num_in_flight() << "-"
  //          << out_port.num_ready() << "," << out_port.mem_size();
  //     if (out_port.in_use()) {
  //       cout << LOC_NAME[out_port.loc()];
  //     }
  //     cout << " ";
  //   }
  // }

  // cout << "m_req:" << _dma_c.mem_reqs() << " ";

  // cout << "\t|";

  // //  cout << "req:"
  // cout << "s_rd" << _stat_scr_bytes_rd << " s_wr:" << _stat_scr_bytes_wr
  //      << " m_rd:" << _stat_mem_bytes_rd << " m_wr:" << _stat_mem_bytes_wr
  //      << " ";
  // //  cout << "sat:" << " m_rd:" << _stat_mem_bytes_rd_sat << " ";
  // //                 << " m_wr:" << _stat_mem_bytes_wr_sat;

  // // Just the indirect ports
  // for (unsigned i = 24; i < 32; ++i) {
  //   int cur_p = i;
  //   if (_port_interf.in_port(cur_p).in_use()) {
  //     cout << cur_p << " "
  //          << (_port_interf.in_port(cur_p).completed() ? "(completed)" : "");
  //   }
  // }

  // // Just the indirect ports
  // for (unsigned i = 24; i < 32; ++i) {
  //   int cur_p = i;
  //   if (_port_interf.out_port(cur_p).in_use()) {
  //     cout << cur_p << " "
  //          << (_port_interf.out_port(cur_p).completed() ? "(completed)" : "");
  //   }
  // }
  // clear_cycle();
}

void accel_t::clear_cycle() {

  _stat_mem_bytes_wr = 0;
  _stat_mem_bytes_rd = 0;
  _stat_scr_bytes_wr = 0;
  _stat_scr_bytes_rd = 0;
  _stat_mem_bytes_wr_sat = 0;
  _stat_mem_bytes_rd_sat = 0;

}

void accel_t::print_status() {
  bool should_return=true;

  // TODO(@were): Support this.
  // if(!lsq()->spu_net_done()) {
  //   should_return = false;
  //   cout << "SPU net not done\n";
  // }

  // TODO(@were): Support this.
  // if(!lsq()->all_spu_done(_ssim->num_active_threads())) {
  //   should_return=false;
  //   cout << "All SPU cores not done\n";
  // }

  if (done(false, -1)) {
    return;
  }

  auto _dfg = bsw.sched ? bsw.sched->ssdfg() : nullptr; 
  if (should_return && !_dfg) {
    std::cout << "No DFG configured\n";
    return;
  }

  cout << "Core: " << lsq()->getCpuId() << endl;
  // _dfg->print_status();

  /*if(!_lsq->all_spu_done(_ssim->num_active_threads())) {
    cout << "All SPU cores not done with threads: " << _ssim->num_active_threads() << "\n";
  }*/
  cout << "---- ACCEL " << _accel_index << " STATUS ----\n";
  cout << "MEM REQs OUTSTANDING: " << _dma_c.mem_reqs() << "\n";
  cout << "Active SEs:\n";
  for (auto elem : bsw.iports()) {
    if (auto stream = input_ports[elem.port].stream) {
      cout << stream->toString() << std::endl;
    }
  }
  for (auto elem : bsw.oports()) {
    if (auto stream = output_ports[elem.port].stream) {
      cout << stream->toString() << std::endl;
    }
  }
  _dma_c.print_status();
  _scr_r_c.print_status();
  _scr_w_c.print_status();
  _net_c.print_status();

  cout << "Non-empty buffers:\n";
  if(!lsq()->is_pending_net_empty()) {
    cout << "SPU network requests pending in queue\n";
  }

  cout << "Ports:\n";
  for (auto &elem : bsw.iports()) {
    unsigned cur_p = elem.port;
    string s = bsw.name(true, cur_p);
    std::cout << "In Port " << cur_p << " " << s << ": ";
    std::cout << "  Mem Size: " << input_ports[cur_p].lanesReady() << " "
              << "  Num Ready: " << input_ports[cur_p].buffer.size()
              << std::endl;
  }


  for (auto &elem : bsw.oports()) {
    unsigned cur_p = elem.port;
    string s = bsw.name(0, cur_p);
    std::cout
      << "Out Port " << cur_p << " " << s << ": "
     << "  Buffer: " << output_ports[cur_p].raw.size() << std::endl;
  }

  std::cout << "Atomic scr val port: " << _scr_w_c.atomic_val_size() << "\n";
  std::cout << "Pending request queue: " << _scr_w_c.pending_request_queue_size() << "\n";
  std::cout << "Conflict detection queue: " << _scr_w_c.conflict_queue_size() << "\n";

  done(true, 0); // print why not done

  // TODO: how do I do this? (should be recursive)
  for(int i=0; i<_ssim->num_active_threads(); ++i) {
    lsq()->print_spu_stats(i);
  }

}

void accel_t::pedantic_statistics(std::ostream &out) {
  double l2_acc_per_cyc =
      ((double)(_stat_tot_loads + _stat_tot_stores)) / ((double)roi_cycles());
  out << "L2 accesses per cycle: " << l2_acc_per_cyc << "\n\n";
  double l2_miss_per_cyc =
      ((double)(_stat_tot_mem_load_acc + _stat_tot_mem_store_acc)) /
      ((double)roi_cycles());
  out << "L2 misses per cycle:       " << l2_miss_per_cyc << "\n\n";

  out << "CGRA Activity Histogram (inst/switch:times used)\n";
  for (auto i : _total_histo) {
    out << name_of_inst(i.first) << ":" << i.second << "\n";
  }
  out << "\n Port Activity Histogram (size:times used)\n ";
  for (auto i : _vport_histo) {
    out << i.first << ":" << i.second << "\n";
  }

  out << "\n Stream Length Statistics\n";
  _stream_stats.print(out);

  out.flush();
}

uint64_t accel_t::roi_cycles() { return _ssim->roi_cycles(); }

void accel_t::print_statistics(std::ostream &out) {

  out << "\nACCEL " << _accel_index << " STATS ***\n";
  out.precision(4);
  out << dec;
  // out << "Start Cycle: " << _stat_start_cycle << "\n";
  // out << "Stop  Cycle: " << _stat_stop_cycle << "\n\n";
  // out << "BYTES READ AT PORT 5: " << _bytes_rd5 << "\n";

  out << "Commands Issued: " << statistics.commands_issued << "\n";
  out << "CGRA Instances: " << _stat_comp_instances << " -- Activity Ratio: "
      << ((double)_stat_cgra_busy_cycles) / ((double)roi_cycles())
      << ", DFGs / Cycle: "
      << ((double)_stat_comp_instances) / ((double)roi_cycles()) << "\n";
  if (bsw.dfg()) {
    out << "For backcgra, Average thoughput of all ports (overall): "
      << ((double)_stat_comp_instances)/((double)roi_cycles() * bsw.dfg()->vouts.size()) // gives seg fault when no dfg
      << ", CGRA outputs/cgra busy cycles: "
      <<  ((double)_stat_comp_instances)/((double)_stat_cgra_busy_cycles)  << "\n";
  }
  out << "CGRA Insts / Computation Instance: "
      << ((double)_stat_ss_insts) / ((double)_stat_comp_instances) << "\n";
  {
    double ilp = ((double)(statistics.dynamic_instructions)) / get_ssim()->statistics.cycleElapsed();
    out << "CGRA Insts / Cycle: "
        << (statistics.dynamic_instructions) << " / " << ((int) get_ssim()->statistics.cycleElapsed())
        << " = " << ilp << " (overall activity factor)\n";
  }
  out << "Mapped DFG utilization: "
      << ((double)_stat_ss_dfg_util) / ((double)roi_cycles()) << "\n";
  // FIXME: see it's use
  out << "Data availability ratio: "
      << ((double)_stat_ss_data_avail_ratio) / ((double)roi_cycles()) << "\n";
  // out << "Atomic scr executed cycles: "
  //     << _stat_cycles_atomic_scr_executed << "\n";
  // out << "Atomic scr issued cycles: "
  //     << _num_cycles_issued << "\n";
  
  // out << "Memory initiation interval: "
  //   << _stat_mem_initiation_interval << "\n";
   
  out << "input port imbalance (%age dgra nodes could not fire): "
      << ((double)_stat_port_imbalance) / ((double)_stat_cgra_busy_cycles) << "\n";
      // << ((double)_stat_port_imbalance) / ((double)roi_cycles()) << "\n";
   out << "Percentage bank conflicts: "
      << ((double)_stat_cycles_atomic_scr_executed) /
             _stat_cycles_atomic_scr_pushed
      << "\n";
  out << "L1 cache hit rate: "
      << ((double)_stat_hit_bytes_rd) / ((double)(_stat_hit_bytes_rd+_stat_miss_bytes_rd)) << "\n";
  out << "Avg wait cycles on a byte read: "
      << ((double)_stat_tot_mem_wait_cycles) / ((double)_stat_mem_bytes_rd) << "\n";
  out << "Percentage cycles spent due to conflict in atomic scr: " 
      << ((double)_stat_conflict_cycles) / ((double)_stat_tot_atom_cycles) << "\n";



  // out << "Allowed input port consumption rate: ";
  // for (int i = 0; i < NUM_OUT_PORTS; ++i) {
  //   out << ((double)_slot_avail[i] / (double)roi_cycles()) << ", ";
  // }
  // out << "\n";
  // out << "percentage time we could not serve input ports: ";
  // for (int i = 0; i < NUM_OUT_PORTS; ++i) {
  //   out << ((double)_could_not_serve[i] / (double)roi_cycles()) << ", ";
  // }
  // out << "\n";

  auto cycles = this->get_ssim()->statistics.cycleElapsed();
  out << "Cycle Breakdown:\n";
  for (int i = 0; i < dsa::stat::Accelerator::Blame::UNKNOWN; ++i) {
    out << BLAME_NAME[i] << ":\t" << (statistics.blame_count[i] / cycles) * 100
        << "%(" << statistics.blame_count[i] << ")" << std::endl;
  }

  auto print_component = [this, &out, &cycles](const std::string &name, int x, int y) {
    CHECK(x >= 0 && x < 2);
    CHECK(y >= 0 && y < LOC::TOTAL);
    auto &traffic = this->statistics.traffic[x][y];
    auto dpc = traffic.traffic / cycles;
    auto dpr = (double) traffic.traffic / traffic.num_requests;
    out << name << ":\t" << traffic.traffic << " B"
        << " (" << dpc << " B/c" << ", "
        <<  dpr << " B/r" << ") " << traffic.num_requests << std::endl;
  };

  auto print_request = [this, &out, &cycles](const std::string &name, int x, int y, int bw) {
    CHECK(x >= 0 && x < 2);
    CHECK(y >= 0 && y < LOC::TOTAL);
    auto &traffic = this->statistics.traffic[x][y];
    int64_t request_traffic = traffic.num_requests * bw;
    auto dpc = request_traffic / cycles;
    out << name << ":\t" << request_traffic << " B" << " (" << dpc << " B/c)" << std::endl;
  };

  out << "Bandwidth Table: (B/c=Bytes/cycle, B/r=Bytes/request) -- Breakdown "
         "(sources/destinatinos): \n";
  print_component("Read SPAD", true, LOC::SCR);
  print_request("R/Request SPAD", true, LOC::SCR, spads[0].bandwidth());
  print_component("Write SPAD", false, LOC::SCR);
  print_request("W/Request SPAD", false, LOC::SCR, spads[0].bandwidth());
  print_component("Read DMA", true, LOC::DMA);
  print_request("R/Request DMA", true, LOC::DMA, get_ssim()->spec.dma_bandwidth);
  out
    << "Avg DMA Latency: " << statistics.averageMemoryLatency() << " cycles "
    << ", " << statistics.averageImpl(statistics.mem_lat_brkd[Minor::LSQ::LSQRequest::LSQRequestState::InTranslation])
    << ", " << statistics.averageImpl(statistics.mem_lat_brkd[Minor::LSQ::LSQRequest::LSQRequestState::Translated])
    << ", " << statistics.averageImpl(statistics.mem_lat_brkd[Minor::LSQ::LSQRequest::LSQRequestState::RequestIssuing])
    << ", " << statistics.averageImpl(statistics.mem_lat_brkd[Minor::LSQ::LSQRequest::LSQRequestState::Complete]) << "\n";
  print_component("Write DMA", false, LOC::DMA);
  print_request("W/Request DMA", false, LOC::DMA, get_ssim()->spec.dma_bandwidth);
  print_component("Recur Bus", false, LOC::REC_BUS);
  
}

// wait and print stats
void accel_t::print_stats() {
  print_statistics(std::cout);
  print_status();

  ofstream stat_file;
  if (char *name = getenv("SS_RUN_NAME")) {
    stat_file.open(string("stats/") + name + ".ss-stats",
                   ofstream::trunc | ofstream::out);
  } else {
    stat_file.open("stats/default.ss-stats", ofstream::trunc | ofstream::out);
  }

  assert(stat_file.is_open());
  print_statistics(stat_file);
  pedantic_statistics(stat_file);
}

// pushed implicit stream in cmd queue -- I guess not being issued
void accel_t::push_net_in_cmd_queue(base_stream_t *s) {
  // _ssim->BroadcastStream(s);
  _ssim->cmd_queue.push_back(s);
}

// --------------------------SCHEDULE STREAMS ONTO
// CONTROLLERS----------------------- This is essentially the stream dispatcher
void accel_t::schedule_streams() {

  // FIXME: check with Tony!
  if(_cleanup_mode) return;

    // if (auto stream = dynamic_cast<remote_core_net_stream_t *>(ip)) {
    //   // should I check if those ports not busy (Although they should not be)
    //   // out_vp = &_port_interf.out_port(NET_ADDR_PORT);
    //   // out_vp2 = &_port_interf.out_port(NET_VAL_PORT);
    //   // assert(out_vp->can_take(LOC::PORT) && !blocked_ovp[NET_ADDR_PORT]);
    //   // assert(out_vp2->can_take(LOC::PORT) && !blocked_ovp[NET_VAL_PORT]);
    //   // scheduled = _scr_w_c.schedule_network_stream(*stream);
    //   // out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   // out_vp2->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    // } else if (auto stream = dynamic_cast<remote_port_stream_t *>(ip)) {
    //   // if (stream->_is_source) { // do not schedule, but check out port available
    //   //   int out_port = stream->_out_port;
    //   //   out_vp = &_port_interf.out_port(out_port);

    //   //   if ((scheduled = (out_vp->can_take(LOC::PORT) && !blocked_ovp[out_port]))) {
    //   //     out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   //     stream->_remote_stream->_is_ready = true; // we'll check this in dest.
    //   //   }
    //   // } else { // destination, time for action!
    //   //   if (ivps_can_take && stream->_is_ready /*src is ready*/) {
    //   //     scheduled = _port_c.schedule_remote_port(*stream);
    //   //   }
    //   // }
    // } else if (auto rem_scr_stream =
    //                dynamic_cast<direct_remote_scr_stream_t *>(ip)) {
    //   // int val_port = rem_scr_stream->_out_port;

    //   // out_vp = &_port_interf.out_port(val_port); // this is in data out port

    //   // if (out_vp->can_take(LOC::PORT) && !blocked_ovp[val_port] &&
    //   //     (scheduled = _net_c.schedule_direct_remote_scr(*rem_scr_stream))) {
    //   //   out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   // }
    // } else if (auto ind_rem_scr_stream =
    //                dynamic_cast<remote_scr_stream_t *>(ip)) {
    //   // int addr_port = ind_rem_scr_stream->_addr_port;
    //   // int val_port = ind_rem_scr_stream->_out_port;

    //   // out_vp = &_port_interf.out_port(addr_port); // this is addr out port
    //   // out_vp2 = &_port_interf.out_port(val_port); // this is in data out port

    //   // if (out_vp->can_take(LOC::PORT) && !blocked_ovp[addr_port] &&
    //   //     out_vp2->can_take(LOC::PORT) && !blocked_ovp[val_port] &&
    //   //     (scheduled = _net_c.schedule_remote_scr(*ind_rem_scr_stream))) {
    //   //   out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   //   out_vp2->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   // }
    // } else if (auto rem_port_multicast_stream =
    //                dynamic_cast<remote_port_multicast_stream_t *>(ip)) {
    //   // make sure the base stream is lower in the comparison
    //   // int out_port = rem_port_multicast_stream->_out_port;
    //   // out_vp = &_port_interf.out_port(out_port); // this is data output port

    //   // if (out_vp->can_take(LOC::PORT) && !blocked_ovp[out_port] &&
    //   //     (scheduled = _net_c.schedule_remote_port_multicast(
    //   //          *rem_port_multicast_stream))) {
    //   //   // printf("Remote port stream is scheduled\n");
    //   //   out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   // }
    // } else if (auto atomic_scr_stream =
    //                dynamic_cast<atomic_scr_stream_t *>(ip)) {
    //   // int addr_port = atomic_scr_stream->_out_port;
    //   // int val_port = atomic_scr_stream->_val_port;

    //   // out_vp = &_port_interf.out_port(addr_port); // this is addr output port
    //   // out_vp2 =
    //   //     &_port_interf.out_port(val_port); // this is increment value port

    //   // if (out_vp->can_take(LOC::PORT) && out_vp2->can_take(LOC::PORT) &&
    //   //     !blocked_ovp[addr_port] && !blocked_ovp[val_port] &&
    //   //     (scheduled = _scr_w_c.schedule_atomic_scr_op(*atomic_scr_stream))) {
    //   //   out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   //   out_vp2->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   // }
    // } else if (auto const_scr_stream = dynamic_cast<const_scr_stream_t *>(ip)) {
    //   // scheduled = _scr_w_c.schedule_const_scr(*const_scr_stream);
    // }

    // // prevent out-of-order access
    // if (auto is = dynamic_cast<IPortStream*>(ip)) {
    //   for (auto &in_port : is->pes) {
    //     blocked_ivp[in_port.port] = true;
    //   }
    // }
    // if (out_vp) {
    //   blocked_ovp[out_vp->port()] = true;
    // }
    // if (out_vp2) {
    //   blocked_ovp[out_vp2->port()] = true;
    // }

    // prior_scratch_read |= ((ip->src() & LOC::SCR) != LOC::NONE);
    // prior_scratch_write |= ((ip->dest() & LOC::SCR) != LOC::NONE);

    // if (scheduled) {

    //   if (auto is = dynamic_cast<IPortStream*>(ip)) {
    //     for (auto &in_port : is->pes) {
    //       port_data_t *in_vp = &_port_interf.in_port(in_port.port);
    //       in_vp->set_status(port_data_t::STATUS::BUSY, ip->unit());
    //       // it means different for recurrence stream: confirm if this makes sense
    //       in_vp->set_repeat(repeat, repeat_str, repeat_flag);
    //     }
    //   }

    //   str_issued++;
    //   DSA_LOG(COMMAND_I) << get_ssim()->CurrentCycle()
    //                  << " ISSUED \tid:" << ip->id() << " ";
    //   if (SS_DEBUG::COMMAND_I) {
    //     // TODO(@were): Change this to toString()
    //     ip->print_status();
    //   }

    //   i = _cmd_queue.erase(i);
    //   if (_ssim->in_roi()) {
    //     _stat_commands_issued++;
    //   }
    // } else {
    //   // we failed to schedule anything!
    //   if (_ssconfig->dispatch_inorder()) { // if INORDER-dispatch, stop!
    //     break;
    //   } else { // if we're in OOO-dispatch mode, just keep going...
    //     ++i;
    //   }
    // }
}

void data_controller_t::add_bw(LOC l1, LOC l2, uint64_t times, uint64_t bytes) {
  _accel->add_bw(l1, l2, times, bytes);
}

ssim_t *data_controller_t::get_ssim() { return _accel->get_ssim(); }

bool data_controller_t::is_shared() { return _accel->is_shared(); }

void data_controller_t::timestamp() { _accel->timestamp(); }

// For port->remote port multicast affine stream
bool network_controller_t::schedule_remote_port_multicast(
    remote_port_multicast_stream_t &new_s) {
  auto *s = new remote_port_multicast_stream_t(new_s);
  _remote_port_multicast_streams.push_back(s);
  _remote_streams.push_back(s);
  return true;
}

// For port->remote scr indirect stream
bool network_controller_t::schedule_remote_scr(remote_scr_stream_t &new_s) {
  auto *s = new remote_scr_stream_t(new_s);
  _remote_scr_streams.push_back(s);
  _remote_streams.push_back(s);
  return true;
}

// For port->remote scr direct stream
bool network_controller_t::schedule_direct_remote_scr(
    direct_remote_scr_stream_t &new_s) {
  auto *s = new direct_remote_scr_stream_t(new_s);
  _direct_remote_scr_streams.push_back(s);
  _remote_streams.push_back(s);
  return true;
}

bool scratch_write_controller_t::schedule_network_stream(
    remote_core_net_stream_t &new_s) {
  auto *s = new remote_core_net_stream_t(new_s);
  _network_streams.push_back(s);
  _write_streams.push_back(s);
  return true;
}

// Atomic stream update: only 1 stream is allowed to be doing this at a time!
bool scratch_write_controller_t::schedule_atomic_scr_op(
    atomic_scr_stream_t &new_s) {
  auto *s = new atomic_scr_stream_t(new_s);
  _atomic_scr_streams.push_back(s);
  _write_streams.push_back(s);
  return true;
}

void apply_map(uint8_t *raw_data, const vector<int> &imap,
               std::vector<uint8_t> &data) {
  assert(imap.size() != 0);
  data.resize(imap.size());
  for (int i = 0; i < imap.size(); ++i) {
    // cout << "at imap[" << imap[i] << "], data=" << raw_data[imap[i]] << "\n";
    data[i] = raw_data[imap[i]];
  }
}

void dma_controller_t::port_resp(unsigned cur_port) {
  if (Minor::LSQ::LSQRequestPtr response =
          _accel->lsq()->findResponse(cur_port)) {

    // First check if we haven't discared the memory request
    // this will only be the case if reqs are equal to zero
    if (_accel->_cleanup_mode) {
      _accel->lsq()->popResponse(cur_port);
      _mem_read_reqs--;
      return;
    }

    if (_accel->_accel_index == response->sdInfo->which_accel) {

      PacketPtr packet = response->packet;
      CHECK(packet->getSize() == get_ssim()->spec.dma_bandwidth)
        << packet->getSize() << " " << get_ssim()->spec.dma_bandwidth;

      vector<uint8_t> data;
      if (response->sdInfo->mask.size() > 0) {
        data = apply_mask(packet->getPtr<uint8_t>(), response->sdInfo->mask);
      } else if (response->sdInfo->map.size() > 0) {
        apply_map(packet->getPtr<uint8_t>(), response->sdInfo->map, data);
      }

      //-----------------

      bool port_in_okay = true;

      // push in byte-by-byte at the ports
      for (int in_port : response->sdInfo->ports) {
        auto &in_vp = _accel->input_ports[in_port];
        port_in_okay = port_in_okay && in_vp.buffer.size() < in_vp.buffer_size;
      }

      if (port_in_okay) {
        bool last = response->sdInfo->as.stream_last;

        std::ostringstream oss;
        for (int i = 0, n = data.size(); i < n; ++i) {
          oss << " " << ((int) data[i]);
        }

        DSA_LOG(MEM_REQ)
          << get_ssim()->CurrentCycle() << " response for "
          << std::hex << packet->getAddr() << std::dec
          << "for port " << cur_port << ", size in bytes: "
          << data.size() << " elements" << (last ? "(last)" : "") << oss.str();

        // FIXME: check if stats are reset at roi
        // _accel->_stat_mem_bytes_rd += data.size();
 
        // request for all added ports
        for (int in_port : response->sdInfo->ports) {
          auto &in_vp = _accel->input_ports[in_port];
          in_vp.push(data, response->sdInfo->as, false);
          if (last) {
            DSA_LOG(STREAM) << in_vp.stream->toString() << " freed!";
            if (auto irs = dynamic_cast<IndirectReadStream*>(in_vp.stream)) {
              auto &out_vp = _accel->output_ports[irs->oports[0]];
              out_vp.freeStream();
            }
            in_vp.freeStream();
          }
          DSA_LOG(MEM_REQ)
            << _accel->bsw.iports()[in_port].vp->name()
            << " buffers " << in_vp.buffer.size() << " element(s)";
        }


        _accel->statistics.countMemoryLatency(response->sdInfo->request_cycle,
                                              response->sdInfo->breakdown);

        // cache hit stats collection
        if(_accel->_ssim->in_roi()) {
          _accel->_stat_tot_mem_wait_cycles += (_accel->get_cur_cycle()-response->sdInfo->request_cycle);
          _accel->_stat_mem_bytes_rd += data.size();
          if(_accel->get_cur_cycle()-response->sdInfo->request_cycle<20) { // probably L1 hit
            // cout << "L1 hit\n";
            _accel->_stat_hit_bytes_rd += data.size();
          } else {
            // cout << "L1 miss\n";
            _accel->_stat_miss_bytes_rd += data.size();
          } 
        }

        if(_accel->_ssim->in_roi()) {
          _accel->_stat_mem_bytes_rd += data.size();
        }
        data.clear();
        _accel->lsq()->popResponse(cur_port);

        _mem_read_reqs--;
        return;
      }
    }
  }
}

// ---------------------STREAM CONTROLLER TIMING
// ------------------------------------ If response, can issue load Limitations:
// 1 Response per cycle (512 bits/cycle)
void dma_controller_t::cycle() {
  // Memory read to config
  if (Minor::LSQ::LSQRequestPtr response =
          _accel->lsq()->findResponse(CONFIG_STREAM)) {
    PacketPtr packet = response->packet;
    uint64_t context = response->sdInfo->which_accel;
    for (int i = 0; i < (int) _accel->get_ssim()->lanes.size(); ++i) {
      if (context >> i & 1) {
        _accel->_ssim->lanes[i]->configure(packet->getAddr(),
                                               packet->getSize() / 8,
                                               packet->getPtr<uint64_t>());
        _accel->statistics.blame = dsa::stat::Accelerator::UNKNOWN;
      }
    }
    _accel->lsq()->popResponse(CONFIG_STREAM);
  }

  // Memory Read to Ports
  for (unsigned i = 0; i < _accel->bsw.iports().size(); ++i) {
    int cur_port = _accel->bsw.iports()[i].port;
    port_resp(cur_port);
  }

}

void dma_controller_t::print_status() {}

void dma_controller_t::finish_cycle() {}

// Delete Streams implementation
template <typename T>
void delete_stream_internal(int i, T *s, std::vector<T *> &vec,
                            std::vector<base_stream_t *> &base_vec) {
 
  base_vec.erase(base_vec.begin() + i);
  vec.erase(std::remove(vec.begin(), vec.end(), s), vec.end());
}


void network_controller_t::delete_stream(int i,
                                         remote_port_multicast_stream_t *s) {
  delete_stream_internal(i, s, _remote_port_multicast_streams, _remote_streams);
}
void network_controller_t::delete_stream(int i, remote_scr_stream_t *s) {
  delete_stream_internal(i, s, _remote_scr_streams, _remote_streams);
}
void network_controller_t::delete_stream(int i, direct_remote_scr_stream_t *s) {
  delete_stream_internal(i, s, _direct_remote_scr_streams, _remote_streams);
}

void scratch_write_controller_t::delete_stream(int i, atomic_scr_stream_t *s) {
  delete_stream_internal(i, s, _atomic_scr_streams, _write_streams);
}

#define MAX_PORT_READY 100000


// It works for both direct/indirect network streams
void scratch_write_controller_t::write_scratch_remote_ind(
    remote_core_net_stream_t &stream) {
// TODO(@were): Bring inter-core writing back.
//  auto &addr_vp = _accel->output_ports[stream._addr_port];
//  auto &val_vp = _accel->output_ports[stream._val_port];
//
//  int bytes_written = 0;
//  SBDT val[8];
//  while(addr_vp.mem_size() && val_vp.mem_size() && stream.stream_active() &&
//      bytes_written < 64) {
//    uint64_t meta_info = addr_vp.pop_out_data(); // no offset here
//    // this addr should have both num_bytes and addr info
//    addr_t addr = meta_info & 65535; // for 16-bits
//    DSA_LOG(NET_REQ) << "addr is: " << addr << "\n";
//    int num_bytes = meta_info >> 16;
//    assert(num_bytes <= 64);
//    for (int j = 0; j < num_bytes / 8; ++j) {
//      val[j] = val_vp.pop_out_data();
//      DSA_LOG(NET_REQ) << now() << " val being written to remote scratchpad: " << val[j];
//    }
//    _accel->write_scratchpad(addr, &val[0], num_bytes, stream.id());
//    bytes_written += num_bytes;
//  }
//  _remote_scr_writes += bytes_written;
//
//  bool is_empty = !stream.stream_active();
//  if (is_empty) {
//    _accel->process_stream_stats(stream);
//
//    DSA_LOG(VP_SCORE2) << "SOURCE: INDIRECT PORT -> SCR\n";
//    addr_vp.freeStream();
//    val_vp.freeStream();
//  }
//
//  if (_accel->_ssim->in_roi()) {
//    add_bw(LOC::PORT, LOC::SCR, 1, bytes_written);
//    _accel->_stat_scratch_writes++;
//  }
//
//  _accel->_stat_scr_bytes_wr += bytes_written;
//  _accel->_stat_scratch_write_bytes += bytes_written;
}


void scratch_read_controller_t::push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry) {
  DSA_LOG(NET_REQ)
    << "Read request reached with requesting core: "
    << req_core << " x dim: " << request_ptr << " y dim: "
    << reorder_entry << " and addr: " << addr << " data bytes: " << data_bytes;
  int num_cache_line_reqs = std::max(1, data_bytes/NUM_SCRATCH_BANKS);
  int local_bytes = std::min(data_bytes, NUM_SCRATCH_BANKS);
  int logical_banks = NUM_SCRATCH_BANKS / local_bytes; // ->data_bytes;
  for(int i=0; i<num_cache_line_reqs; ++i) {
    indirect_scr_read_req request;
    request.data_ptr = request_ptr; // X loc
    request.addr = addr+i*64;
    request.bytes = local_bytes;
    request.irob_entry_id = (reorder_entry+i)%(_accel->_ind_rob_size); // Y-dim
    request.remote = is_remote;
    request.req_core = req_core;


    // Assuming linear mapping by default
    int bank_id = request.addr & (logical_banks - 1);

    DSA_LOG(NET_REQ)
      << "Remote read request at bank: " << bank_id << " and core: "
      << _accel->lsq()->getCpuId() << " for addr: " << request.addr;
    assert(request.addr<SCRATCH_SIZE);
    _indirect_scr_read_requests[bank_id].push(request);
  }

}

// write the data into IROB (no arbitration modeled)
void scratch_read_controller_t::push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    DSA_LOG(NET_REQ)
      << "Pushing the returned data of indirect read with x dim: " << request_ptr
      << " y dim: " << reorder_entry << " addr: " << addr << " data bytes: " << data_bytes
      << "Currently at core: " << _accel->lsq()->getCpuId();
    auto it = _reorder_entry_id.find(reorder_entry);
    assert(it!=_reorder_entry_id.end() && "entry should have been created in unordered map");
    ind_reorder_entry_t *x = _reorder_entry_id[reorder_entry];
    // cout << "Being served, request ptr: " << request_ptr << " data bytes: " << data_bytes << endl;
    assert(request_ptr+data_bytes<=64 && "reorder entry can only be 64 bytes");
    std::memcpy(&x->data[request_ptr], data, data_bytes);
    // cout << "Initial completed: " << x->completed << " and data bytes: " << data_bytes << endl;
    x->completed += data_bytes;
}

void scratch_read_controller_t::serve_ind_read_banks() {
  // cout << "Inside serve ind banks function\n";
  int bytes_read = 0;
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (!_indirect_scr_read_requests[i].empty()) {

      indirect_scr_read_req request = _indirect_scr_read_requests[i].front();
      // cout << "Identified a request at indirect scr bank which is remote? " << request.remote << endl;

      if(request.remote) {
          int8_t return_data[64]; // 8 byte*64 (uint64_t is 8 bytes)
          void * copy_addr = &return_data[0];
          // FIXME: stream id is incorrect, should we send it over network
          // or just keep default stream id
    
          _accel->read_scratchpad(copy_addr, request.addr, request.bytes,0);
                                  // reorder_entry.stream->id());

          _accel->lsq()->push_rem_read_return(request.req_core, return_data, request.data_ptr, request.addr, request.bytes, request.irob_entry_id);
      } else {
          ind_reorder_entry_t &reorder_entry = *_reorder_entry_id[request.irob_entry_id];
          void* copy_addr = reorder_entry.data + request.data_ptr;
          _accel->read_scratchpad(copy_addr, request.addr, request.bytes,
                              reorder_entry.stream->id());

          reorder_entry.completed += reorder_entry.data_bytes;
          // assert(reorder_entry.size > 0 && reorder_entry.size <= 64);
          // assert(reorder_entry.completed <= reorder_entry.size);
      }

      // bytes_read += reorder_entry.data_bytes;
      bytes_read += request.bytes;
      _indirect_scr_read_requests[i].pop();
    }
  }

  if (bytes_read > 0) {
    _accel->_stat_scr_bytes_rd += bytes_read;
    _accel->_stat_scratch_read_bytes += bytes_read;
    if (_accel->_ssim->in_roi()) {
      add_bw(LOC::SCR, LOC::PORT, 1, bytes_read);
      _accel->_stat_scratch_reads++;
    }
  }


  // Stage 3: See if we can pop an item from the indirect rob
  // int bytes_pushed=0;
  if (_ind_ROB.size() > 0) {
    ind_reorder_entry_t *reorder_entry = _ind_ROB.front();

    if (reorder_entry->size == reorder_entry->completed) {
      // The entry is ready for transfer!
      
      auto &stream = *reorder_entry->stream;

      for (int i = 0; i < reorder_entry->size; ++i) {
        uint8_t data_byte = reorder_entry->data[i];
        (void) data_byte;
        // TODO(@were): Implement this.
        // for (int in_port : stream.in_ports()) {
        //   port_data_t &in_vp = _accel->port_interf().in_port(in_port);
        //   in_vp.push_data_byte(data_byte);
        // }
      }

      if(reorder_entry->last) assert(!stream.stream_active());

      // ivp totally free
      // if this is the last reorder entry allocated in irob and no more
      // entries to allocated in stream
      if (reorder_entry->last && !stream.stream_active()) {
        DSA_LOG(VP_SCORE2) << "SOURCE: Indirect SCR->PORT \n";

        // TODO(@were): Implement this.
        // for (int in_port : stream.in_ports()) {
        //   port_data_t &in_vp = _accel->port_interf().in_port(in_port);
        //   in_vp.set_status(port_data_t::STATUS::FREE, LOC::SCR);
        // }
      }
      assert(_ind_ROB.size()==_reorder_entry_id.size());
      assert(_reorder_entry_id.find(reorder_entry->id)!=_reorder_entry_id.end());
      _reorder_entry_id.erase(reorder_entry->id);
      delete reorder_entry;
      _ind_ROB.pop();
    }
  }
}


// This function does two things:
// 1. Check the reorder buffer if the top entry is ready to complete
// 2. Carry out the actual reads
// TODO:FIXME: This function actually can send data on the bus to the
// ports... so it should really be arbitrated
int scratch_read_controller_t::cycle_read_queue() {
  // First, check if we can commit one request from the reorder buffer
  int bytes_pushed = 0;

  return bytes_pushed;
}

// FIXME: test when the input and output port is 1-byte
void network_controller_t::multicast_data(
    remote_port_multicast_stream_t &stream, int message_size) {

  // TODO(@were): Bring multi-core stuff back.

//  port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
//  int remote_in_port = stream._remote_port;
//  uint64_t bytes_written = 0;
//  uint64_t remote_elem_sent = 0;
//
//  int data_width = out_vp.get_port_width(); // in bytes
//  int8_t val[64]; // number of 8-byte elements to send
//
//
//  // cout << "stream active? " << stream.stream_active() << " out vp mem size: " << out_vp.mem_size() << " message size: " << message_size << endl;
//
//  if (stream.stream_active() && out_vp.mem_size() >= message_size && out_vp.mem_size()) {
//    while (stream.stream_active() // enough in dest
//           && out_vp.mem_size() &&
//           bytes_written < 64) { // enough in source (64-bytes)
//      // TODO: peek out and pop later if message buffer size was full?
//      
//      // POP data_width amount of data
//      SBDT data = out_vp.pop_out_data();
//      for(int i=0; i<data_width; i++){
//        val[i+remote_elem_sent*data_width] = (data >> (i*8)) & 255;
//      }
//
//      bytes_written += data_width;
//      remote_elem_sent+=1;
//      stream._num_elements--;
//
//      DSA_LOG(NET_REQ)
//        << now() << "POPPED b/c port->remote port WRITE: " << out_vp.port() << " "
//        << out_vp.mem_size() << " data: " << std::hex << data;
//      DSA_LOG(NET_REQ) << "After issue: " << stream.toString();
//    }
//    // cout << "Sending message of size: " << (remote_elem_sent*data_width) << endl;
//    // if(stream._out_port==7) { cout << "Core id: " << _accel->get_core_id() << " REMOTE BYTES SENT: " << (remote_elem_sent*data_width) << endl; }
//    _accel->lsq()->push_spu_req(stream._out_port, remote_in_port, val, remote_elem_sent*data_width, stream._core_mask);
//  }
//  // TODO: do we need n/w ctrl bus bandwidth util
//  if (_accel->_ssim->in_roi()) {
//    add_bw(stream.src(), stream.dest(), 1, bytes_written);
//    _accel->_stat_port_multicast += remote_elem_sent;
//  }
}

// indirect stream
// TODO: incomplete
void network_controller_t::write_remote_scr(remote_scr_stream_t &stream) {
//   port_data_t& val_vp = _accel->port_interf().out_port(stream._out_port);
//   port_data_t& addr_vp = _accel->port_interf().out_port(stream._addr_port);
//   int64_t base_addr = stream._remote_scr_base_addr;
//   uint64_t bytes_written=0;
//   uint64_t remote_elem_sent=0;
//   while(stream.stream_active() //enough in dest
//             && val_vp.mem_size() && addr_vp.mem_size() && bytes_written<64) {
//   //enough in source (64-bytes)
//     // peek out and pop later if message buffer size was full?
//     SBDT val = val_vp.pop_out_data();
//     SBDT index = addr_vp.pop_out_data();
// 
//           addr_t final_scr_addr = base_addr + index;
//           // TODO:FIXME: decide on addressing bits
//           // printf("val: %ld and addr: %ld\n",val,final_scr_addr);
//     // Total address bits = log((linear_spad_size*C),2)
//     int spad_offset_bits = 0;
//           uint64_t scr_offset=0;
// 
//     spad_offset_bits = (int)log2(SCRATCH_SIZE+LSCRATCH_SIZE);
//           scr_offset = final_scr_addr & (SCRATCH_SIZE+LSCRATCH_SIZE-1);
//     // TODO: can i get num_cores info here somehow?
//     int cores = 64;
//         int dest_core_id = (final_scr_addr >> spad_offset_bits) & (cores-1); //
//   last bits? if(SS_DEBUG::NET_REQ){ std::cout << "dest_core_id: " <<
//   dest_core_id << "\n"; std::cout << "scr_offset: " << scr_offset << "\n";
//         }
//     // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset,
//   dest_core_id, stream.id());
//     // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset,
//   dest_core_id, stream.id());
// 
//     bytes_written += DATA_WIDTH;
//     remote_elem_sent+=1;
//     stream._num_elements--;
// 
//         if(SS_DEBUG::NET_REQ){
//       timestamp(); cout << "POPPED b/c port->remote scr WRITE: " <<
//   addr_vp.port() << " " << addr_vp.mem_size() << " and " << val_vp.port() << " "
//   << val_vp.mem_size() << "\n";
// 
//           printf("After issue: ");
//           stream.print_status();
//     }
//   }
//   if(_accel->_ssim->in_roi()) {
//     // add_bw(stream.src(), stream.dest(), 1, bytes_written);
//     _accel->_stat_remote_scratch_writes+=remote_elem_sent;
//   }
}

void network_controller_t::write_direct_remote_scr(
    direct_remote_scr_stream_t &stream) {
  // TODO(@were): Bring multi-core stuff back.
  // port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
  // uint64_t bytes_written = 0;
  // uint64_t remote_elem_sent = 0;

  // int8_t val[64]; // there will be only 1 base address
  // int message_size = 64/stream.data_width(); // num of data_width elements to be written
  // if(stream._num_elements<64/stream.data_width()){
  //   message_size=stream._num_elements;
  // }
  // if (stream.stream_active() && val_vp.mem_size() >= message_size) {
  //   addr_t final_scr_addr = stream.cur_addr();
  //   addr_t base_addr = final_scr_addr & SCR_MASK;
  //   addr_t max_addr = base_addr + SCR_WIDTH;

  //   int spad_offset_bits = 0;
  //   uint64_t scr_offset=0;
  //   spad_offset_bits = (int)log2(SCRATCH_SIZE+LSCRATCH_SIZE);
  //   int cores = _accel->_ssim->num_active_threads();
  //   int dest_core_id = (final_scr_addr >> spad_offset_bits) & (cores-1); // last bits?
  //   DSA_LOG(NET_REQ) << "dest_core_id: " << dest_core_id;
  //   DSA_LOG(NET_REQ) << "scr_offset: " << scr_offset;
  //   while (final_scr_addr < max_addr && stream.stream_active() // enough in dest
  //          && val_vp.mem_size() &&
  //          bytes_written < 64) { // enough in source (64-bytes)
  //     // peek out and pop later if message buffer size was full?
  //     SBDT data = val_vp.pop_out_data();
  //     for(int i=0; i<stream.data_width(); i++){
  //       val[remote_elem_sent*stream.data_width()+i] = (data >> (i*8)) & 255;
  //     }

  //     // addr_t final_scr_addr = base_addr + index;
  //     // printf("val: %ld and addr: %ld\n",val,final_scr_addr);
  //     // FIXME: check if this is correct
  //     final_scr_addr = stream.cur_addr();
  //     bytes_written += stream.data_width();
  //     remote_elem_sent+=1;
  //     // stream._num_elements--;

  //     DSA_LOG(NET_REQ)
  //       << now() << ": POPPED b/c direct port->remote scr WRITE: "
  //       << val_vp.port() << " " << val_vp.mem_size();
  //     DSA_LOG(NET_REQ) << "After issue: " << stream.toString();
  //   }
  //   // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset, dest_core_id, stream.id());
  //   // num_bytes = remote_elem_sent*8
  //   _accel->lsq()->push_spu_scr_wr_req(&val[0], remote_elem_sent*stream.data_width(), 
  //                                     scr_offset, dest_core_id, stream.id());
  // }
  // if (_accel->_ssim->in_roi()) {
  //   add_bw(stream.src(), stream.dest(), 1, bytes_written);
  //   _accel->_stat_remote_scratch_writes += remote_elem_sent;
  // }
}

void network_controller_t::serve_pending_net_req() {
  _accel->lsq()->serve_pending_net_req();
}

void network_controller_t::check_cpu_response_queue() {
  _accel->lsq()->check_cpu_response_queue();
}

void network_controller_t::cycle() {
  if(!_accel->bsw.dfg()) return;
  serve_pending_net_req();
  check_cpu_response_queue();
  int i = 0;
  // cycle over all the streams
  for (i = 0; i < _remote_streams.size(); ++i) {
    _which_remote =
        (_which_remote + 1) >= _remote_streams.size() ? 0 : _which_remote + 1;
    // base_stream_t *s = _remote_streams[_which_remote];
    // derivative should always come first
    // TODO(@were): Implement remote stream execution.
    // if (auto *sp = dynamic_cast<direct_remote_scr_stream_t *>(s)) {
    //   auto &stream = *sp;
    //   if (stream.stream_active()) {
    //     port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
    //     // we can send max of message_size*bus_width messages in 1 cycle or till
    //     // the queue is full (Oh, this bus is also 512-bit bus)
    //     write_direct_remote_scr(stream);

    //     if (!stream.stream_active()) {
    //       _accel->process_stream_stats(stream);
    //       DSA_LOG(VP_SCORE2) << "SOURCE: DIRECT PORT->REMOTE SCR";
    //       DSA_LOG(NET_REQ) << "Direct remote scratchpad write stream empty";
    //       val_vp.freeStream();
    //       delete_stream(_which_remote, sp);
    //     }
    //     break;
    //   }
    // }
    // if (auto *sp = dynamic_cast<remote_scr_stream_t *>(s)) {
    //   auto &stream = *sp;
    //   if (stream.stream_active()) {
    //     port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
    //     port_data_t &addr_vp =
    //         _accel->port_interf().out_port(stream._addr_port);
    //     // we can send max of message_size*bus_width messages in 1 cycle or till
    //     // the queue is full (Oh, this bus is also 512-bit bus)
    //     write_remote_scr(stream);

    //     if (!stream.stream_active()) {
    //       _accel->process_stream_stats(stream);
    //       DSA_LOG(VP_SCORE2) << "SOURCE: INDIRECT PORT->REMOTE SCR";
    //       DSA_LOG(NET_REQ) << "Indirect remote scratchpad write stream empty";
    //       val_vp.freeStream();
    //       addr_vp.freeStream();
    //       delete_stream(_which_remote, sp);
    //     }
    //     break;
    //   }
    // } else if (auto *sp = dynamic_cast<remote_port_multicast_stream_t *>(s)) {
    //   auto &stream = *sp;
    //   port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
    //   int message_size=100000; // infinity
    //   if (stream.stream_active()) {

    //     int data_width = out_vp.get_port_width(); // in bytes
    //     message_size = 64/data_width; // num of data_width elements to be written
    //     // cout << "Message size original: " << message_size << endl;
    //     if(stream._num_elements<64/data_width){
    //       message_size=stream._num_elements;
    //     }
    //     // cout << "Message size after comp: " << message_size << " stream elements: " << stream._num_elements <<  endl;
    //    if(stream.timeout()) {
    //       message_size = out_vp.mem_size() < message_size ? out_vp.mem_size() : message_size;
    //     } else if(out_vp.mem_size() < message_size) { // this should be here if no timeout
    //       stream.inc_wait_cycles();
    //       continue;
    //     }
    //   }
    //   if (stream.stream_active() && out_vp.mem_size() >= message_size) {

    //     // we can send max of message_size*bus_wisth messages in 1 cycle or till
    //     // the queue is full (Oh, this bus is also 512-bit bus)
    //     multicast_data(stream, message_size);
    //     break;
    //   }
    // }
  }
}

void network_controller_t::finish_cycle() {}

void network_controller_t::print_status() {
}

void scratch_read_controller_t::print_status() {
}

bool scratch_write_controller_t::crossbar_backpressureOn() {
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (_atomic_scr_issued_requests[i].size() == MAX_BANK_BUFFER_SIZE)
      return true;
  }
  return false;
}


bool scratch_write_controller_t::is_conflict(addr_t scr_addr, int num_bytes) {
   // return false;
  _accel->_stat_tot_atom_cycles++;
  int check_bytes1 = std::min(num_bytes, 64);
  // cout << "Checking for scr_addr: " << scr_addr << " and bytes: " << num_bytes << endl;
  for(auto it=_conflict_detection_queue.begin(); it!=_conflict_detection_queue.end(); ++it) {
    int check_bytes2 = std::min(it->second, 64);
    // cout << "Found addr: " << it->first << " bytes: " << it->second;
    if(scr_addr+check_bytes1>it->first && scr_addr<it->first+check_bytes2) {
      _accel->_stat_conflict_cycles++;
      return true;
    }
  }
  return false;
}


void scratch_write_controller_t::cycle(bool can_perform_atomic_scr,
                                       bool &performed_atomic_scr) {

  // bool issued_linear_on_banked_scr = false;
  // bool is_linear_addr_banked = false;

   // FOR THE BANKED SCRATCHPAD
  for (unsigned i = 0; i < _write_streams.size(); ++i) {
    _which_wr = (_which_wr + 1) >= _write_streams.size() ? 0 : _which_wr + 1;
    // base_stream_t *s = _write_streams[_which_wr];
    // if (auto *sp = dynamic_cast<remote_core_net_stream_t *>(s)) {
    //   auto &stream = *sp;
    //   // int bytes_written=0;
    //   if (stream.stream_active()) { // should always be true
    //     port_data_t &addr_port =
    //         _accel->port_interf().out_port(stream._addr_port);
    //     port_data_t &val_port =
    //         _accel->port_interf().out_port(stream._val_port);

    //     if (addr_port.mem_size() > 0 && val_port.mem_size() > 0) {
    //       // printf("COmes to check remote core net stream\n");
    //       write_scratch_remote_ind(stream);
    //       /*
    //       if(stream.empty()) {
    //         delete_stream(_which_wr,sp);
    //       }
    //       */
    //       break;
    //     }
    //   }
    // }
  }


  // FOR LINEAR WR STREAMS ON THE LINEAR SCRATCHPAD (issues only if the address
  // belongs to linear spad)
  // if((!issued_linear_on_banked_scr) || (issued_linear_on_banked_scr &&
  // !is_linear_addr_banked)) {
}

void scratch_write_controller_t::push_remote_wr_req(uint8_t *val, int num_bytes,
                                                    addr_t scr_addr) {
  // TODO(@were): Bring multi-core remote access back.
  // port_data_t &addr_vp = _accel->port_interf().out_port(NET_ADDR_PORT);
  // port_data_t &val_vp = _accel->port_interf().out_port(NET_VAL_PORT);

  // // assert(num_bytes % 8 == 0);
  // int64_t meta_info =
  //     num_bytes << 16 | scr_addr; // Unnecessary encoding/decoding
  // // addr_vp.push_mem_data(meta_info);
  // addr_vp.push_data(meta_info); // this is by-default 8-bytes
  // // addr_vp.push_mem_data(addr);
  // for (int i = 0; i < num_bytes; i++) {
  //   // val_vp.push_data_byte(val[i]);
  //   val_vp.push_data(val_vp.get_byte_vector(val[i],1));
  //   // cout << "data_byte being pushed to net_in ports: " << val[i] << "\n";
  // }
}

void scratch_write_controller_t::finish_cycle() {}

void scratch_write_controller_t::print_status() {
}

// void port_controller_t::cycle() {
//  // Remote-Port
//  for (unsigned i = 0; i < _remote_port_streams.size(); ++i) {
//    // _which_rp =
//    //     (_which_rp + 1) == _remote_port_streams.size() ? 0 : _which_rp + 1;
//    // auto &pi = _accel->port_interf();
//    // auto &stream = _remote_port_streams[_which_rp];
//    // if (!stream.stream_active()) {
//    //   continue;
//    // }
//    // // vp out comes from a different core!
//    // // port_data_t& vp_out = pi.out_port(stream._out_port);
//
//    // int acc_index = _accel->accel_index() - stream._which_core;
//    // if (acc_index == -1) {
//    //   acc_index = (_accel->NUM_ACCEL - 1);
//    // }
//    // if (acc_index == (_accel->NUM_ACCEL)) {
//    //   acc_index = 0;
//    // }
//    // accel_t *rem_acc = _accel->get_ssim()->get_acc(acc_index);
//    // auto &rem_pi = rem_acc->port_interf();
//    // port_data_t &vp_out = rem_pi.out_port(stream._out_port);
//
//    // bool port_in_okay = true;
//    // for (int in_port : stream.in_ports()) {
//    //   port_data_t &vp_in =
//    //       pi.in_port(in_port); // just wait until 8 items can come
//    //   port_in_okay = port_in_okay && (vp_in.mem_size() < VP_LEN - 8*8/vp_in.get_port_width());
//    //   // port_in_okay = port_in_okay && (vp_in.mem_size() < VP_LEN - 8);
//    // }
//
//    // if (vp_out.mem_size() && port_in_okay) { // okay go for it
//    //   uint64_t total_pushed = 0;
//    //   for (int i = 0; i < PORT_WIDTH && vp_out.mem_size() && stream.stream_active(); ++i) {
//    //     SBDT val = vp_out.pop_out_data();
//
//    //     stream._num_elements--;
//    //     if (stream._padding_size != NO_PADDING)
//    //       (++stream._padding_cnt) %= stream._padding_size;
//
//    //     //timestamp();
//    //     //std::cerr << "POPPED b/c port -> remote_port WRITE: " << vp_out.port() << " stream: "
//    //     //          << stream._num_elements << " vp: " << vp_out.mem_size() << std::endl;
//
//    //     for (int in_port : stream.in_ports()) {
//    //       port_data_t &in_vp = _accel->port_interf().in_port(in_port);
//    //       in_vp.push_data(val);
//    //       if (stream._padding_size != NO_PADDING && stream._padding_cnt == 0) {
//    //         int to_fill = in_vp.port_cgra_elem() - stream._padding_size % in_vp.port_cgra_elem();
//    //         std::vector<uint8_t> data(in_vp.get_port_width(), 0);
//    //         for (int i = 0; i < to_fill; ++i) {
//    //           in_vp.push_data(data, stream.fill_mode() != STRIDE_DISCARD_FILL);
//    //         }
//    //       }
//    //     }
//
//    //     total_pushed++;
//    //   }
//
//    //   add_bw(stream.src(), stream.dest(), 1, total_pushed * DATA_WIDTH);
//
//    //   bool is_empty = stream.!stream_active();
//    //   if (is_empty) {
//    //     _accel->process_stream_stats(stream);
//
//    //     if (SS_DEBUG::VP_SCORE2) {
//    //       cout << "SOURCE: PORT->PORT\n";
//    //     }
//    //     for (int in_port : stream.in_ports()) {
//    //       port_data_t &in_vp = _accel->port_interf().in_port(in_port);
//    //       in_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE,
//    //                        stream.fill_mode());
//    //     }
//
//    //     if (SS_DEBUG::VP_SCORE2) {
//    //       cout << "SOURCE: PORT->PORT\n";
//    //     }
//    //     vp_out.set_status(port_data_t::STATUS::FREE);
//    //   }
//
//    //   break;
//    // }
//  }
// }

bool accel_t::done(bool show, int mask) {
  static const std::string IO_STR[] = {"Output", "Input"};
  for (int io = 0; io < 2; ++io) {
    for (auto &elem : bsw.ports[io]) {
      auto vp = this->port(io, elem.port);
      if (vp->stream && (vp->stream->barrier_mask() & mask)) {
        DSA_LOG(WAIT) << IO_STR[io] << "Port: " << vp->id() << " " << vp->stream->toString();
        return false;
      }
      if (mask == -1) {
        if (!vp->empty()) {
          DSA_LOG(WAIT) << IO_STR[io] << "Port: " << vp->id() << " data entry not empty!";
          return false;
        }
      }
    }
  }
  return true;
}

// checks only stream engines to see if concurrent operations are done
bool accel_t::done_concurrent(bool show, int mask) {
  bool done = true;

  if (!_dma_c.done(show, mask) || !_net_c.done(show, mask)) {
    done = false;
  }

  bool check_cgra_done = true;
  if(mask==STREAM_WAIT) {
    check_cgra_done = false;
  }

  if(check_cgra_done) {
    if (done && !cgra_done(show, mask)) {
      done = false;
    }
  }

  return done;
}

// checks everything to see if it's done  (includes queues)
bool accel_t::done_internal(bool show, int mask) {
  if (!done_concurrent(show, mask)) {
    return false;
  }

  return true;
}

bool network_controller_t::remote_port_multicast_requests_active() {
  return _remote_port_multicast_streams.size();
}
bool network_controller_t::remote_scr_requests_active() {
  return _remote_scr_streams.size();
}
bool network_controller_t::direct_remote_scr_requests_active() {
  return _direct_remote_scr_streams.size();
}

bool dma_controller_t::done(bool show, int mask) {
  for (auto elem : _read_streams) {
    if (elem->barrier_mask() & mask) {
      if (elem->stream_active()) {
        return false;
      }
    }
  }
  for (auto elem : _write_streams) {
    if (elem->barrier_mask() & mask) {
      if (elem->stream_active()) {
        return false;
      }
    }
  }
  return true;
}

bool scratch_write_controller_t::atomic_scr_streams_active() {
  return _atomic_scr_streams.size();
}

bool scratch_read_controller_t::indirect_scr_read_requests_active() {
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (!_indirect_scr_read_requests[i].empty()) {
      return true;
    }
  }
  return false;
}

bool network_controller_t::done(bool show, int mask) {
//  if (mask == 0 || mask & WAIT_CMP || mask == STREAM_WAIT) {
//      // if (mask == 0 || mask & WAIT_CMP || mask == GLOBAL_WAIT || mask == STREAM_WAIT) {
//    if(!_accel->_lsq->is_pending_net_empty()) {
//      if (show)
//        cout << "SPU network requests pending in queue\n";
//      return false;
//    }
//    if (remote_port_multicast_requests_active()) {
//      if (show)
//        cout << "PORT -> REMOTE PORT Stream Not Empty\n";
//      return false;
//    }
//    if (remote_scr_requests_active()) {
//      if (show)
//        cout << "INDIRECT PORT -> REMOTE SCR Stream Not Empty\n";
//      return false;
//    }
//    if (direct_remote_scr_requests_active()) {
//      if (show)
//        cout << "DIRECT PORT -> REMOTE SCR Stream Not Empty\n";
//      return false;
//    }
//  }
  return true;
}

bool accel_t::cgra_done(bool show, int mask) {

  if (mask == 0 || mask & WAIT_CMP) {
      // if (mask == 0 || mask & WAIT_CMP || mask == GLOBAL_WAIT) {
    for (unsigned i = 0; i < bsw.iports().size(); ++i) {
      int cur_port = bsw.iports()[i].port;
      auto &in_vp = input_ports[cur_port];
      if (!in_vp.buffer.empty()) {
        // if (in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size() ||
        // _dfg->is_busy()) {
        if (show) {
          cout << "In VP: " << cur_port << " Not Empty (";
          cout << " Buffer: " << in_vp.buffer.size();
          cout << ") \n";
        }

        return false;
      }
    }
  }
  if (mask == 0) {
    for (unsigned i = 0; i < bsw.oports().size(); ++i) {
      int cur_port = bsw.oports()[i].port;
      auto &out_vp = output_ports[cur_port];
      if (!out_vp.raw.empty()) {
        if (show) {
          cout << "Out VP: " << cur_port << " Not Empty (";
          cout << " Buffer : " << out_vp.raw.size();
          cout << ") \n";
        }
        return false;
      }
    }
  }
  return true;
}

// Configure once you get all the bits
void accel_t::configure(addr_t addr, int size, uint64_t *bits) {

  DSA_LOG(MEM_REQ)
    << "dsaURE(response): " << "0x" << std::hex
    << addr << " " << std::dec << size;

  std::string basename = ((char*)(bits) + 9);
  SSDfg *dfg = dsa::dfg::Import(".sched/" + basename + ".dfg.json");
  auto sched = new Schedule(_ssconfig, dfg);
  sched->LoadMappingInJson(".sched/" + basename + ".sched.json");
  bsw = dsa::sim::BitstreamWrapper(sched);

  for (auto &elem : bsw.iports()) {
    auto &vp = input_ports[elem.port];
    vp.vp = elem.vp;
  }

  for (auto &elem : bsw.iports()) {
    auto &vp = input_ports[elem.port];
    // If this input port's tag is not -1, find its corresponding vp.
    if (vp.ivp()->tid != -1) {
      for (auto &another : bsw.iports()) {
        auto &another_vp = input_ports[another.port];
        if (another.vp->id() == vp.ivp()->tid) {
          vp.affine_tag = &another_vp;
        }
      }
    }
  }

  for (auto &elem : bsw.oports()) {
    auto &vp = output_ports[elem.port];
    vp.vp = elem.vp;
  }

}

void scratch_write_controller_t::insert_pending_request_queue(int tid, vector<int> start_addr, int bytes_waiting) {
  if(start_addr.size()==0) return; // for current bad impl
  auto it = _pending_request_queue.find(tid);
  if(it!=_pending_request_queue.end()) {
    for(unsigned i=0; i<start_addr.size(); ++i) {
      it->second.second.push_back(start_addr[i]);
    }
  } else {
    _pending_request_queue.insert(make_pair(tid, make_pair(bytes_waiting, start_addr)));
  }
  assert(bytes_waiting==_accel->_assumed_bytes);
  // assert(it==_pending_request_queue.end() && "this tid already present...");
  // cout << "Inserted a task in pq for tid: " << tid << " at core: " << _accel->_lsq->getCpuId() << endl;
}

int scratch_write_controller_t::push_and_update_addr_in_pq(int tid, int num_bytes) {
  // cout << "Searching for a task in pq for tid: " << tid << " at core: " << _accel->_lsq->getCpuId() << endl;
  auto it = _pending_request_queue.find(tid);
  
  // Step2: push new addresses
  assert(it!=_pending_request_queue.end());

  unsigned num_addr = it->second.second.size();
  // cout << "Came to push new atomic addresses, with num_addr: " << num_addr << " and num bytes: " << num_bytes << endl;
  assert(num_addr!=0);
   // Step1: delete the entry if not required anymore
  it->second.first -= num_bytes;
  
  // SPECIAL CASE WHEN TASKS CANNOT BE INTERLEAVED WITH OTHER TASKS
  if(it->second.first!=0) return 0;
  num_bytes=_accel->_assumed_bytes; // TODO: fix later
  reorder_buffer[_cur_drob_fill_ptr] = make_pair(tid,make_pair(num_addr, false)); // assuming value is not received yet!
  _cur_drob_fill_ptr = (_cur_drob_fill_ptr+1)%MAX_ATOM_REQ_QUEUE_SIZE;
  _drob_size_used++;

  // TODO: allowed to push data if values complete, and enough bytes available

  // port_data_t &in_addr = _accel->port_interf().in_port(ATOMIC_ADDR_PORT);
  // port_data_t &bytes_prt = _accel->port_interf().in_port(ATOMIC_BYTES_PORT);

  // // push num_addr bytes while these are addresses in int
  // for(int i=0; i<num_addr; ++i) {
  //   in_addr.push_data(in_addr.get_byte_vector(it->second.second[i], ATOMIC_ADDR_DATA_WIDTH));
  //   bytes_prt.push_data(bytes_prt.get_byte_vector(num_bytes,BYTES_PORT_DATA_WIDTH));
  //   // it->second.second[i] += num_bytes; // update address
  // }
  // _pending_request_queue.erase(tid); // should be completed here
  return num_addr;
}

// FIXME: didn't make sure if the data is in there or not...
void scratch_write_controller_t::push_atomic_inc(int tid, std::vector<uint8_t> inc, int repeat_times) {

  auto it = _atom_val_store.find(tid);
  if(it==_atom_val_store.end()) {
      _atom_val_store.insert(make_pair(tid, inc));
  } else {
    for(unsigned i=0; i<inc.size(); ++i) {
      it->second.push_back(inc[i]);
    }
  }

  // if done, sets its corresponding reorder entry to be true
  auto it2 = _atom_val_store.find(tid);
  assert(it2->second.size()<=_accel->_assumed_bytes);
  if(it2->second.size()==_accel->_assumed_bytes) {
    bool set=false;
    for(int i=0; i<MAX_ATOM_REQ_QUEUE_SIZE; ++i) { // can be optimized
      if(reorder_buffer[i].first==tid) {
        reorder_buffer[i].second.second=true;
        // cout << "Setting index i: " << i << " true!\n";
        set=true;
        break;
      }
    }
    assert(set);
    // _atom_val_store.erase(it2);
  }

}


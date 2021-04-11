#include <fstream>
#include <iomanip>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <sstream>
#include <numeric>
#include <functional>

#include "dsa/dfg/utils.h"

#include "../cpu.hh"
#include "./ssim.hh"
#include "./arbiter.h"
#include "./request.h"

using namespace std;

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
  return _ssim->now() - _ssim->roi_enter_cycle();
}


void port_data_t::bindStream(base_stream_t *exec_stream) {
  CHECK(!stream) << "Stream is still occupied!";
  stream = exec_stream;
  if (_isInput) {
    struct PostProcessor : dsa::sim::stream::Functor {
      void Visit(IPortStream *ips) {
        pes_ptr = &ips->pes;
      }
      void Visit(PortPortStream *pps) {
        pes_ptr = &pps->iports;
      }
      std::vector<PortExecState> *pes_ptr{nullptr};
    };
    PostProcessor pp;
    exec_stream->Accept(&pp);
    if (pp.pes_ptr) {
      for (auto &elem : *pp.pes_ptr) {
        if (port() == elem.port) {
          pes = elem;
          return;
        }
      }
      CHECK(false) << "Port state set! Only port-involved can be bound.";
    }
  }
}

void port_data_t::freeStream() {
  CHECK(stream) << "No stream to free!";
  CHECK(!stream->stream_active()) << "Stream is still active!";
  struct PostProcessor : dsa::sim::stream::Functor {
    void Visit(LinearReadStream *lrs) {
      if (lrs->be) {
        CHECK(lrs->be->use == lrs);
        lrs->be->use = nullptr;
      }
    }
    void Visit(LinearWriteStream *lws) {
      if (lws->be) {
        CHECK(lws->be->load == lws);
        lws->be->load = nullptr;
      }
    }
  };
  PostProcessor functor;
  stream->Accept(&functor);
  stream = nullptr;
}

Minor::MinorDynInstPtr accel_t::cur_minst() { return _ssim->cur_minst(); }

void accel_t::sanity_check_stream(base_stream_t *s) {
  // sanity check -- please don't read/write a stream that's not configured!
  if (auto is = dynamic_cast<IPortStream*>(s)) {
    for (auto &port : is->pes) {
      int in_port = port.port;
      if (in_port != -1 && in_port < START_IND_PORTS) {
        vector<int>::iterator it =
            std::find(_soft_config.in_ports_active.begin(),
                      _soft_config.in_ports_active.end(), in_port);
        if (it == _soft_config.in_ports_active.end()) {
          timestamp();
          std::cout << "In port " << in_port << " is not active for "
                    << s->short_name() << ", maybe a configure error!\n";
          s->print_status();
        }
      }
    }
  }

  if (s->out_port() != -1 && s->out_port() < START_IND_PORTS) {
    vector<int>::iterator it =
        std::find(_soft_config.out_ports_active.begin(),
                  _soft_config.out_ports_active.end(), s->out_port());
    if (it == _soft_config.out_ports_active.end()) {
      timestamp();
      std::cout << "Out port " << s->out_port() << " is not active for "
                << s->short_name() << ", maybe a configure error!\n";
    }
  }
}

void accel_t::request_reset_data() {
  _cleanup_mode = true;
  reset_data();
  _cmd_queue.clear(); // check this as well
  LOG(COMMAND_O) << "Complete reset request served except deleting requested memory reads/writes: "
                 << lsq()->getCpuId() << "\n";
}

// Let's not consider input ports here
bool accel_t::all_ports_empty() {

  // It should check only those output ports which are busy
  for (unsigned i = 0; i < _soft_config.out_ports_active_plus.size(); ++i) {
    int cur_port = _soft_config.out_ports_active_plus[i];
    auto &out_vp = _port_interf.out_port(cur_port);
    // if(out_vp.in_use() && out_vp.mem_size()) {
    if(out_vp.in_use() && out_vp.mem_size()) {
      std::cout << "Output port not empty: " << cur_port << " at core id: " << lsq()->getCpuId() << endl;
      return false;
    }
  }

  std::cout << "Output ports empty\n";

  // Let's check atomic bank buffers
  if(_scr_w_c.atomic_scr_issued_requests_active()) {
    std::cout << "Atomic bank buffers not empty!\n";
    return false;
  }

  return true;
}

void accel_t::request_reset_streams() {

  if(!all_ports_empty()) return;

  LOG(COMMAND_O) << "RESET STREAM REQUEST RELEASED for core: " << lsq()->getCpuId();

  _cmd_queue.clear();

  // free ports and streams (not input ports?)
  // reset_data();
  for (unsigned i = 0; i < _soft_config.out_ports_active_plus.size(); ++i) {
    int cur_port = _soft_config.out_ports_active_plus[i];
    auto &out_vp = _port_interf.out_port(cur_port);
    out_vp.reset_data();
    /*if(out_vp.in_use()) {
      out_vp.reset_data();
    }*/
  }

  for (unsigned i = 0; i < _soft_config.in_ports_active_plus.size(); ++i) {
    int cur_port = _soft_config.in_ports_active_plus[i];
    auto &in_vp = _port_interf.in_port(cur_port);
    /*if(cur_port==5 || cur_port==31) {
      continue;
    } else {
      in_vp.reset_data();
    }*/
    if(in_vp.in_use()) {
      // also remove data from it
      // in_vp.set_status(port_data_t::STATUS::FREE);
      in_vp.reset_data();
    }
  }

  _dma_c.reset_data();
  _scr_r_c.reset_data();
  _scr_w_c.reset_data();
  _net_c.reset_data();

  _stream_cleanup_mode=false;
  _cleanup_mode = true; // it should wait for outstanding mem req to be done (wait on o/p ports and then cleanup memory)
}

void accel_t::switch_stream_cleanup_mode_on() {
  _stream_cleanup_mode = true;
}

void accel_t::reset_data() {
  if (SS_DEBUG::COMMAND) {
    timestamp();
    std::cout << "RESET_DATA REQUEST INITIALIZED\n";
  }

  for (unsigned i = 0; i < _soft_config.in_ports_active_plus.size(); ++i) {
    int cur_port = _soft_config.in_ports_active_plus[i];
    auto &in_vp = _port_interf.in_port(cur_port);
    in_vp.reset_data();
  }

  // TODO: FIXME: I think we should not need to reset network ports? (reset is
  // for compute -- cannot reset it before config also)
  /*
  auto &in_addr = _port_interf.in_port(NET_ADDR_PORT);
  in_addr.reset_data();
 
  auto &in_val = _port_interf.in_port(NET_VAL_PORT);
  in_val.reset_data();
  */
  
  for (unsigned i = 0; i < _soft_config.out_ports_active_plus.size(); ++i) {
    int cur_port = _soft_config.out_ports_active_plus[i];
    auto &out_vp = _port_interf.out_port(cur_port);
    out_vp.reset_data();
  }

  _dma_c.reset_data();
  _scr_r_c.reset_data();
  _scr_w_c.reset_data();
  _net_c.reset_data();

}

// --------------------------------- CONFIG
// ---------------------------------------
void soft_config_t::reset() {
  cur_config_addr = 0;
  in_ports_active.clear();
  in_ports_active_group.clear();
  in_ports_active_backcgra.clear();
  in_ports_name.clear();
  out_ports_name.clear();
  group_thr.clear();
  out_ports_active.clear();
  out_ports_active_group.clear();
  in_ports_active_plus.clear();
  out_ports_active_plus.clear();
  in_port_delay.clear();
  out_ports_lat.clear();
  cgra_in_ports_active.clear();
  inst_histo.clear();
}

// ------------------------------ VECTOR PORTS
void port_data_t::initialize(SSModel *ssconfig, int port, bool isInput) {
  _isInput = isInput;
  _port = port;
  _bytes_waiting = 0;
  pes.port = port;
}

void port_data_t::reset() {
  assert(_outstanding == 0);
  assert(status() == STATUS::FREE);
  // assert(_loc == LOC::NONE); // TODO: should this be enforced?
  // removed because for data which enters remotely, cgra might get data from
  // somewhere else assert(_mem_data.size()==0);
  assert(_mem_data.size() == _valid_data.size());

  assert(_num_in_flight == 0);
  CHECK(_num_ready == 0) << port() << " " << _num_ready;
  _dfg_vec=NULL;

  _total_pushed = 0;
}


// total size elements of the vector port
unsigned port_data_t::port_vec_elem() {
  if(_dfg_vec) {
    // return _dfg_vec->length();
    return _dfg_vec->logical_len();
  } else {
    return 1;
  }
}

void port_data_t::reformat_in() { // rearrange data for CGRA
  // resize the data queue to macth the data elements of port vector
  /*unsigned extra_elem = _mem_data.size() % port_vec_elem();
  if(extra_elem!=0) {
    std::cerr << "Too few elem, zero padding\n";
    _mem_data.resize(_mem_data.size() + port_vec_elem()-extra_elem);
  }*/
  // cout << "Came to reformat in at port: " << _port << " and mem data size: " << _mem_data.size() << endl;
  while (_mem_data.size() >= port_vec_elem()) {
    reformat_in_work();
  }
}

void port_data_t::reformat_in_one_vec() { // rearrange data for CGRA
  if (_mem_data.size() >= port_vec_elem() && _num_ready + port_depth() <= CGRA_FIFO_LEN) {
    reformat_in_work();
  }
}

void port_data_t::reformat_in_work() {
  if (pes.repeat == 0) { // on repeat==0, delete data
    _mem_data.erase(_mem_data.begin(), _mem_data.begin() + port_vec_elem());
    _valid_data.erase(_valid_data.begin(), _valid_data.begin() + port_vec_elem());
    return;
  }

  int cgra_port=0; //just stripe accross the port
  for(unsigned i = 0; i < port_vec_elem(); ++i,    
      cgra_port = cgra_port+1 == _cgra_data.size() ? 0 : cgra_port + 1) {
      std::vector<uint8_t> val = _mem_data.front();
      bool valid = _valid_data.front();
      _cgra_data[cgra_port].push_back(val);
      _cgra_valid[cgra_port].push_back(valid);
      _mem_data.pop_front();
      _valid_data.pop_front();
  }

  _num_ready += port_depth();
}

// pop data from input port
// It should be allowed to pop only _port_width data
SBDT port_data_t::pop_in_data() {
  assert(_mem_data.size());
  assert(_mem_data.size() == _valid_data.size());
  SBDT val = peek_out_data();
  _mem_data.pop_front();
  _valid_data.pop_front();
  return val;
}

template <typename T> 
T port_data_t::pop_in_custom_data() {
  assert(_mem_data.size());
  vector<uint8_t> v = _mem_data.front();
  T val = get_custom_val<T>(v, _port_width);
  _mem_data.pop_front();
  _valid_data.pop_front();
  return val;
}

// pop SBDT data from output port
SBDT port_data_t::pop_out_data(int bytes) {
  if (bytes == -1) {
    bytes = _port_width;
  } else {
    assert(bytes % _port_width == 0);
  }
  SBDT res = 0;
  for (int i = 0; i < bytes; i += _port_width) {
    SBDT val = peek_out_data();
    _mem_data.pop_front();
    _valid_data.pop_front(); // TODO: for now we ignore invalid, hope that's okay?
    res |= val << (i * 8);
  }
  if (SS_DEBUG::DATA_STATUS) {
    std::cout << "bytes popped: " << bytes << ", value is " << res << " at port id: " << _port << std::endl;
    std::cout << "remain " << _port_width * _mem_data.size() << " bytes" << std::endl;
  }
  return res;
}

// pop T amount of data from output port
template <typename T> T port_data_t::pop_out_custom_data() {
  assert(_mem_data.size());
  vector<uint8_t> v = _mem_data.front();
  T val = get_custom_val<T>(v, _port_width);
  _mem_data.pop_front();
  _valid_data.pop_front();
  return val;
}

// peek at output data (return original data typecasted to SBDT)
// starting from idx, return bytes amount of data
SBDT port_data_t::peek_out_data(int idx, int bytes) {

  if (bytes == -1) {
    bytes = _port_width;
  } else {
    assert(bytes % _port_width == 0);
  }

  CHECK(idx + bytes / _port_width <= _mem_data.size());
  
  SBDT res = 0;

  for (int i = 0; i < bytes; i += _port_width) {
    SBDT partial = 0;
#define RI(x) case x: {                                              \
  partial = *((uint##x##_t*)(&_mem_data[idx + i / _port_width][0])); \
  break;                                                             \
}
    switch(_port_width * 8) {
    RI(64)
    RI(32)
    RI(16)
    RI(8)
    default:
      assert(false);
    }
#undef RI
    res |= partial << (8 * i);
  }

  if (SS_DEBUG::DATA_STATUS) {
    std::cout << "peeked value is " << bytes << std::endl;
  }

  return res;
}

// Throw away data in CGRA input ports
void port_data_t::pop(unsigned instances) {
  assert(_num_ready >= instances);
  _num_ready -= instances;
  assert(_mem_data.size() == _valid_data.size());

  for (unsigned cgra_port = 0; cgra_port < _cgra_data.size(); ++cgra_port) {
    assert(_cgra_data[cgra_port].size() == _cgra_valid[cgra_port].size());
    assert(_cgra_data[cgra_port].size() >= instances);
    _cgra_data[cgra_port].erase(_cgra_data[cgra_port].begin(),
                                _cgra_data[cgra_port].begin() + instances);
    _cgra_valid[cgra_port].erase(_cgra_valid[cgra_port].begin(),
                                 _cgra_valid[cgra_port].begin() + instances);
  }
  assert(_mem_data.size() == _valid_data.size());
}

// rearrange data from CGRA
void port_data_t::reformat_out() {
    // cout << "Came to reformat in at port: " << _port << " and mem data size: " << _mem_data.size() << endl;
   // cout << "Can output: " << can_output() << endl;
  while (can_output()) {
    reformat_out_work();
  }
}

void port_data_t::reformat_out_one_vec() {
  // if (can_output() && _mem_data.size() + port_vec_elem() <= VP_LEN) {
  if (can_output() && _mem_data.size() + port_vec_elem() <= VP_LEN*8/_port_width) {
    assert(_cgra_data[0].size() >= _num_ready);
    assert(_cgra_data[0].size() >= 1);
    reformat_out_work();
  }
}

void port_data_t::reformat_out_work() {
  unsigned num = 0;
  for (unsigned cgra_port = 0; cgra_port < _cgra_data.size(); ++cgra_port) {
    assert(num == 0 || _cgra_data[cgra_port].size() == num);
    num = _cgra_data[cgra_port].size();
  }

  int cgra_port=0; //just stripe accross the port
  for(unsigned i = 0; i < port_vec_elem(); ++i,
    cgra_port = cgra_port+1 == _cgra_data.size() ? 0 : cgra_port + 1) {
    assert(_cgra_data[cgra_port].size() > 0);
    assert(_cgra_data[cgra_port].size() == _cgra_valid[cgra_port].size());

    vector<uint8_t> v = _cgra_data[cgra_port].front();
    bool valid = _cgra_valid[cgra_port].front();

    if (valid) push_data(v, v.size());

    _cgra_data[cgra_port].pop_front();
    _cgra_valid[cgra_port].pop_front();
  }

  _num_ready -= port_depth();
}

// ----------------------------- Port Interface
void port_interf_t::initialize(SSModel *ssconfig) {
  _in_port_data.resize(NUM_IN_PORTS);
  _out_port_data.resize(NUM_OUT_PORTS);

  for(int i=0; i < NUM_IN_PORTS; ++i) {
    _in_port_data[i].initialize(ssconfig, i, true);
  }

  for(int j=0; j < NUM_OUT_PORTS; ++j) {
    _out_port_data[j].initialize(ssconfig, j, false);
  }

}

// ---------------------------- ACCEL ------------------------------------------
uint64_t accel_t::now() {
  return lsq()->get_cpu().curCycle();
}

Minor::LSQ *accel_t::lsq() {
  return get_ssim()->lsq();
}

accel_t::accel_t(int i, ssim_t *ssim)
    : NUM_ACCEL(ssim->NUM_ACCEL), arbiter(new dsa::sim::RoundRobin()),
      _ssim(ssim), _accel_index(i), _accel_mask(1 << i),
      _dma_c(this, &_scr_r_c, &_scr_w_c, &_net_c), _scr_r_c(this, &_dma_c),
      _scr_w_c(this, &_dma_c), _net_c(this, &_dma_c) {
  spads.emplace_back(4, 16, SCRATCH_SIZE, 1, new dsa::sim::InputBuffer(4, 16, 1));

  int ugh = system("mkdir -p stats/");
  ugh += system("mkdir -p viz/");

  if (SS_DEBUG::VERIF_MEM || SS_DEBUG::VERIF_PORT || SS_DEBUG::VERIF_CGRA ||
      SS_DEBUG::VERIF_SCR || SS_DEBUG::VERIF_CMD) {
    ugh += system("mkdir -p verif/");
    cout << "DUMPING VERIFICATION OUTPUTS (dir: verif/) ... SIMULATION WILL BE "
            "SLOWER\n";
    cout << "ALSO, MEMORY ACCESS STATISTICS MAY DIFFER\n";
  }
  if (SS_DEBUG::VERIF_PORT) {
    in_port_verif.open(
        ("verif/" + SS_DEBUG::verif_name + "in_port.txt").c_str(),
        ofstream::trunc | ofstream::out);
    assert(in_port_verif.is_open());
    out_port_verif.open(
        ("verif/" + SS_DEBUG::verif_name + "out_port.txt").c_str(),
        ofstream::trunc | ofstream::out);
    assert(out_port_verif.is_open());
  }
  if (SS_DEBUG::VERIF_SCR) {
    scr_rd_verif.open(("verif/" + SS_DEBUG::verif_name + "scr_rd.txt").c_str(),
                      ofstream::trunc | ofstream::out);
    assert(scr_rd_verif.is_open() && scr_rd_verif.good());

    scr_wr_verif.open(("verif/" + SS_DEBUG::verif_name + "scr_wr.txt").c_str(),
                      ofstream::trunc | ofstream::out);
    assert(scr_wr_verif.is_open());
  }
  if (SS_DEBUG::VERIF_CMD) {
    cmd_verif.open(("verif/" + SS_DEBUG::verif_name + "core_trace.txt").c_str(),
                   ofstream::trunc | ofstream::out);
    assert(cmd_verif.is_open());
  }

  _cgra_dbg_stream = &std::cout;

  if (SS_DEBUG::VERIF_CGRA) {
    SS_DEBUG::COMP = 1;
    cgra_multi_verif.open(("verif/" + SS_DEBUG::verif_name + "cgra_comp" +
                           std::to_string(_accel_index) + ".txt")
                              .c_str(),
                          ofstream::trunc | ofstream::out);
    assert(cgra_multi_verif.is_open());
    _cgra_dbg_stream = &cgra_multi_verif;
  }

  const char *fifo_len_str = std::getenv("FU_FIFO_LEN");
  if (fifo_len_str != nullptr) {
    _fu_fifo_len = atoi(fifo_len_str);
  }

  const char *ind_rob_size = std::getenv("IND_ROB_SIZE");
  if (ind_rob_size != nullptr) {
    _ind_rob_size = atoi(ind_rob_size);
  }

  assert(_ind_rob_size<64 && "ind rob size exceeded the bits allocated in SPU-net transaction");

  const char *ssconfig_file = std::getenv("SBCONFIG");
 
  if (SS_DEBUG::SUPRESS_STATS==0) {
    // static bool printed_this_before = false;
    if (!_ssim->printed_this_before()) {
      std::cout << "Loading SS Config (env SBCONFIG): \"" << ssconfig_file
                << "\"\n";
      std::cout << "FU_FIFO_LEN:" << _fu_fifo_len << "\n";
      std::cout << "IND_ROB_SIZE:" << _ind_rob_size << "\n";
      _ssim->set_printed_this_before(true);
      // printed_this_before=true;
    }
  }

  const char *back_cgra_str = std::getenv("BACKCGRA");
  if (back_cgra_str != NULL) {
    _back_cgra = true;
  }
 
  const char *linear_spad_str = std::getenv("LINEAR_SCR");
  if (linear_spad_str != NULL) {
    _linear_spad = true;
  }
  _banked_spad_mapping_strategy = std::getenv("MAPPING");

  _ssconfig = new SSModel(ssconfig_file);
  // cout << "Came here to create a new configuration for a new accel\n";
  _ssconfig->setMaxEdgeDelay(_fu_fifo_len);
  _port_interf.initialize(_ssconfig);

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

bool accel_t::can_add_stream() {
  _ssim->set_in_use();
  return _cmd_queue.size() < _queue_size;
}

bool accel_t::in_roi() { return _ssim->in_roi(); }

// The job of whos_to_blame(group) is to determine the reason of whether
// a group is able to issue or not.
// It's not possible to determine if
// there is data/stream for one port, but no data/stream for another, why
// that is (is it scatchpad or core?), so that's not answered here.
pipeline_stats_t::PIPE_STATUS accel_t::whos_to_blame(int group) {
  bool any_empty_fifo = false, any_scr = false, any_dma = false,
       any_const = false, any_rec = false;

  auto &active_ports = _soft_config.in_ports_active_group[group];
  auto &active_out_ports = _soft_config.out_ports_active_group[group];

  if (active_ports.size() == 0)
    return pipeline_stats_t::NOT_IN_USE;

  if (_cgra_issued_group[group])
    return pipeline_stats_t::ISSUED;

  // Iterate over inputs
  bool any_input_activity = false;
  bool all_inputs_inactive = true;
  int num_unknown_input = 0;
  for (unsigned i = 0; i < active_ports.size(); ++i) {
    auto &in_vp = _port_interf.in_port(active_ports[i]);
    bool assigned_ivp = in_vp.in_use() || in_vp.completed();
    bool empty_fifo = in_vp.any_data() == 0;
    any_empty_fifo |= empty_fifo;
    if (empty_fifo && assigned_ivp) {
      switch (in_vp.loc()) {
      case LOC::DMA:
        any_dma = true;
        break;
      case LOC::SCR:
        any_scr = true;
        break;
      case LOC::PORT:
        any_rec = true;
        break;
      case LOC::CONST:
        any_const = true;
        break;
      default:
        break;
      }
    }
    num_unknown_input += (empty_fifo && !assigned_ivp);
    any_input_activity |= assigned_ivp || !empty_fifo;
    all_inputs_inactive = all_inputs_inactive && empty_fifo && !assigned_ivp;
  }

  if (num_unknown_input == 0) {
    if (!any_empty_fifo)
      return pipeline_stats_t::CGRA_BACK;
    if (any_const)
      return pipeline_stats_t::CONST_FILL;
    if (any_scr)
      return pipeline_stats_t::SCR_FILL;
    if (any_dma)
      return pipeline_stats_t::DMA_FILL;
    if (any_rec)
      return pipeline_stats_t::REC_WAIT;
  }

  for (unsigned i = 0; i < active_out_ports.size(); ++i) {
    // This is just a guess really, but things really are bad
    // if memory write is the bottleneck...
    auto &out_vp = _port_interf.out_port(active_out_ports[i]);
    if (out_vp.any_data()) {
      if (!lsq()->sd_transfers[MEM_WR_STREAM].canReserve()) {
        return pipeline_stats_t::DMA_WRITE;
      }
    }
  }

  bool bar_scratch_read = false; // These are set by scratch
  bool bar_scratch_write = false;
  bool bar_remote_scratch_write = false;
  std::set<int> scratch_waiters;
  std::set<int> waiters;
  for (auto i = _cmd_queue.begin(); i != _cmd_queue.end(); ++i) {
    base_stream_t *ip = i->get();
    if (auto stream = dynamic_cast<Barrier *>(ip)) {
      bar_scratch_read |= stream->bar_scr_rd();
      bar_scratch_write |= stream->bar_scr_wr();
      bar_remote_scratch_write |= stream->bar_scr_wr_df(); // it has no waiters
    }
    if (auto is = dynamic_cast<IPortStream*>(ip)) {
      for (auto &port : is->pes) {
        port_data_t &in_vp = _port_interf.in_port(port.port);
        auto it = std::find(active_ports.begin(), active_ports.end(), port.port);
        if (it == active_ports.end())
          continue;

        if (!(in_vp.in_use() || in_vp.completed() || in_vp.any_data())) {
          if (bar_scratch_write && ((ip->src() & LOC::SCR) != LOC::NONE)) {
            scratch_waiters.insert(port.port);
          } else {
            waiters.insert(port.port);
          }
        }
      }
    }
  }

  // if all of our unknowns are waiting for us, then maybe its scratch or cmd
  // queue
  unsigned total_waiters = scratch_waiters.size() + waiters.size();
  if (total_waiters == num_unknown_input) {
    if (scratch_waiters.size()) {
      return pipeline_stats_t::SCR_BAR_WAIT;
    }
    return pipeline_stats_t::CMD_QUEUE;
  }
  if (scratch_waiters.size() && _cmd_queue.size() >= _queue_size) {
    return pipeline_stats_t::SCR_BAR_WAIT;
  }

  any_input_activity |= (total_waiters); // waiters counts as input activity

  bool any_out_activity = false, any_cgra_activity = false;
  for (unsigned i = 0; i < active_out_ports.size(); ++i) {
    auto &out_vp = _port_interf.out_port(active_out_ports[i]);
    bool assigned_ovp = out_vp.in_use() || out_vp.completed();
    any_cgra_activity |= out_vp.num_in_flight();
    any_out_activity |= assigned_ovp || out_vp.any_data();
  }

  // No activity on this group, so ignore it
  if (!any_input_activity && !any_cgra_activity && !any_out_activity) {
    return pipeline_stats_t::NOT_IN_USE;
  }

  if (!any_input_activity) { // no input, but at least some cgra/out
    return pipeline_stats_t::DRAIN;
  }

  // some input activity

  return pipeline_stats_t::CORE_WAIT;
  // cout << num_unassigned << "/" << total_ivps << "-" <<
  // num_unassigned_queued; done(true,0);
}

// Figure out who is to blame for a given cycle's not issuing
void accel_t::whos_to_blame(
    std::vector<pipeline_stats_t::PIPE_STATUS> &blame_vec,
    std::vector<pipeline_stats_t::PIPE_STATUS> &group_vec) {
  if (_soft_config.in_ports_active.size() == 0) {
    blame_vec.push_back(pipeline_stats_t::NOT_IN_USE);
    return;
  }

  if (_in_config) {
    blame_vec.push_back(pipeline_stats_t::CONFIG);
    return;
  }

  bool draining = false, cgra_back = false;

  // Temp vec are for reasons that we consider to have priority
  // when considering blame over draining and backpressure.
  std::vector<pipeline_stats_t::PIPE_STATUS> temp_vec;

  for (int g = 0; g < NUM_GROUPS; ++g) {
    if (_dfg->group_prop(g).is_temporal)
      continue;

    pipeline_stats_t::PIPE_STATUS blame = whos_to_blame(g);
    group_vec.push_back(blame);
    switch (blame) {
    case pipeline_stats_t::DRAIN:
      draining = true;
      break;
    case pipeline_stats_t::CGRA_BACK:
      cgra_back = true;
      break;
    case pipeline_stats_t::NOT_IN_USE:
      break;
    default:
      temp_vec.push_back(blame);
      break;
    }
  }

  if (_dedicated_cgra_issued > 1) {
    blame_vec.push_back(pipeline_stats_t::ISSUED_MULTI);
    return;
  }
  if (_dedicated_cgra_issued == 1) {
    blame_vec.push_back(pipeline_stats_t::ISSUED);
    return;
  }

  if (_backcgra_issued != 0) {
    blame_vec.push_back(pipeline_stats_t::TEMPORAL_ONLY);
    return;
  }

  if (temp_vec.size() == 0) {
    if (draining) {
      blame_vec.push_back(pipeline_stats_t::DRAIN);
      return;
    }
    if (cgra_back) {
      blame_vec.push_back(pipeline_stats_t::CGRA_BACK);
      return;
    }
    blame_vec.push_back(pipeline_stats_t::CORE_WAIT);
    return;
  }

  for (auto i : temp_vec) {
    blame_vec.push_back(i);
  }

  // bool any_stream_active =
  //   _dma_c.any_stream_active() || _scr_r_c.any_stream_active() ||
  //   _scr_w_c.any_stream_active();

  // bool any_buf_active = _scr_r_c.buf_active() || _scr_w_c.buf_active();

  // bool mem_rds = _dma_c.mem_reads_outstanding();
  // bool mem_wrs = _dma_c.mem_writes_outstanding();
  // bool scr_req = _dma_c.scr_reqs_outstanding();

  // bool cgra_input = cgra_input_active();
  // bool cgra_compute = cgra_compute_active();
  // bool cgra_output = cgra_output_active();
  // bool cgra_active = cgra_input || cgra_compute || cgra_output;

  // bool queue = _cmd_queue.size();

  // bool busy = any_stream_active || any_buf_active || cgra_active ||
  //               mem_wrs || mem_rds || scr_req || queue;

  // if(!busy) return pipeline_stats_t::NO_ACTIVITY;
}

struct StreamExecutor : dsa::sim::stream::Functor {

  void Visit(ConstPortStream *cps) override {
    auto &stream = *cps;
    auto &pi = accel->port_interf();
    int pushed = 0;
    while (stream.stream_active() && pushed < accel->get_ssim()->spec.cache_line) {
      for (auto &elem : stream.pes) {
        auto &in = pi.in_port(elem.port);
        in.push_data(in.get_byte_vector(stream.ls->poll(false), stream.data_width()));
      }
      auto value = stream.ls->poll(true);
      LOG(CONST) << "Const value pushed: " << value;
      LOG(CONST) << stream.ls->toString();
      pushed += stream.data_width();
    }
    accel->add_bw(stream.src(), stream.dest(), 1, pushed);
    if (!stream.stream_active()) {
      accel->process_stream_stats(stream);
      for (auto &elem : stream.pes) {
        auto &in = pi.in_port(elem.port);
        in.freeStream();
      }
    }
  }

  void requestDMAWrite(OPortStream *ops,
                       std::vector<uint8_t> &data, int bytes,
                       const dsa::sim::stream::LinearStream::LineInfo &info) {

    SSMemReqInfoPtr sdInfo =
        new SSMemReqInfo(ops->id(), accel->accel_index(), MEM_WR_STREAM);

    std::ostringstream oss;
    for (int i = 0, n = data.size(); i < n; ++i) {
      oss << " " << ((int) data[i]);
    }
    LOG(MEM_REQ) << accel->get_ssim()->CurrentCycle() << " " << data.size()
                 << "-byte write request for port->dma, addr:" << info.start
                 << oss.str() << " " << ops->toString();

    // make store request
    accel->lsq()->pushRequest(ops->minst(), false /*isLoad*/, data.data(),
                              bytes, info.start, 0 /*flags*/, 0 /*res*/,
                              nullptr/*atomic op*/,
                              std::vector<bool>(), sdInfo);

    accel->_stat_mem_bytes_wr += bytes;

    if (accel->get_ssim()->in_roi()) {
      accel->add_bw(ops->src(), ops->dest(), 1, bytes);
      accel->_stat_tot_stores++;
      accel->_stat_tot_mem_stored += data.size();
    }

  }

  void requestDMARead(int sid, int padding,
                      const std::vector<int> &ports,
                      const dsa::sim::stream::LinearStream::LineInfo &info) {

    int cacheline = accel->get_ssim()->spec.cache_line;
    SSMemReqInfoPtr sdInfo = new SSMemReqInfo(sid, accel->accel_index(), ports,
                                              info.mask, accel->get_cur_cycle(),
                                              padding, info.stride_first,
                                              info.stride_last, info.stream_last);
    accel->lsq()->sd_transfers[ports[0]].reserve();
    // make request
    accel->lsq()->pushRequest(accel->cur_minst(), true /*isLoad*/, NULL /*data*/,
                              cacheline /*cache line*/, info.linebase, 0 /*flags*/,
                              0 /*res*/, nullptr /*atomic op*/,
                              std::vector<bool>() /*byte enable*/,
                              sdInfo);

    std::ostringstream oss;
    int bytes_read = 0;
    oss << " ";
    for (int i = 0, n = info.mask.size(); i < n; ++i) {
      oss << ((int) info.mask[i]);
      bytes_read += info.mask[i];
    }
    LOG(MEM_REQ) << accel->get_ssim()->CurrentCycle() << "\t"
                 << "request for " << info.linebase << " for "
                 << bytes_read;

  }

  void Visit(LinearReadStream *lrs) override {
    auto &stream = *lrs;
    if (stream.stream_active()) {

      int in_port = stream.pes[0].port;
      auto &in_vp = accel->port_interf().in_port(in_port);

      // this is number of vectors ready (each assumed to be 64-bit?)
      bool no_space = (in_vp.num_ready() * in_vp.port_vec_elem() + (in_vp.mem_size())*8/in_vp.get_port_width()) > CGRA_FIFO_LEN;
      // bool no_space = in_vp.can_push_bytes_vp(1);

      if (no_space) {
        return;
      }

      if (stream.src() == LOC::DMA) {
        if (!accel->lsq()->sd_transfers[in_port].unreservedRemainingSpace() ||
            !accel->lsq()->canRequest()) {
          return;
        }
      } else {
        CHECK(stream.src() == LOC::SCR);
        if (!accel->spads[0].rb->Available()) {
          return;
        }
      }

      {
        int cacheline = accel->get_ssim()->spec.cache_line;
        int available = cacheline;
        if (stream.be) {
          available = std::min(stream.be->occupied, available);
        }
        auto info = stream.ls->cacheline(cacheline, available);
        info.padding = stream.padding;
        int bytes_read = 0;
        std::vector<bool> bm = info.mask;
        std::vector<int> ports;
        for (auto &elem : stream.pes) {
          ports.push_back(elem.port);
        }
        if (stream.src() == LOC::DMA) {
          if (accel->get_ssim()->in_roi()) {
            accel->add_bw(stream.src(), stream.dest(), 1, bytes_read);
            accel->_stat_tot_mem_fetched += bytes_read;
            accel->_stat_tot_loads += 1;
          }
          if (!stream.stream_active()) {
            accel->process_stream_stats(stream);
          }
          requestDMARead(stream.id(), stream.padding, ports, info);
        } else {
          if (accel->get_ssim()->in_roi()) {
            accel->add_bw(stream.src(), stream.dest(), 1, bytes_read);
            accel->_stat_tot_mem_fetched += bytes_read;
            accel->_stat_tot_loads += 1;
          }
          if (!stream.stream_active()) {
            accel->process_stream_stats(stream);
          }
          if (stream.be) {
            info.start = stream.be->Translate(info.start, false);
            info.linebase = stream.be->Translate(info.linebase, true);
          }
          accel->spads[0].rb->Decode(ports[0], dsa::sim::Request::Operation::read, info, {});
        }
        LOG(MEM_REQ) << "read request: " << info.linebase << ", " << info.start
                     << " for " << stream.toString();
      }
    }
  }

  void Visit(IndirectReadStream *irs) {
    auto &idxp = accel->port_interf().out_port(irs->idx_port);
    if (!idxp.mem_size()) {
      return;
    }
    std::vector<int> ports;
    for (auto &elem : irs->pes) {
      ports.push_back(elem.port);
    }
    if (irs->src() == LOC::DMA) {
      if (!accel->lsq()->canRequest() ||
          !accel->lsq()->sd_transfers[irs->idx_port].canReserve()) {
        return;
      }
      auto index = idxp.peek_out_data();
      idxp.pop_out_data();
      uint64_t cacheline = accel->get_ssim()->spec.cache_line;
      std::vector<bool> bm(cacheline, false);
      auto addr = (irs->start + index * irs->data_width());
      LOG(IND)
        << irs->start << " + " << index << " * " << irs->data_width() << " = " << addr;
      int linebase = addr & ~(cacheline - 1);
      for (int i = 0; i < irs->data_width(); ++i) {
        bm[i + addr % cacheline] = true;
      }
      dsa::sim::stream::LinearStream::LineInfo info(linebase, addr, bm);
      ++irs->i;
      if (!irs->stream_active()) {
        accel->process_stream_stats(*irs);
        info.stream_last = true;
      }
      requestDMARead(irs->id(), 0, ports, info);
    } else {
      CHECK(false) << "Not supported yet!";
    }
  }

  void Visit(LinearWriteStream *lws) {
    auto &stream = *lws;
    CHECK(stream.stream_active()) << "Inactive stream should be freed!";
    port_data_t &out_port = accel->port_interf().out_port(stream.port);
    if (!out_port.mem_size()) {
      return;
    }
    if (stream.garbage()) {
      while (stream.stream_active() && out_port.mem_size() > 0) {
        out_port.pop_out_data(); // get rid of data
        int cacheline = accel->get_ssim()->spec.cache_line;
        stream.ls->cacheline(cacheline, out_port.get_port_width());
      }
      return;
    } else {
      // Check if the write unit is available.
      if (lws->dest() == LOC::DMA) {
        if ((out_port.mem_size() > 0)) {
          if (accel->lsq()->canRequest() &&
                     accel->lsq()->sd_transfers[MEM_WR_STREAM].canReserve()) {
            accel->lsq()->sd_transfers[MEM_WR_STREAM].reserve();
          } else {
            return;
          }
        }
      } else {
        if (!accel->spads[0].rb->Available()) {
          return;
        }
      }
      // Prepare the data according to the port data availability and memory bandwidth
      int memory_bw = accel->get_ssim()->spec.cache_line;
      auto &ovp = out_port;
      int addressable = accel->get_ssim()->spec.memory_addressable;
      int port_available = ovp.mem_size() * ovp.get_port_width();
      // Make sure the value requested does not exceed the buffet buffer.
      if (lws->be) {
        port_available =
          std::min(lws->be->SpaceAvailable(), port_available);
      }
      auto info = stream.ls->cacheline(memory_bw, std::min(memory_bw, port_available));
      int to_pop = std::accumulate(info.mask.begin(), info.mask.end(), (int) 0, std::plus<int>());
      to_pop *= addressable;
      CHECK(to_pop % ovp.get_port_width() == 0);
      std::vector<uint8_t> data;
      for (int i = 0; i < to_pop / ovp.get_port_width(); ++i) {
        auto temp = ovp.peek_out_data();
        // If we reinterpret a int64, say x, in bytes, it will be
        // [x&255, x>>8&255, ...] in the byte array.
        uint8_t *ptr = (uint8_t*)(&temp);
        std::vector<uint8_t> value(ptr, ptr + ovp.get_port_width());
        data.insert(data.end(), value.begin(), value.end());
        ovp.pop_out_data();
      }
      if (lws->dest() == LOC::DMA) {
        requestDMAWrite(lws, data, to_pop, info);
      } else {
        if (lws->be) {
          lws->be->Append(data.size());
          info.start = lws->be->Translate(info.start, false);
          info.linebase = lws->be->Translate(info.linebase, true);
          LOG(BUFFET) << "Append " << data.size() << " bytes, " << lws->be->toString();
        }
        std::vector<uint8_t> before(info.start - info.linebase, 0);
        data.insert(data.begin(), before.begin(), before.end());
        std::vector<uint8_t> after(info.mask.size() - data.size(), 0);
        data.insert(data.end(), after.begin(), after.end());
        CHECK(data.size() == info.mask.size());
        accel->spads[0].rb->Decode(stream.port, dsa::sim::Request::Operation::write, info, data);
      }
      if (!stream.stream_active()) {
        accel->process_stream_stats(stream);
        LOG(VP_SCORE2) << "SOURCE: PORT->DMA";
        out_port.freeStream();
      }
      LOG(MEM_REQ) << "write request: " << info.linebase << ", " << info.start
                   << " for " << stream.toString();
    }
  }

  void Visit(PortPortStream *pps) {
    auto &ovp = accel->port_interf().out_port(pps->oport);
    CHECK(pps->dtype % ovp.get_port_width() == 0);
    std::vector<uint8_t> data;
    int port_width = ovp.get_port_width();
    while (ovp.mem_size() >= pps->dtype / port_width && pps->stream_active()) {
      for (int i = 0; i < pps->dtype / port_width; ++i) {
        auto value = ovp.pop_out_data();
        auto *ptr = (uint8_t*) (&value);
        data.insert(data.end(), ptr, ptr + port_width);
      }
      ++pps->i;
    }
    for (auto &elem : pps->iports) {
      auto &ivp = accel->port_interf().in_port(elem.port);
      ivp.push_data(data);
    }
    if (!pps->stream_active()) {
      accel->process_stream_stats(*pps);
      LOG(VP_SCORE2) << "SOURCE: PORT->DMA";
      ovp.freeStream();
      for (auto &elem : pps->iports) {
        accel->port_interf().in_port(elem.port).freeStream();
      }
    }
  }

  StreamExecutor(accel_t *accel_) : accel(accel_) {}

  accel_t *accel;
};

void accel_t::tick() {

  _cgra_issued = 0; // for statistics reasons
  _dedicated_cgra_issued = 0;
  _backcgra_issued = 0;

  for (int i = 0; i < NUM_GROUPS; ++i) {
    _cgra_issued_group[i] = false;
  }

  auto scheduled = arbiter->Arbit(_soft_config, port_interf());
  for (auto stream : scheduled) {
    StreamExecutor se(this);
    stream->Accept(&se);
  }

  auto spad_response = spads[0].Step();
  if (spad_response.id != -1) {
    if (spad_response.op == dsa::sim::Request::Operation::read) {
      auto &vp = port_interf().in_port(spad_response.id);
      CHECK(vp.stream)
        << "Stream affiliated with the port is no longer active! " << vp.port();

      struct PostProcess : dsa::sim::stream::Functor {

        void Visit(IPortStream *is) override {
          is_is = true;
          std::vector<uint8_t> data;
          data = apply_mask(response.raw.data(), response.info.mask);
          LOG(MEM_REQ)
            << "SPAD response stream " << is->id() << ", "
            << "base: " << response.info.linebase << ", "
            << "start: " << response.info.start << ", "
            << "data: " << data.size() << " bytes";
          for (auto &elem : is->pes) {
            auto &ivp = accel->port_interf().in_port(elem.port);
            ivp.push_data(data);
            ivp.PadData(response.info.stride_first,
                        response.info.stride_last,
                        static_cast<Padding>(response.info.padding));
            if (response.info.stream_last) {
              ivp.freeStream();
            }
          }
        }

        void Visit(LinearReadStream *lrs) override {
          Visit(static_cast<IPortStream*>(lrs));
          if (lrs->be) {
            auto new_addr = std::max(lrs->be->address, response.info.shrink);
            auto to_pop = new_addr - lrs->be->address;
            if (to_pop) {
              LOG(MEM_REQ) << "Buffet shrink " << lrs->be->address << " -> " << new_addr;
            }
            lrs->be->Shrink(to_pop);
          }
        }

        PostProcess(const dsa::sim::Response &response_, accel_t *accel_) :
          response(response_), accel(accel_) {}

        bool is_is{false};
        const dsa::sim::Response &response;
        accel_t *accel;
      };

      PostProcess pp(spad_response, this);
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
    if (!performed_atomic_scr) {
      _scr_r_c.cycle(performed_read);
    }
  }
  if (_scr_ctrl_turn == 1) {
    _scr_r_c.cycle(performed_read);
    _scr_w_c.cycle(!performed_read, performed_atomic_scr);
  }
  _scr_ctrl_turn = (_scr_ctrl_turn + 1) % 2;
  // TODO: remove this after removing the above logic
  _scr_r_c.linear_scratch_cycle();
  _scr_w_c.linear_scratch_write_cycle();

  _net_c.cycle();

  cycle_in_interf();
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

    if (SS_DEBUG::CYC_STAT && _accel_index == SS_DEBUG::ACC_INDEX) {
      if (_back_cgra) {
        cycle_status_backcgra();
      } else {
        cycle_status();
      }
    }

    std::vector<pipeline_stats_t::PIPE_STATUS> blame_vec;
    std::vector<pipeline_stats_t::PIPE_STATUS> group_vec;

    whos_to_blame(blame_vec, group_vec);
    if (SS_DEBUG::BLAME && _accel_index == SS_DEBUG::ACC_INDEX &&
        group_vec.size() && (blame_vec.size() != group_vec.size())) {
      for (int g = 0; g < group_vec.size(); ++g) {
        if (_soft_config.in_ports_active_group[g].size()) {
          cout << " " << pipeline_stats_t::name_of(group_vec[g]);
        }
      }
      cout << " >";
    }

    for (auto i : blame_vec) {
      if (SS_DEBUG::BLAME && _accel_index == SS_DEBUG::ACC_INDEX) {
        cout << " " << pipeline_stats_t::name_of(i);
      }

      _pipe_stats.pipe_inc(i, 1.0f / (float)blame_vec.size());
    }

    if ((SS_DEBUG::CYC_STAT || SS_DEBUG::BLAME) &&
        _accel_index == SS_DEBUG::ACC_INDEX) {
      cout << "\n";
    }

    // if(_accel_index==0) {
    //  if(new_now!=_prev_roi_clock+1) {
    //    cout << "\n\n";
    //  }
    //  timestamp();
    //  cout << pipeline_stats_t::name_of(who) << "\n";
    //}

    //_prev_roi_clock=new_now;
  }

  cycle_out_interf();
  cycle_indirect_interf();
  schedule_streams();
  _waiting_cycles++;

  _dma_c.finish_cycle();
  _scr_w_c.finish_cycle();
  _scr_r_c.finish_cycle();
  _net_c.finish_cycle();

  if(_stream_cleanup_mode) {
    request_reset_streams();
  }

}

// forward from indirect inputs to indirect outputs -- this makes the protocol
// simpler than it used to be
void accel_t::cycle_indirect_interf() {
  for (unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i) { // ind read ports
    if(_ssim->num_active_threads()>1) {
      if(i==ATOMIC_ADDR_PORT || i==ATOMIC_BYTES_PORT) continue; //dedicated ports
    }
    auto &ind_in_port = _port_interf.in_port(i);
    auto &ind_out_port = _port_interf.out_port(i);
    
    // we only allow 8 words in the output vector port, since there's already
    // buffering in the input vector port, so it would be kind of cheating
    while (ind_in_port.mem_size() && ind_out_port.mem_size() < 64) {
      ind_out_port.push_data(ind_in_port.pop_in_custom_data<uint8_t>());
    }
  }
}

void accel_t::cycle_in_interf() {
  for (unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    int cur_port = _soft_config.in_ports_active[i];
    _port_interf.in_port(cur_port).reformat_in_one_vec();
  }
}

void accel_t::cycle_out_interf() {
  for (unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    int cur_port = _soft_config.out_ports_active[i];
    _port_interf.out_port(cur_port).reformat_out_one_vec();
  }
}

// Simulate the portion of the dataflow graph which requires either
// 1. backpressure, or 2. temporal sharing.
// Ports relevant for this simulation this are called "active_in_ports_bp"

void accel_t::cycle_cgra_backpressure() {

  int num_computed = 0;
  auto &active_in_ports = _soft_config.in_ports_active_backcgra;

  auto vps = _sched->ssModel()->subModel()->vport_list();
  for (int i = 0; i < active_in_ports.size(); ++i) {
    int port_index = active_in_ports[i];
    auto &cur_in_port = _port_interf.in_port(port_index);

    auto vp_iter = std::find_if(vps.begin(), vps.end(), [port_index] (ssvport *vp) {
      return vp->port() == port_index && vp->in_links().empty();
    });
    CHECK(vp_iter != vps.end());
    ssvport *vp = *vp_iter;
    auto *vec_in = dynamic_cast<dsa::dfg::InputPort*>(_sched->dfgNodeOf(vp));
    // TODO: make it compatible for different datawidth of the port and repeat
    // count
    // TODO(@were): Flagged repeat port cannot be easily supported.
    // if(cur_in_port.repeat_flag() && cur_in_port.num_times_repeated()==0 && cur_in_port.num_ready() && vec_in->can_push()) { // need to pop to set it
    //   auto &repeat_prt = port_interf().out_port(cur_in_port.repeat());
    //   if(repeat_prt.mem_size()) {
    //     uint64_t x = repeat_prt.pop_out_data();
    //     if(SS_DEBUG::MEM_REQ) {
    //       cout << "Port details: " << cur_in_port.port_cgra_elem() << " " << vec_in->logical_len() << " ( " << vec_in->name() << " ) "<< endl;
    //       cout << "NEW REPEAT COUNT: " << x << endl;
    //     }
    //     int y = (x << REPEAT_FXPNT);
    //     cur_in_port.set_cur_repeat_lim(y);
    //   } else { // sstream_size data is not yet available
    //     cur_in_port.set_cur_repeat_lim(-1);
    //   }
    // }

    LOG(REPEAT)
         << "Port details: " << cur_in_port.port_cgra_elem() << " "
         << vec_in->logical_len() << " ( " << vec_in->name() << " ) "
         << cur_in_port.pes.toString()
         << " and num_ready: " << cur_in_port.num_ready()
         << " and mem size: " << cur_in_port.mem_size();

    if (cur_in_port.num_ready() && cur_in_port.pes.repeat > 0) {

      assert(vec_in != NULL && "input port pointer is null\n");

      if (vec_in->can_push()) {
        forward_progress();

        // execute_dfg
        vector<SBDT> data;
        vector<bool> data_valid;
        SBDT val = 0;
        bool valid = false;

        CHECK(cur_in_port.port_cgra_elem() == vec_in->logical_len())
          << cur_in_port.port_cgra_elem() << " " << vec_in->logical_len()
          << " ( " << vec_in->name() << " )";

        for (unsigned port_idx = 0; port_idx < cur_in_port.port_cgra_elem(); ++port_idx) {
          // port_idx are the scalar cgra nodes
          int cgra_port = cur_in_port.cgra_port_for_index(port_idx);
          if (_soft_config.cgra_in_ports_active[cgra_port] == false) {
            break;
          }

          // get the data of the instance of CGRA FIFO
          val = cur_in_port.value_of(port_idx, 0);
          valid = cur_in_port.valid_of(port_idx, 0);
          data.push_back(val);
          data_valid.push_back(valid);
        }

        // TODO: this is supposed to be the verif flag, fix it later!
        // num_computed = _dfg->push_vector(vec_in, data, data_valid, print, true);
        for (int i = 0; i < data.size(); ++i) {
          vec_in->values[i].push(data[i], data_valid[i], 0);
        }

        LOG(COMP) << "Vec name: " << vec_in->name() << " allowed to push input: ";
        for (size_t i = 0; i < data.size(); ++i) {
          LOG(COMP) << &vec_in->values[i] << "|" << data[i] << "(" << data_valid[i] << ") ";
        }

        // pop input from CGRA port after it is pushed into the dfg node
        bool should_pop = cur_in_port.pes.tick();
        LOG(PORTS) << cur_in_port.pes.toString();
        if (should_pop) {
          LOG(PORTS) << "pop! " << cur_in_port.num_ready();
          cur_in_port.pop(1);
        }

      }

    } else if(cur_in_port.num_ready() && cur_in_port.pes.repeat==0) { // when the element has to be repeated 0 times (discard)
      // cout << "Came inside repeat=0 for Vec name: " << vec_in->name() << "\n";
      cur_in_port.pop(1);
    }
  }

  // calling with the default parameters for now
  num_computed = _dfg->forward(_back_cgra);

  _cgra_issued += _dfg->total_dyn_insts(0) + _dfg->total_dyn_insts(1);
  _dedicated_cgra_issued += _dfg->total_dyn_insts(0);
  _backcgra_issued += _dfg->total_dyn_insts(1);

  if (in_roi()) {
    _stat_ss_insts += num_computed;
    _stat_ss_dfg_util += (double)num_computed / _dfg->instructions.size();
  }

  // pop the ready outputs
  auto &active_out_ports = _soft_config.out_ports_active;
  vector<SBDT> data;
  vector<bool> data_valid;

  for (int i = 0; i < active_out_ports.size(); ++i) {
    int port_index = active_out_ports[i];
    auto &cur_out_port = _port_interf.out_port(port_index);

    auto vp_iter = std::find_if(vps.begin(), vps.end(), [port_index] (ssvport *vp) {
      return vp->port() == port_index && vp->out_links().empty();
    });
    CHECK(vp_iter != vps.end());
    ssvport* vp = *vp_iter;
    auto *vec_output = dynamic_cast<dsa::dfg::OutputPort*>(_sched->dfgNodeOf(vp));

    assert(vec_output != NULL && "output port pointer is null\n");

    int data_width = cur_out_port.get_port_width();
 
    // we need to send the real vec_outputs
    int len = ceil(cur_out_port.port_vec_elem()*cur_out_port.get_port_width()/float(8));
    if (vec_output->can_pop()) {
      forward_progress();
      data.clear();       // make sure it is empty here
      data_valid.clear(); // make sure it is empty here
      vec_output->pop(data, data_valid);

      if(SS_DEBUG::COMP) {
        cerr << "outvec[" << vec_output->name() << "] allowed to pop output: ";
        assert(data_valid.size() == data.size());
        for (int j = 0, n = data.size(); j < n; ++j) {
          std::cout << data[j] << "(" << data_valid[j] << ") ";
        }
        cerr << "\n";
      }
      if (in_roi()) {
        _stat_comp_instances += 1;
      }

      CHECK(data_valid.size() == len) << "port vec elem: " << cur_out_port.port_vec_elem()
                                      << " port width: " << cur_out_port.get_port_width()
                                      << " length of output required: " << len
                                      << " and size: " << data_valid.size();

      int j = 0, n_times=0;
      for (j = 0; j < len; ++j) {
        // push the data to the CGRA output port only if discard is not 0
        if (data_valid[j]) {

          n_times = 8/data_width;
          if(vec_output->logical_len()-j*n_times>0) {
            n_times = std::min(n_times, (int)vec_output->logical_len()-j*n_times);
          }
          // push elements from dfg output port
          for(int k=0; k<n_times; ++k) {
            vector<uint8_t> v = cur_out_port.get_byte_vector(
              data[j] >> (k*data_width*8), data_width);
            cur_out_port.push_data(v);
          }
          // cur_out_port.inc_ready(1); // FIXME:CHECKME: I hope this is correct!
        } 
      }
    } else {
      // cout << "Vec name: " << vec_output->name() << " not allowed to pop output: ";
    }
  }

  // Statistics to measure port imbalance
  if(_ssim->in_roi() && _ssim->in_use()) { // would measure only when something was issued
    // TODO(@were): Fix this.
    // _stat_port_imbalance += _dfg->count_starving_nodes();
  }
}

void accel_t::cycle_cgra() {
  // printf("ACCEL ID: %d and cycle: %d\n", _lsq->getCpuId(), get_cur_cycle());
  if (!_dfg)
    return;

  // cout << "issuing cycle cgra backpressure\n";

  cycle_cgra_backpressure();

  if (in_roi()) {
    _stat_cgra_busy_cycles += (_cgra_issued > 0);
  }

  // printf("ACCEL ID: %d\n", _lsq->getCpuId());
  if (_cgra_issued > 0 && SS_DEBUG::COMP && SS_DEBUG::NET_REQ) {
    printf("ACCEL ID: %d\n", lsq()->getCpuId());
  }
}

// Print out a string on one line indicating hardware status for the previous
// cycle
// Buffer Sizes                                     |      Bus Activity
// ip 1:5 2:5 7:7; op 1:2 scr_wr:1 cq:1 mem_req:14  | ip: op: scr_rd: scr_wr:
// mr:
void accel_t::cycle_status() {
  if (_soft_config.in_ports_active.size() == 0) {
    return;
  }

  timestamp();
  cout << "cq" << _cmd_queue.size();

  for (int group = 0; group < NUM_GROUPS; ++group) {
    auto &active_ports = _soft_config.in_ports_active_group[group];
    if (active_ports.size()) {
      cout << "|";
      for (unsigned i = 0; i < active_ports.size(); ++i) {
        unsigned cur_p = active_ports[i];
        cout << "i" << cur_p << ":" << _port_interf.in_port(cur_p).mem_size()
             << "," << _port_interf.in_port(cur_p).num_ready();
        auto &in_port = _port_interf.in_port(active_ports[i]);
        if (in_port.in_use()) {
          cout << base_stream_t::loc_short_name(in_port.loc());
          if (in_port.completed()) {
            cout << "#";
          }
        }
        cout << " ";
      }
    }
  }

  // for(unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i ) {  //ind read ports
  //  unsigned cur_p = i;
  //  cout << " " << cur_p << ": "
  //            << _port_interf.in_port(cur_p).mem_size()  << ","
  //            << _port_interf.in_port(cur_p).num_ready() <<" ";
  //}
  cout << "\t";
  for (int group = 0; group < NUM_GROUPS; ++group) {
    auto &active_ports = _soft_config.out_ports_active_group[group];
    if (active_ports.size()) {
      cout << "|";
      for (unsigned i = 0; i < active_ports.size(); ++i) {
        unsigned cur_p = active_ports[i];
        auto &out_port = _port_interf.out_port(cur_p);
        cout << "o" << cur_p << ":" << out_port.num_in_flight() << "-"
             << out_port.num_ready() << "," << out_port.mem_size();
        if (out_port.in_use()) {
          cout << base_stream_t::loc_short_name(out_port.loc());
        }
        cout << " ";
      }
    }
  }

  cout << "m_req:" << _dma_c.mem_reqs() << " ";

  cout << "\t|";

  cout << "s_rd" << _stat_scr_bytes_rd << " s_wr:" << _stat_scr_bytes_wr
       << " m_rd:" << _stat_mem_bytes_rd << " m_wr:" << _stat_mem_bytes_wr
       << " ";

  // Just the indirect ports
  //  for(unsigned i = 24; i < 32; ++i) {
  //    int cur_p=i;
  //    if(_port_interf.in_port(cur_p).in_use()) {
  //      cout << cur_p << " "  <<
  //      (_port_interf.in_port(cur_p).completed()?"(completed)":"");
  //    }
  //  }

  // Just the indirect ports
  //  for(unsigned i = 24; i < 32; ++i) {
  //    int cur_p=i;
  //    if(_port_interf.out_port(cur_p).in_use()) {
  //      cout << cur_p << " " <<
  //      (_port_interf.out_port(cur_p).completed()?"(completed)":"");
  //    }
  //  }

  clear_cycle();
}

void accel_t::cycle_status_backcgra() {
  if (_soft_config.in_ports_active_backcgra.size() == 0) {
    return;
  }

  timestamp();
  cout << "cq" << _cmd_queue.size();
  auto &active_in_ports = _soft_config.in_ports_active_backcgra;

  if (active_in_ports.size()) {
    cout << "|";
    for (unsigned i = 0; i < active_in_ports.size(); ++i) {
      unsigned cur_p = active_in_ports[i];
      cout << "i" << cur_p << ":" << _port_interf.in_port(cur_p).mem_size()
           << "," << _port_interf.in_port(cur_p).num_ready();
      auto &in_port = _port_interf.in_port(active_in_ports[i]);
      if (in_port.in_use()) {
        cout << base_stream_t::loc_short_name(in_port.loc());
        if (in_port.completed()) {
          cout << "#";
        }
      }
      cout << " ";
    }
  }

  // for(unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i ) {  //ind read
  // ports
  //  unsigned cur_p = i;
  //  cout << " " << cur_p << ": "
  //            << _port_interf.in_port(cur_p).mem_size()  << ","
  //            << _port_interf.in_port(cur_p).num_ready() <<" ";
  //}
  cout << "\t";
  auto &active_out_ports = _soft_config.out_ports_active;
  if (active_out_ports.size()) {
    cout << "|";
    for (unsigned i = 0; i < active_out_ports.size(); ++i) {
      unsigned cur_p = active_out_ports[i];
      auto &out_port = _port_interf.out_port(cur_p);
      cout << "o" << cur_p << ":" << out_port.num_in_flight() << "-"
           << out_port.num_ready() << "," << out_port.mem_size();
      if (out_port.in_use()) {
        cout << base_stream_t::loc_short_name(out_port.loc());
      }
      cout << " ";
    }
  }

  cout << "m_req:" << _dma_c.mem_reqs() << " ";

  cout << "\t|";

  //  cout << "req:"
  cout << "s_rd" << _stat_scr_bytes_rd << " s_wr:" << _stat_scr_bytes_wr
       << " m_rd:" << _stat_mem_bytes_rd << " m_wr:" << _stat_mem_bytes_wr
       << " ";
  //  cout << "sat:" << " m_rd:" << _stat_mem_bytes_rd_sat << " ";
  //                 << " m_wr:" << _stat_mem_bytes_wr_sat;

  // Just the indirect ports
  for (unsigned i = 24; i < 32; ++i) {
    int cur_p = i;
    if (_port_interf.in_port(cur_p).in_use()) {
      cout << cur_p << " "
           << (_port_interf.in_port(cur_p).completed() ? "(completed)" : "");
    }
  }

  // Just the indirect ports
  for (unsigned i = 24; i < 32; ++i) {
    int cur_p = i;
    if (_port_interf.out_port(cur_p).in_use()) {
      cout << cur_p << " "
           << (_port_interf.out_port(cur_p).completed() ? "(completed)" : "");
    }
  }
  clear_cycle();
}

void accel_t::clear_cycle() {
  // std::map<int,int> _stat_ivp_put;
  // std::map<int,int> _stat_ivp_get;
  // std::map<int,int> _stat_ovp_put;
  // std::map<int,int> _stat_ovp_get;
  //
  // if(_ssim->in_roi()) {
  //  _stat_tot_mem_stored+=_stat_mem_bytes_wr;
  //}

  _stat_mem_bytes_wr = 0;
  _stat_mem_bytes_rd = 0;
  _stat_scr_bytes_wr = 0;
  _stat_scr_bytes_rd = 0;
  _stat_mem_bytes_wr_sat = 0;
  _stat_mem_bytes_rd_sat = 0;

  //_stat_cmds_issued=0;
  //_stat_cmds_complete=0;
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
  for (int i = 0; i < 2; ++i) {
    for (auto elem : _soft_config.active_ports(i)) {
      if (auto stream = port_interf().ports(i)[elem].stream) {
        stream->print_status();
      }
    }
  }
  _dma_c.print_status();
  _scr_r_c.print_status();
  _scr_w_c.print_status();
  _net_c.print_status();

  cout << "Waiting SEs: (" << _cmd_queue.size() << ")\n";
  for (auto i : _cmd_queue) {
    i->print_status();
  }

  cout << "Non-empty buffers:\n";
  if (_scr_w_c.atomic_scr_issued_requests_active()) {
    cout << "ATOMIC SCR buffers not empty\n";
  }
  if(!lsq()->is_pending_net_empty()) {
    cout << "SPU network requests pending in queue\n";
  }

  cout << "Ports:\n";
  for (unsigned i = 0; i < _soft_config.in_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.in_ports_active[i];
    string &s = _soft_config.in_ports_name[cur_p];
    std::cout << "In Port " << cur_p << " " << s << ": ";
    std::cout << "  Mem Size: " << _port_interf.in_port(cur_p).mem_size() << "";
    std::cout << "  Num Ready: " << _port_interf.in_port(cur_p).num_ready()
              << " Rem writes: " << _port_interf.in_port(cur_p).get_rem_wr() // remove this!
              << "\n";
  }

  for (unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i) { // ind read
                                                                // ports
    unsigned cur_p = i;
    std::cout << "Ind In Port " << cur_p << ": ";
    std::cout << "  Mem Size: " << _port_interf.in_port(cur_p).mem_size() << "";
    std::cout << "  Num Ready: " << _port_interf.in_port(cur_p).num_ready()
              << " Rem writes: " << _port_interf.in_port(cur_p).get_rem_wr() // remove this!
              << "\n";
  }

  for (unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
    unsigned cur_p = _soft_config.out_ports_active[i];
    string &s = _soft_config.out_ports_name[cur_p];
    std::cout << "Out Port " << cur_p << " " << s << ": ";
    std::cout << "  In Flight: " << _port_interf.out_port(cur_p).num_in_flight()
              << "";
    std::cout << "  Num Ready: " << _port_interf.out_port(cur_p).num_ready()
              << "";
    std::cout << "  Mem Size: " << _port_interf.out_port(cur_p).mem_size()
              << "\n";
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
  if (SS_DEBUG::VERIF_PORT) { // flush all the log files
    in_port_verif.flush();
    out_port_verif.flush();
  }
  if (SS_DEBUG::VERIF_SCR) {
    scr_rd_verif.flush();
    scr_wr_verif.flush();
  }
  if (SS_DEBUG::VERIF_CMD) {
    cmd_verif.flush();
  }
  if (SS_DEBUG::VERIF_CGRA) {
    cgra_multi_verif.flush();
  }

  if (SS_DEBUG::SUPRESS_STATS) {
    return; // If we don't want to print stats
  }

  out << "\nACCEL " << _accel_index << " STATS ***\n";
  out.precision(4);
  out << dec;
  // out << "Start Cycle: " << _stat_start_cycle << "\n";
  // out << "Stop  Cycle: " << _stat_stop_cycle << "\n\n";
  // out << "BYTES READ AT PORT 5: " << _bytes_rd5 << "\n";

  out << "Commands Issued: " << _stat_commands_issued << "\n";
  out << "CGRA Instances: " << _stat_comp_instances << " -- Activity Ratio: "
      << ((double)_stat_cgra_busy_cycles) / ((double)roi_cycles())
      << ", DFGs / Cycle: "
      << ((double)_stat_comp_instances) / ((double)roi_cycles()) << "\n";
  if (_dfg) {
    out << "For backcgra, Average thoughput of all ports (overall): "
      << ((double)_stat_comp_instances)/((double)roi_cycles()*_dfg->vouts.size()) // gives seg fault when no dfg
      << ", CGRA outputs/cgra busy cycles: "
      <<  ((double)_stat_comp_instances)/((double)_stat_cgra_busy_cycles)  << "\n";
  }
  out << "CGRA Insts / Computation Instance: "
      << ((double)_stat_ss_insts) / ((double)_stat_comp_instances) << "\n";
  out << "CGRA Insts / Cycle: "
      << ((double)_stat_ss_insts) / ((double)roi_cycles())
      << " (overall activity factor)\n";
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

  out << "Cycle Breakdown: ";
  _pipe_stats.print_histo(out, roi_cycles());
  out << "\n";

  /*
   * Depricated
  out << "\n";
  out << "Avg. Scratch Read Port Req Size:  "
    << ((double)_stat_scratch_read_bytes)/((double)_stat_scratch_reads) << "\n";
  out << "Avg. Scratch Write Port Req Size: "
    << ((double)_stat_scratch_write_bytes)/((double)_stat_scratch_writes) <<
  "\n";

  out << "Scratch reads per cycle:  " <<
  ((double)_stat_scratch_reads)/((double)roi_cycles()) << "\n"; out << "Scratch
  writes per cycle: " << ((double)_stat_scratch_writes)/((double)roi_cycles())
  << "\n"; out << "\n";


  out << "Avg. Mem Load Port Req Size:  "
    << ((double)_stat_tot_mem_fetched)/((double)_stat_tot_loads) << "\n";
  out << "Avg. Mem Store Port Req Size: "
    << ((double)_stat_tot_mem_stored)/((double)_stat_tot_stores) << "\n";

  out << "Mem loads per cycle:    " <<
  ((double)_stat_tot_loads)/((double)roi_cycles()) << "\n"; out << "Mem stores
  per cycle:   " << ((double)_stat_tot_stores)/((double)roi_cycles()) << "\n";
  out << "\n";
*/

  /*
   // TODO: FIXME: This is useful info -- figure out how to git it
  out << "Mem load misses per cycle:  " <<
  ((double)_stat_tot_mem_load_acc)/((double)roi_cycles()) << "\n"; out << "Mem
  store misses per cycle: " <<
  ((double)_stat_tot_mem_store_acc)/((double)roi_cycles()) << "\n"; out << "\n";
*/

  // const std::vector<LOC> alocs {LOC::DMA, LOC::SCR};
  static const std::vector<LOC> locs{LOC::NONE, LOC::DMA, LOC::SCR, LOC::PORT,
                                     LOC::CONST};

  auto print_bwm = [&](LOC l1, LOC l2) {
    auto &p = _bw_map[make_pair(l1, l2)];
    if (p.second == 0)
      out << "(0 B/c, 0 B/r) ";
    else {
      out.precision(3);
      out << ((double)p.first / (double)roi_cycles());
      out << "(" << ((double)p.second / (double)roi_cycles()) << "B/c";
      out << ", " << ((double)p.second / (double)p.first) << "B/r) ";
    }
  };
  auto print_src = [&](string name, LOC l1) {
    out << name;
    print_bwm(l1, LOC::TOTAL);
    out << " -- ";
    for (auto l2 : locs) {
      if (l2 == LOC::TOTAL)
        continue;
      auto &p = _bw_map[make_pair(l1, l2)];
      if (p.first > 0) {
        out << base_stream_t::loc_name(l2) << ":";
        print_bwm(l1, l2);
      }
    }
    out << "\n";
  };

  auto print_dest = [&](string name, LOC l2) {
    out << name;
    print_bwm(LOC::TOTAL, l2);
    out << " -- ";
    for (auto l1 : locs) {
      if (l1 == LOC::TOTAL)
        continue;
      auto &p = _bw_map[make_pair(l1, l2)];
      if (p.first > 0) {
        out << base_stream_t::loc_name(l1) << ":";
        print_bwm(l1, l2);
      }
    }
    out << "\n";
  };

  out << "Bandwidth Table: (B/c=Bytes/cycle, B/r=Bytes/request) -- Breakdown "
         "(sources/destinatinos): \n";
  print_src("SP_READ:\t", LOC::SCR);
  print_dest("SP_WRITE:\t", LOC::SCR);
  print_src("DMA_LOAD:\t", LOC::DMA);
  print_dest("DMA_STORE:\t", LOC::DMA);
  print_dest("REC_BUS_READ:\t", LOC::REC_BUS);
  print_dest("NETWORK:\t", LOC::NETWORK);
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
  _ssim->add_bitmask_stream(s);
}

// --------------------------SCHEDULE STREAMS ONTO
// CONTROLLERS----------------------- This is essentially the stream dispatcher
void accel_t::schedule_streams() {

  struct StreamScheduler : dsa::sim::stream::Functor {
    StreamScheduler(accel_t *accel_) : accel(accel_) {}

    /*!
     * \brief Check the avaiability of the ports of this stream.
     * \param is The input stream to be scheduled.
     */
    bool IVPAvailable(const std::vector<PortExecState> &pes) {
      for (auto &elem : pes) {
        auto &in = accel->port_interf().in_port(elem.port);
        if (in.stream) {
          return false;
        }
        if (std::find(iports.begin(), iports.end(), elem.port) != iports.end()) {
          return false;
        }
        if (in.mem_size() || in.num_ready()) {
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

    /*!
     * \brief Update the status of ports involved by the scheduled stream.
     * \param is The scheduled stream.
     */
    void BindStreamToPorts(const vector<PortExecState> &pes, base_stream_t *stream) {
      for (auto &elem : pes) {
        accel->port_interf().in_port(elem.port).bindStream(stream);
      }
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
        LOG(BAR) << "Cannot retire because of prior streams.";
        return;
      }
      // If there are active stream affected by this barrier,
      // this barrier should not retire.
      for (auto i : accel->_soft_config.in_ports_active) {
        auto &in = accel->port_interf().in_port(i);
        if (in.stream && (in.stream->barrier_mask() & bar->_mask)) {
          LOG(BAR) << "Cannot retire because of active streams.";
          return;
        }
      }
      for (auto i : accel->_soft_config.out_ports_active) {
        auto &out = accel->port_interf().out_port(i);
        if (out.stream && (out.stream->barrier_mask() & bar->_mask)) {
          LOG(BAR) << "Cannot retire because of active streams.";
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
      if (!(ips->barrier_mask() & barrier_mask)) {
        if (IVPAvailable(ips->pes)) {
          auto cloned = ips->clone();
          BindStreamToPorts(cloned->pes, cloned);
          retire = true;
        }
      }
      std::for_each(ips->pes.begin(), ips->pes.end(),
                    [this] (const PortExecState &pes) { iports.push_back(pes.port); });
    }

    /*!
     * \brief Schedule the write stream.
     */
    void Visit(OPortStream *ops) override {
      barred |= ops->barrier_mask();
      if (!(ops->barrier_mask() & barrier_mask)) {
        int port = ops->port;
        auto &out = accel->port_interf().out_port(port);
        if (out.stream == nullptr &&
            std::find(oports.begin(), oports.end(), port) == oports.end()) {
          out.bindStream(ops->clone());
          retire = true;
        }
      }
      oports.push_back(ops->port);
    }

    /*!
     * \brief Schedule a port to port stream.
     */
    void Visit(PortPortStream *pps) override {
      barred |= pps->barrier_mask();
      if (!(pps->barrier_mask() & barrier_mask)) {
        int port = pps->oport;
        auto &out = accel->port_interf().out_port(port);
        if (out.stream == nullptr && IVPAvailable(pps->iports) &&
            std::find(oports.begin(), oports.end(), port) == oports.end()) {
          auto cloned = pps->clone();
          BindStreamToPorts(pps->iports, cloned);
          out.bindStream(cloned);
          retire = true;
        }
      }
      std::for_each(pps->iports.begin(), pps->iports.end(),
                    [this] (const PortExecState &pes) { iports.push_back(pes.port); });
      oports.push_back(pps->oport);
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
     * \brief The ports occupied by prior streams.
     * Think about a scenario like this:
     *
     * mem -> iport0
     * recur oport0 -> iport0
     * oport0 -> mem
     *
     * In this case, all these streams should be executed in the order they appear.
     * Recur appears earlier than port-memory, and recur cannot be scheduled until
     * memory-port stream is done. Though recur is yet bound to an oport,
     * it should be stored in the oports data structure so that the port-memory
     * can check this to know recur blocks its own execution.
     */
    std::vector<int> iports, oports;
  };

  // FIXME: check with Tony!
  if(_cleanup_mode) return;

  int str_width = _ssconfig->dispatch_width();
  int str_issued = 0;

  StreamScheduler functor(this);
  // schedule for ports (these need to go in program order per-vp)
  for (auto i = _cmd_queue.begin(); i != _cmd_queue.end() && str_issued < str_width;) {
    i->get()->Accept(&functor);
    if (functor.retire) {
      LOG(COMMAND) << i->get()->id() << " scheduled";
      i = _cmd_queue.erase(i);
      ++str_issued;
    } else {
      ++i;
      if (_ssconfig->dispatch_inorder()) {
        break;
      }
    }
    functor.Reset();

    // if (auto buffet = dynamic_cast<BuffetStream*>(ip)) {
    //   out_vp = &_port_interf.out_port(buffet->src_port);
    //   auto &shadow = _port_interf.out_port(buffet->shadow);
    //   if (ivps_can_take && !out_vp->stream && !blocked_ovp[buffet->src_port] &&
    //       !shadow.stream && !blocked_ovp[buffet->shadow]) {
    //     if ((scheduled = _scr_w_c.schedule_buffet(buffet))) {
    //       // out_vp->set_status(port_data_t::STATUS::BUSY, unit);
    //       // shadow.set_status(port_data_t::STATUS::BUSY, unit);
    //     }
    //   }
    // } else if (auto stream = dynamic_cast<remote_core_net_stream_t *>(ip)) {
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
    // } else if (auto ind_stream = dynamic_cast<indirect_wr_stream_t *>(ip)) {
    //   // int ind_port = ind_stream->_ind_port; // grab output res
    //   // int out_port = ind_stream->_out_port;
    //   // auto *ind_vp = &_port_interf.out_port(ind_port); // indirect output port
    //   // out_vp = &_port_interf.out_port(out_port);
    //   // out_vp2 = ind_vp;

    //   // if (out_vp->can_take(LOC::PORT) && ind_vp->can_take(LOC::PORT) &&
    //   //     !blocked_ovp[out_port] && !blocked_ovp[ind_port]) {
    //   //   bool succ = false;
    //   //   if (ind_stream->scratch())
    //   //     succ = _scr_w_c.schedule_indirect_wr(*ind_stream);
    //   //   else
    //   //     succ = _dma_c.schedule_indirect_wr(*ind_stream);
    //   //   if (succ) {
    //   //     scheduled = true;
    //   //     out_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   //     ind_vp->set_status(port_data_t::STATUS::BUSY, LOC::PORT);
    //   //   }
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
    //   LOG(COMMAND_I) << get_ssim()->CurrentCycle()
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
}

void data_controller_t::add_bw(LOC l1, LOC l2, uint64_t times, uint64_t bytes) {
  _accel->add_bw(l1, l2, times, bytes);
}

ssim_t *data_controller_t::get_ssim() { return _accel->get_ssim(); }

bool data_controller_t::is_shared() { return _accel->is_shared(); }

void data_controller_t::timestamp() { _accel->timestamp(); }

bool dma_controller_t::schedule_indirect(indirect_stream_t &new_s) {
  auto *s = new indirect_stream_t(new_s);
  _indirect_streams.push_back(s);
  _read_streams.push_back(s);
  return true;
}

bool dma_controller_t::schedule_indirect_wr(indirect_wr_stream_t &new_s) {
  auto *s = new indirect_wr_stream_t(new_s);
  _indirect_wr_streams.push_back(s);
  _write_streams.push_back(s);
  return true;
}

// bool scratch_read_controller_t::schedule_scr_scr(scr_scr_stream_t& new_s) {
//  for(auto& s : _scr_scr_streams) {
//    if(s.empty()) {
//      s=new_s; //copy the values
//      //cout << s._remote_stream << " became " << &s << "\n";
//
//      s._remote_stream->_remote_stream = &s;//update pointer to remote
//      //cout << "hooked up writer to stable reader\n";
//      return true;
//    }
//  }
//  return false;
//}

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

bool scratch_write_controller_t::schedule_indirect_wr(
    indirect_wr_stream_t &new_s) {
  auto *s = new indirect_wr_stream_t(new_s);
  _ind_wr_streams.push_back(s);
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

base_stream_t *scratch_write_controller_t::schedule_port_scr(LinearWriteStream &new_s) {
  auto *s = new LinearWriteStream(new_s);
  _port_scr_streams.push_back(s);
  _write_streams.push_back(s);
  return s;
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

      auto &pi = _accel->port_interf();

      PacketPtr packet = response->packet;
      if (packet->getSize() != MEM_WIDTH) {
        assert(0 && "weird memory response size");
      }

      vector<uint8_t> data;
      bool indirect=false;
      if (response->sdInfo->mask.size() > 0) {
        data = apply_mask(packet->getPtr<uint8_t>(), response->sdInfo->mask);
      } else if (response->sdInfo->map.size() > 0) {
        indirect=true;
        apply_map(packet->getPtr<uint8_t>(), response->sdInfo->map, data);
      }

      //-----------------

      bool port_in_okay = true;

      // push in byte-by-byte at the ports
      for (int in_port : response->sdInfo->ports) {
        port_data_t &in_vp = pi.in_port(in_port);
        port_in_okay = port_in_okay && (in_vp.can_push_bytes_vp(data.size()));
      }

      port_data_t &sample_vp = pi.in_port(response->sdInfo->ports[0]);
      if (port_in_okay) {
        bool last = response->sdInfo->last;
        sample_vp.inc_bytes_waiting(-data.size());
        if(indirect) { // only for mem requests
          last = sample_vp.is_bytes_waiting_zero() && sample_vp.get_is_bytes_waiting_final();
          if(last) sample_vp.reset_is_bytes_waiting_final();
        }

        std::ostringstream oss;
        for (int i = 0, n = data.size(); i < n; ++i) {
          oss << " " << ((int) data[i]);
        }

        LOG(MEM_REQ)
          << get_ssim()->CurrentCycle() << " response for "
          << std::hex << packet->getAddr() << std::dec
          << "for port " << cur_port << ", size in bytes: "
          << data.size() << " elements" << (last ? "(last)" : "") << oss.str();

        // FIXME: check if stats are reset at roi
        // _accel->_stat_mem_bytes_rd += data.size();
 
        // request for all added ports
        for (int in_port : response->sdInfo->ports) {
          port_data_t &in_vp = pi.in_port(in_port);
          in_vp.push_data(data);
          in_vp.PadData(response->sdInfo->stride_first,
                        response->sdInfo->stride_last,
                        static_cast<Padding>(response->sdInfo->fill_mode));
          if (last) {
            LOG(VP_SCORE2) << "SOURCE: DMA->PORT2 (port:" << cur_port << ")";
            in_vp.freeStream();
            sample_vp.reset_is_bytes_waiting_final();
          }
        }

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
    int NUM_ACCEL = _accel->NUM_ACCEL;
    for (uint64_t i = 0, b = 1; i < NUM_ACCEL_TOTAL; ++i, b <<= 1) {
      if (context & b) {
        _accel->_ssim->accel_arr[i]->configure(packet->getAddr(),
                                               packet->getSize() / 8,
                                               packet->getPtr<uint64_t>());
      }
    }
    _accel->lsq()->popResponse(CONFIG_STREAM);
  }

  // Memory Read to Ports
  for (unsigned i = 0; i < _accel->_soft_config.in_ports_active.size(); ++i) {
    int cur_port = _accel->_soft_config.in_ports_active[i];
    port_resp(cur_port);
  }
  for (unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i) { // ind read
                                                                // ports
    port_resp(i);
  }

  make_read_request();  // read request
  make_write_request(); // write request
}

void dma_controller_t::print_status() {
  for (auto &i : _read_streams) {
    if (!i->empty()) {
      i->print_status();
    }
  }
  for (auto &i : _write_streams) {
    if (!i->empty()) {
      i->print_status();
    }
  }
}

void dma_controller_t::finish_cycle() {}

// Delete Streams implementation
template <typename T>
void delete_stream_internal(int i, T *s, std::vector<T *> &vec,
                            std::vector<base_stream_t *> &base_vec) {
 
  base_vec.erase(base_vec.begin() + i);
  vec.erase(std::remove(vec.begin(), vec.end(), s), vec.end());
}

void dma_controller_t::delete_stream(int i, indirect_stream_t *s) {
  delete_stream_internal(i, s, _indirect_streams, _read_streams);
}
void dma_controller_t::delete_stream(int i, indirect_wr_stream_t *s) {
  delete_stream_internal(i, s, _indirect_wr_streams, _write_streams);
}

void scratch_read_controller_t::delete_stream(int i, LinearReadStream *s) {
  delete_stream_internal(i, s, _scr_port_streams, _read_streams);
}
void scratch_read_controller_t::delete_stream(int i, indirect_stream_t *s) {
  delete_stream_internal(i, s, _ind_port_streams, _read_streams);
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

void scratch_write_controller_t::delete_stream(int i,
                                               LinearWriteStream *s) {
  delete_stream_internal(i, s, _port_scr_streams, _write_streams);
}

void scratch_write_controller_t::delete_stream(int i, atomic_scr_stream_t *s) {
  delete_stream_internal(i, s, _atomic_scr_streams, _write_streams);
}
void scratch_write_controller_t::delete_stream(int i, indirect_wr_stream_t *s) {
  delete_stream_internal(i, s, _ind_wr_streams, _write_streams);
}

void dma_controller_t::make_write_request() {
  for (unsigned i = 0; i < _write_streams.size(); ++i) {
    _which_wr = (_which_wr + 1) >= _write_streams.size() ? 0 : _which_wr + 1;
    base_stream_t *s = _write_streams[_which_wr];

    if (auto *sp = dynamic_cast<indirect_wr_stream_t *>(s)) {
      auto &stream = *sp;
      if (stream.stream_active() && _accel->lsq()->canRequest() &&
          _accel->lsq()->sd_transfers[MEM_WR_STREAM].canReserve()) {

        port_data_t &out_port =
            _accel->port_interf().out_port(stream._out_port);
        port_data_t &ind_port =
            _accel->port_interf().out_port(stream._ind_port);
        auto &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
        
        if (out_port.mem_size() >= stream._data_bytes/out_port.get_port_width() && ind_port.mem_size() >= stream._index_bytes && 
           (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes/subsize_vp.get_port_width())) ) {
          // _accel->lsq()->sd_transfers[MEM_WR_STREAM].reserve();
          ind_write_req(stream);

          if (stream.empty()) {
            delete_stream(_which_wr, sp);
          }

          return;
        }
      }
    } else {
      assert(0 && "invalid type");
    }
  }
}

#define MAX_PORT_READY 100000

// FIXME: if this is 0? (somehow arbitrate among cores in that case)
void dma_controller_t::make_read_request() {
  //--------------------------
  float min_port_ready = -2;
  int max_reqs=-1;
  int num_cores = _accel->_ssim->num_active_threads();
  if(num_cores>1) {
    assert(num_cores<64 && "currently we support max 64 threads");
    if(_accel->lsq()->getCpuId()==0) num_cores=1;
    max_reqs=SD_TRANSFERS_ALLOWED/num_cores; // 22/16=1
    // if(max_reqs==0) if(_accel->get_cur_cycle()%2) max_reqs=1;
    if(mem_reqs()>=max_reqs) return; // may be in worse case
  }
  //----------------------------

  auto to_sort = _read_streams;

  //auto to_key = [this](base_stream_t *s) {
  //  int ready = 0, on_the_fly;
  //  for (auto &port : s->in_ports())
  //    ready += _accel->port_interf().in_port(port).mem_size();
  //  on_the_fly = s->on_the_fly;
  //  return std::pair<int, int>(ready, on_the_fly);
  //};

  //std::sort(to_sort.begin(), to_sort.end(), [to_key](base_stream_t *a, base_stream_t *b) {
  //  return to_key(a) < to_key(b);
  //});

  _which_rd += 1;

  for (unsigned i = 0, n = to_sort.size(); i < n; ++i) {
    _which_rd = (_which_rd + i) % to_sort.size();

    base_stream_t *s = _read_streams[_which_rd];

    if (auto *sp = dynamic_cast<indirect_stream_t *>(s)) {
      indirect_stream_t &stream = *sp;

      auto &ind_vp = _accel->port_interf().out_port(stream._ind_port);
      auto &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
      // TODO(@were): Implement indirect stream!
      int in_port = 0; // stream.first_in_port();

      if (stream.stream_active()) {

        //-----------------------------
        bool skip_check = SS_DEBUG::UNREAL_INPUTS;

        if (min_port_ready == -2) {
          min_port_ready = calc_min_port_ready();
        }
        if (!skip_check && min_port_ready >= MAX_PORT_READY)
          continue;


        auto &in_vp = _accel->port_interf().in_port(in_port);
        // if 0 space is left, then this... (include both ready and mem_size)
        // bool no_space = in_vp.can_push_bytes_vp(1); // in_vp.num_ready() * in_vp.port_vec_elem() + in_vp.mem_size() > CGRA_FIFO_LEN;

        bool no_space = (in_vp.num_ready() * in_vp.port_vec_elem() + (in_vp.mem_size())*8/in_vp.get_port_width()) > CGRA_FIFO_LEN;
        if(no_space) continue;

        if(num_cores>1) {
          if(in_vp.get_bytes_waiting()>MEM_WIDTH*max_reqs) continue; 
        }

        //--------------------------------
        // cout << "memory in ind_vp: " << ind_vp.mem_size() << "\n";  // 0
        // cout << "susize mem: " << subsize_vp.mem_size() << "\n"; // 0
        // cout << "lsq sd transfer: " <<  _accel->lsq()->sd_transfers[in_port].unreservedRemainingSpace() << endl;
        // cout << "cal lsq request: " << _accel->lsq()->canRequest() << endl;

        // if (ind_vp.mem_size() >= stream._index_bytes && 
        //   (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes)) && 
          if (ind_vp.mem_size() >= stream._index_bytes/ind_vp.get_port_width() && 
           (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes/subsize_vp.get_port_width())) && 
     
            _accel->lsq()->sd_transfers[in_port].unreservedRemainingSpace() >
                0 && _accel->lsq()->canRequest()) {

          auto &in_vp = _accel->port_interf().in_port(in_port);
          if (in_vp.num_can_push() > 8) { // FIXME:CHECKME: make sure vp isn't full
            ind_read_req(stream);

            if (stream.empty()) {
              // destination ivp
              if (SS_DEBUG::VP_SCORE2) {
                cout << "SOURCE: Indirect DMA->PORT \n";
              }
              /*if(in_vp.in_use()) { // not sure why is it required..
                in_vp.set_status(port_data_t::STATUS::COMPLETE, LOC::DMA);
              }*/

              // out_ind_vp is where we keep track of the resource
              if (SS_DEBUG::VP_SCORE2) {
                cout << "SOURCE: Indirect DMA->PORT \n";
              }
              ind_vp.freeStream();
              delete_stream(_which_rd, sp);
            }

            return;
          }
        }
      }
    } else {
      assert(0 && "invalid type");
    }
  }
}

// It works for both direct/indirect network streams
void scratch_write_controller_t::write_scratch_remote_ind(
    remote_core_net_stream_t &stream) {
  port_data_t &addr_vp = _accel->port_interf().out_port(stream._addr_port);
  port_data_t &val_vp = _accel->port_interf().out_port(stream._val_port);

  int bytes_written = 0;
  SBDT val[8];
  while(addr_vp.mem_size() && val_vp.mem_size() && stream.stream_active() &&
      bytes_written < 64) {
    uint64_t meta_info = addr_vp.pop_out_data(); // no offset here
    // this addr should have both num_bytes and addr info
    addr_t addr = meta_info & 65535; // for 16-bits
    if (SS_DEBUG::NET_REQ) {
      cout << "addr is: " << addr << "\n";
    }
    int num_bytes = meta_info >> 16;
    assert(num_bytes <= 64);
    for (int j = 0; j < num_bytes / 8; ++j) {
      val[j] = val_vp.pop_out_data();
      if (SS_DEBUG::NET_REQ) {
        timestamp();
        cout << "val being written to remote scratchpad: " << val[j] << "\n";
      }
    }
    _accel->write_scratchpad(addr, &val[0], num_bytes, stream.id());
    bytes_written += num_bytes;
  }
  _remote_scr_writes += bytes_written;

  bool is_empty = !stream.stream_active();
  if (is_empty) {
    _accel->process_stream_stats(stream);

    if (SS_DEBUG::VP_SCORE2) {
      cout << "SOURCE: INDIRECT PORT -> SCR\n";
    }
    addr_vp.freeStream();
    val_vp.freeStream();
  }

  if (_accel->_ssim->in_roi()) {
    add_bw(LOC::PORT, LOC::SCR, 1, bytes_written);
    _accel->_stat_scratch_writes++;
  }

  _accel->_stat_scr_bytes_wr += bytes_written;
  _accel->_stat_scratch_write_bytes += bytes_written;
}

void scratch_write_controller_t::write_scratch_ind(
    indirect_wr_stream_t &stream) {
  port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
  port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);

  int bytes_written = 0;

  while (ind_vp.mem_size() >= stream._index_bytes && out_vp.mem_size() && stream.stream_active() &&
         bytes_written < 64) {
    
    uint64_t ind=0;
    // FIXME: need to fix according to ind mem
    // This work for ind_vp being lower data width than indirect value config
    // (And also 8?)
    /*for(int i=0; i<stream._index_bytes; ++i) {
      uint8_t temp = ind_vp.peek_out_data(i);
      ind = ind | (temp << i*8);
    }*/
    if(stream._index_bytes > ind_vp.get_port_width()) {
      for(int i=0; i<stream._index_bytes/ind_vp.get_port_width(); ++i) {
        SBDT temp = ind_vp.peek_out_data(i);
        ind = ind | (temp << i*8*ind_vp.get_port_width());
      }
    } else {
      SBDT temp_val = ind_vp.peek_out_data();
      for(int i=0; i<stream._index_bytes; ++i) {
        uint8_t temp = (temp_val >> i*8) & 255;
        ind = ind | (temp << i*8);
      }
    }

    addr_t addr = stream.cur_addr(ind);
    // addr_t addr = stream.cur_addr(ind_vp.peek_out_data());

    bool pop_ind_vp = stream.pop_elem();
    if (pop_ind_vp) { // this is for sub-word granularity reads
      for(int i=0; i<stream._index_bytes; ++i) {
        ind_vp.pop_out_data();
      }
      // ind_vp.pop_out_data();
    }

    uint64_t val = stream.cur_value(out_vp.peek_out_data());

    // push the entry into the write bank queues
    _accel->write_scratchpad(addr, &val, stream._data_bytes, stream.id());

    bytes_written += stream._data_bytes;
    stream._ind_bytes_complete += stream._data_bytes;

    if (stream._ind_bytes_complete == 8) {
      out_vp.pop_out_data();
      stream._ind_bytes_complete = 0;
    }
  }

  bool is_empty = stream.check_set_empty();
  if (is_empty) {
    _accel->process_stream_stats(stream);

    if (SS_DEBUG::VP_SCORE2) {
      cout << "SOURCE: INDIRECT PORT -> SCR\n";
    }
    out_vp.freeStream();
    ind_vp.freeStream();
  }

  // have the same bank queues thing here
  if (_accel->_ssim->in_roi()) {
    add_bw(LOC::PORT, LOC::SCR, 1, bytes_written);
    _accel->_stat_scratch_writes++;
  }

  _accel->_stat_scr_bytes_wr += bytes_written;
  _accel->_stat_scratch_write_bytes += bytes_written;
}

void scratch_read_controller_t::read_linear_scratch_ind(
    indirect_stream_t &stream,
    uint64_t scr_addr /*if scr is dest, also todo*/) {

  port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
  auto &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
  if(!stream._is_2d_stream) { 
    assert(stream._sn_port==1);
  }

  bool first = true;
  addr_t base_addr = 0;
  addr_t max_addr = 0;

  // vector<int> imap;
  int bytes_read=0;

  while (ind_vp.mem_size() >= stream._index_bytes && stream.stream_active()
     && (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes))) {


    if(stream._is_2d_stream && stream._first_ss_access) {
      uint64_t ind2=0;
      /*for(int i=0; i<stream._index_bytes; ++i) {
        uint8_t temp = subsize_vp.peek_out_data(i);
        ind2 = ind2 | (temp << i*8);
      }*/
      if(stream._index_bytes > subsize_vp.get_port_width()) {
        for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
          SBDT temp = subsize_vp.peek_out_data(i);
          ind2 = ind2 | (temp << i*8*subsize_vp.get_port_width());
        }
      } else {
        SBDT temp_val = subsize_vp.peek_out_data();
        for(int i=0; i<stream._index_bytes; ++i) {
          uint8_t temp = (temp_val >> i*8) & 255;
          ind2 = ind2 | (temp << i*8);
        }
      }
      
      stream._sstream_size = ind2; // should be done only in the first size
      stream._first_ss_access=false;
    }



    uint64_t ind=0;
    /*for(int i=0; i<stream._index_bytes; ++i) {
      uint8_t temp = ind_vp.peek_out_data(i);
      ind = ind | (temp << i*8); // FIXME: check the endianness
    }*/
    if(stream._index_bytes > ind_vp.get_port_width()) {
      for(int i=0; i<stream._index_bytes/ind_vp.get_port_width(); ++i) {
        SBDT temp = ind_vp.peek_out_data(i);
        ind = ind | (temp << i*8*ind_vp.get_port_width());
      }
    } else {
      SBDT temp_val = ind_vp.peek_out_data();
      for(int i=0; i<stream._index_bytes; ++i) {
        uint8_t temp = (temp_val >> i*8) & 255;
        ind = ind | (temp << i*8);
      }
    }
    addr_t addr = stream.cur_addr(ind);
    // addr_t addr = stream.cur_addr(ind_vp.peek_out_data());

    // cout << "idx:" << idx << "\taddr:" << hex << addr << dec << "\n";
    if (first) {
      first = false;
      base_addr = addr & SCR_MASK;
      max_addr = base_addr + SCR_WIDTH;
    } else { // not first
      if (addr >= max_addr || addr < base_addr) {
        break;
      }
    }

    int data_bytes = stream._data_bytes;
    SBDT val=0;

    // TODO: if address is remote, then send request to read from addr and
    // data_bytes width (need a request queue?) -- or just indirect addr, and
    // bytes to the default stream

    _accel->read_scratchpad(&val, addr, data_bytes, stream.id());
    // TODO(@were): Implement this!
    // for (int in_port : stream.in_ports()) {
    //   // cout << "PUSHING DATA TO THE INPUT PORTS: " << data_bytes << " bytes\n";
    //   port_data_t &in_vp = _accel->port_interf().in_port(in_port);
    //   in_vp.push_data(in_vp.get_byte_vector(val,data_bytes));
    // }

    if (SS_DEBUG::SCR_ACC) {
      _accel->timestamp();
      cout << "linear scr indirect read for " << std::hex << base_addr << std::dec
           << " for " << data_bytes << " needed bytes"
           << " and size of stream: " << stream._num_elements
           << "\n";
    }

    bool pop_ind_vp = stream.pop_elem();
    if (pop_ind_vp) {
      stream._first_ss_access=true;
      for(int i=0; i<stream._index_bytes; ++i) {
        ind_vp.pop_out_data();
        if(stream._is_2d_stream) subsize_vp.pop_out_data();
      }
      // ind_vp.pop_out_data();
    }

    bytes_read += data_bytes;
  }

  if (bytes_read!=0 && _accel->_ssim->in_roi()) {
    add_bw(stream.src(), stream.dest(), 1, bytes_read);
    _accel->_stat_scr_bytes_rd += bytes_read;
    _accel->_stat_scratch_read_bytes += bytes_read;
    _accel->_stat_scratch_reads++;
  }
}

void scratch_read_controller_t::push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry) {
  if(SS_DEBUG::NET_REQ) {
      cout << "Read request reached with requesting core: " << req_core << " x dim: " << request_ptr << " y dim: " << reorder_entry << " and addr: " << addr << " data bytes: " << data_bytes << endl;
  }
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

    LOG(NET_REQ)
      << "Remote read request at bank: " << bank_id << " and core: "
      << _accel->lsq()->getCpuId() << " for addr: " << request.addr;
    assert(request.addr<SCRATCH_SIZE);
    _indirect_scr_read_requests[bank_id].push(request);
  }

}

// write the data into IROB (no arbitration modeled)
void scratch_read_controller_t::push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry) {
    LOG(NET_REQ)
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
      if (reorder_entry->last && stream.check_set_empty()) {
        if (SS_DEBUG::VP_SCORE2) {
          cout << "SOURCE: Indirect SCR->PORT \n";
        } 

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

void scratch_read_controller_t::read_scratch_ind(
    indirect_stream_t &stream,
    uint64_t scr_addr /*if scr is dest, also todo*/) {

//  port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
//  int ind_vp_slices = stream._index_bytes/ind_vp.get_port_width();
//
//  // STAGE 1: Push the requests to ports
//  if (stream.stream_active() && ind_vp.mem_size() >= ind_vp_slices &&
//      _ind_ROB.size() < _accel->_ind_rob_size) {
//
//    int reqd = stream._data_bytes*stream._val_num/NUM_SCRATCH_BANKS; // required bytes / maximum allowed at a time...can be split at network controller
//    int num_irob_entries_reqd = std::max(1, reqd);
//    if(_ind_ROB.size() + num_irob_entries_reqd > _accel->_ind_rob_size) return;
//
//   // cout << "Came in with slices: " << ind_vp_slices << " irob: " << num_irob_entries_reqd << endl;
//    int serve_width=stream._data_bytes;
//    // FIXME: current limitation, either data width or multiple of
//    // scratch banks
//    if(stream._val_num!=1) serve_width = NUM_SCRATCH_BANKS; 
//
//    uint64_t ind=0;
//    addr_t addr=0;
//    // TODO: accumulate all remote requests here (check for local and remote),
//    // and then send (allowed for sequential memory location, addr, 64*C bytes)
//    int acc_remote_bytes=0; addr_t init_addr=-1;
//    int init_data_ptr=-1; int init_irob_ptr=-1;
//    int last_dest_core=-1; bool is_remote = false;
//
//    bool last_remote=false;
//    bool wrap_up=false;
//    for(int req=0; req<num_irob_entries_reqd; ++req) {
//      assert(_ind_ROB.size() <= _accel->_ind_rob_size && "overflow of indirect rob");
//
//      _cur_irob_ptr = (_cur_irob_ptr + 1)%(_accel->_ind_rob_size);
//      if(init_irob_ptr==-1) init_irob_ptr=_cur_irob_ptr;
//      ind_reorder_entry_t *reorder_entry = new ind_reorder_entry_t();
//
//      assert(_reorder_entry_id.find(_cur_irob_ptr)==_reorder_entry_id.end() && "allocated location should be empty in reorder buffer");
//      _reorder_entry_id.insert(make_pair(_cur_irob_ptr, reorder_entry));
//
//      reorder_entry->id = _cur_irob_ptr;
//
//      _ind_ROB.push(reorder_entry);
//      reorder_entry->stream = &stream;
//      reorder_entry->data_bytes = serve_width; // stream._data_bytes*stream._val_num;
//      reorder_entry->size = 0; // paranoia
//      reorder_entry->last = false;
//
//
//      // just distribute the elements to the queues
//      while ((req>0 || (ind_vp.mem_size() >= ind_vp_slices && stream.stream_active())) && reorder_entry->size < NUM_SCRATCH_BANKS) {
//        wrap_up=false;
//
//        if(req==0) { // should only be done to get the base address
//          ind = ind_vp.peek_out_data(0, stream._index_bytes);
//          addr = stream.cur_addr(ind);
//          // cout << "peeked out addr using indirect port: " << addr << endl;
//
//          bool pop_ind_vp = stream.pop_elem();
//          // cout << "Read new index, pop value: " << pop_ind_vp << " earlier mem size: " << ind_vp.mem_size();
//          if (pop_ind_vp) { // this is for sub-word granularity reads
//            ind_vp.pop_out_data(stream._index_bytes); // pop port_width data
//          }
//          // cout << " new mem size: " << ind_vp.mem_size() << endl;
//        }
//
//        indirect_scr_read_req request;
//        request.data_ptr = reorder_entry->size; // <64
//        request.addr = addr + req*serve_width;
//        assert(request.addr < (SCRATCH_SIZE)*_accel->_ssim->num_active_threads() && "addr crossed the address space of 8 cores");
//        
//        if(init_data_ptr==-1) {
//          init_data_ptr=request.data_ptr;
//          init_addr = addr;
//          // cout << "init data ptr: " << init_data_ptr << " serve_width: " << serve_width << endl;
//        }
//
//        request.bytes = serve_width;
//        request.irob_entry_id = _cur_irob_ptr;
//
//        reorder_entry->size += serve_width;
//
//        // dest_core_id for this local cache line
//        int dest_core_id = stream.get_core_id(request.addr); // request.addr/SCRATCH_SIZE;
//        is_remote = dest_core_id!=(_accel->lsq()->getCpuId()-1);
//        if(_accel->_ssim->num_active_threads()==1) is_remote=false;
//        // cout << " Is request remote: " << is_remote << endl;
//
//        if(is_remote && SS_DEBUG::NET_REQ) {
//            std::cout << "Sending remote scr read request for reorder x loc: " << request.data_ptr << " y loc: " << _cur_irob_ptr << " addr in scratch: " << request.addr << " and bytes to be read: " << request.bytes << endl;
//            std::cout << "Cur core: " << _accel->lsq()->getCpuId() << " cur data ptr: " << init_data_ptr << " irob: " << init_irob_ptr << " addr: " << init_addr << " remote bytes: " << acc_remote_bytes << endl;
//        }
//        if(!is_remote) { // if current request is local
//          wrap_up=true;
//          if(last_remote) { // send the last request
//            assert(init_data_ptr!=-1 && "current local, last remote");
//            _accel->lsq()->push_rem_read_req(last_dest_core, init_data_ptr, init_addr, acc_remote_bytes, init_irob_ptr);
//            // acc_remote_bytes=0;
//            // init_data_ptr=-1; init_irob_ptr=-1;
//          }
//          // for the current one
//          // cout << "Sending local request with x loc: " << request.data_ptr << " bytes: " << request.bytes << " y loc: " << _cur_irob_ptr << endl;
//          _accel->lsq()->push_rem_read_req(dest_core_id, request.data_ptr, request.addr, request.bytes, _cur_irob_ptr);
//          // assert(acc_remote_bytes==0 && "no prev rem bytes should be left");
//          // init_data_ptr=-1; init_irob_ptr=-1;
//        } else {
//          acc_remote_bytes += request.bytes;
//
//          if((last_dest_core!=-1 && last_dest_core!=dest_core_id) || acc_remote_bytes>=7*NUM_SCRATCH_BANKS) { // if core switched
//            assert(init_data_ptr!=-1 && "current remote, reached thresh or switched core");
//            // cout << "serving because it crossed the threshold of requests\n";
//            _accel->lsq()->push_rem_read_req(last_dest_core, init_data_ptr, init_addr, acc_remote_bytes, init_irob_ptr);
//            wrap_up=true;
//            // acc_remote_bytes=0;
//            // init_data_ptr=-1; init_irob_ptr=-1;
//          } else {
//            _accel->_stat_num_spu_req_coalesced++;
//          }
//        }
//        last_remote = is_remote;
//        last_dest_core = dest_core_id;
//        // _accel->lsq()->push_rem_read_req(request.data_ptr, request.addr, request.bytes, _cur_irob_ptr);
//        
//
//        if(wrap_up) {
//          acc_remote_bytes=0; init_data_ptr=-1; 
//          if(reorder_entry->size==NUM_SCRATCH_BANKS) init_irob_ptr=-1;
//        }
//      }
//
//
//      // cout << "Reorder entry size in the end: " << reorder_entry->size << endl;
//      assert(reorder_entry->size > 0 && reorder_entry->size <= NUM_SCRATCH_BANKS && "it should have went in the loop");
//
//      if(req==num_irob_entries_reqd-1) {
//
//        if (!stream.stream_active()) { // vps -> complete state
//        // if (stream.check_set_empty()) { // vps -> complete state
//          reorder_entry->last = true;
//          _accel->process_stream_stats(stream);
//          if (SS_DEBUG::VP_SCORE2) {
//            std::cout << "SOURCE: Indirect SCR->PORT (queue)\n";
//          }
//
//          // the in_vp isn't really full yet because we have to wait until
//          // the conflict-free data arrives
//          for (int in_port : stream.in_ports()) {
//            port_data_t &in_vp = _accel->port_interf().in_port(in_port);
//            in_vp.set_status(port_data_t::STATUS::COMPLETE, LOC::SCR);
//          }
//
//          if (SS_DEBUG::VP_SCORE2) {
//            std::cout << "SOURCE: Indirect SCR->PORT queue)\n";
//          }
//          ind_vp.set_status(port_data_t::STATUS::FREE);
//        }
//      }
//    }
//    // send all pending remote requests (not -1 if pending)
//    if(last_remote && init_data_ptr!=-1) {
//       assert(init_addr!=-1 && acc_remote_bytes!=0 && "outside the loop");
//       assert(init_irob_ptr!=-1 && "irob should have an allocated entry");
//      _accel->lsq()->push_rem_read_req(last_dest_core, init_data_ptr, init_addr, acc_remote_bytes, init_irob_ptr);
//    }
//  }
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

void dma_controller_t::ind_read_req(indirect_stream_t &stream) {
  // port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
  // auto &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
  // if(!stream._is_2d_stream) { 
  //   assert(stream._sn_port==1);
  // }

  // bool first = true;
  // addr_t base_addr = stream.cur_base_addr;
  // addr_t max_addr = 0;

  // vector<int> imap;
  // // bool first_ss_access=true; // used for 2d stream
  
  // if (stream.straddle_bytes()>0) { 
  //   first=false; 
  //   for (int i = 0; i < stream.straddle_bytes(); ++i) {
  //     imap.push_back(i);
  //   }
  //   stream.set_straddle_bytes(0);
  // }
  // else { stream.cur_base_addr = 0; }

  // while (ind_vp.mem_size() >= stream._index_bytes / ind_vp.get_port_width() &&
  //        stream.stream_active() && (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes/subsize_vp.get_port_width()))) {

  //   if(stream._is_2d_stream && stream._first_ss_access) {
  //     uint64_t ind2=0;

  //     // this works when subsize_vp width <= stream index bytes
  //     for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
  //       SBDT ssize = subsize_vp.peek_out_data(i);
  //       // cout << "8 bytes output: " << ssize << endl;
  //       ind2 = ind2 | (ssize << i*8*subsize_vp.get_port_width());
  //       // ind2 = (ind2 << i*8*subsize_vp.get_port_width()) | ssize;
  //     }

  //     stream._sstream_size = ind2; // should be done only in the first size
  //     if(SS_DEBUG::SCR_ACC || SS_DEBUG::MEM_RD) {
  //       cout << "Current ssize: " << ind2 << endl;
  //     }
  //     
  //     // If sub-stream size is 0, then pop the corresponding element from ind
  //     // port, also again set first ss access
  //     if(ind2==0) {
  //       // Oh, this is required to reduce the total num of elements in atomic
  //       // scr
  //       stream._sstream_size = 1; // hack, assume 1 element done so we could pop this element from stream
  //       stream.pop_elem();
  //       // assert(stream.pop_elem() && "Stream already at end, should allow");
  //       ind_vp.pop_out_data(stream._index_bytes);
  //       subsize_vp.pop_out_data(stream._index_bytes);
  //       /*for(int i=0; i<stream._index_bytes/ind_vp.get_port_width(); ++i) {
  //         ind_vp.pop_out_data();
  //       }
  //       for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
  //         subsize_vp.pop_out_data();
  //       }*/
  //       continue;
  //     } else {
  //       // stream._sstream_size = ind2; // should be done only in the first size
  //       // cout << "SIZE PORT NUMBER: " << stream._sn_port << " config bytes: " << stream._index_bytes << endl;
  //       // cout << "UPDATED SSTREAM SIZE TO: " << stream._sstream_size << endl;
  //       stream._first_ss_access=false;
  //     }
  //   }

  //   uint64_t ind=0;

  //   for(int i=0; i<stream._index_bytes/ind_vp.get_port_width(); ++i) {
  //     SBDT temp = ind_vp.peek_out_data(i);
  //     // cout << "8-bit value from ind port: " << temp << endl;
  //     // ind = (ind << i*8*ind_vp.get_port_width()) | temp;
  //     // FIXME:IMP: lower one works for the simple outer product
  //     ind = ind | (temp << i*8*ind_vp.get_port_width());
  //   }
  //   addr_t addr = stream.cur_addr(ind);
  //   // cout << "Current indirect address sent is: " << std::hex << addr << endl;
  //   // addr_t addr = stream.cur_addr(ind_vp.peek_out_data());

  //   if (first) {
  //     first = false;
  //     base_addr = addr & MEM_MASK;
  //     max_addr = base_addr + MEM_WIDTH;
  //   } else { // not first
  //     if (addr >= max_addr || addr < base_addr) {
  //       break;
  //     }
  //   }

  //   int index = addr - base_addr;

  //   int data_bytes = stream._data_bytes;

  //   // if index+i>64, then what? (send 2 cache line requests)
  //   // cout << "Max map in the cache line: " << index+data_bytes;
  //   if(index + data_bytes > 64) { 
  //     cout << "STRADDLING CACHE LINES\n";
  //     stream.set_straddle_bytes(index+data_bytes-64);
  //     stream.cur_base_addr=max_addr; 
  //   }
 
  //   for (int i = 0; i < data_bytes && (index+i < 64); ++i) {
  //     imap.push_back(index + i);
  //   }

  //   bool pop_ind_vp = stream.pop_elem();
  //   if (pop_ind_vp) {
  //     stream._first_ss_access=true;
  //     // need to consider data width of indirect port as well
  //     for(int i=0; i<stream._index_bytes/ind_vp.get_port_width(); ++i) {
  //       ind_vp.pop_out_data();
  //     }
  //     if(stream._is_2d_stream) {
  //       for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
  //         subsize_vp.pop_out_data();
  //       }
  //     }
  //   }

  //   // if(stream._is_2d_stream) stream.print_status();
  // }
  // bool last = stream.check_set_empty();
  // int in_port = stream.first_in_port();
  // port_data_t &in_vp = _accel->port_interf().in_port(in_port);

  // if (last) {
  //   in_vp.set_is_bytes_waiting_final();
  //   _accel->process_stream_stats(stream);
  // }

  // if(imap.size()==0) return; // case when data-dependent sub-stream size was 0

  // _accel->lsq()->sd_transfers[in_port].reserve();

  // in_vp.inc_bytes_waiting(imap.size());

  // if (SS_DEBUG::MEM_REQ) {
  //   _accel->timestamp();
  //   cout << "indirect request for " << std::hex << base_addr << std::dec
  //        << " for " << imap.size() << " needed bytes"
  //        << "\n";
  // }

  // SSMemReqInfoPtr sdInfo = NULL;
  // sdInfo = new SSMemReqInfo(stream.id(), _accel->_accel_index,
  //                           stream.in_ports(), imap, _accel->get_cur_cycle(),
  //                           stream.fill_mode(), );
  //                           // stream.in_ports(), imap, _accel->_ssim->now(), last, stream.fill_mode());

  // // make request
  // _accel->lsq()->pushRequest(stream.minst(), true /*isLoad*/, NULL /*data*/,
  //                           MEM_WIDTH /*cache line*/, base_addr, 0 /*flags*/,
  //                           0 /*res*/, nullptr /*atomic op*/,
  //                           std::vector<bool>() /*byte enable*/, sdInfo);

  // if (_accel->_ssim->in_roi()) {
  //   add_bw(stream.src(), stream.dest(), 1, imap.size());
  //   _accel->_stat_tot_mem_fetched += imap.size();
  //   _accel->_stat_tot_loads += 1;
  // }
  // // imap.clear();

  // _mem_read_reqs++;
}

void dma_controller_t::ind_write_req(indirect_wr_stream_t &stream) {
  // port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
  // port_data_t &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
  // port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
  // if(!stream._is_2d_stream) { 
  //   assert(stream._sn_port==1);
  // }

  // bool first = true;
  // addr_t init_addr = 0;
  // addr_t prev_addr = 0;

  // unsigned bytes_written = 0;

  // std::vector<uint8_t> data;
  // data.resize(MEM_WIDTH);
  // uint8_t *data8 = (uint8_t *)data.data();
  // // uint16_t* data16 = (uint16_t*)data8;
  // uint64_t *data64 = (uint64_t *)data8;

  // // FIXME: The size of the indirect stream should be made configurable!
  // // int stream_size = stream._data_bytes;
  // int stream_size = 8;

  // int index = 0;
  // while (out_vp.mem_size() && ind_vp.mem_size() >= stream._index_bytes && stream.stream_active() && (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes/subsize_vp.get_port_width()))) {

  //   // load the substream data size
  //   if(stream._is_2d_stream && stream._first_ss_access) {
  //     // uint64_t ind2=0;

  //     // this works when subsize_vp width <= stream index bytes
  //     uint64_t ind2 = subsize_vp.peek_out_data(0, stream._index_bytes);
  //     /* for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
  //       SBDT ssize = subsize_vp.peek_out_data(i);
  //       // cout << "8 bytes output: " << ssize << endl;
  //       ind2 = ind2 | (ssize << i*8*subsize_vp.get_port_width());
  //       // ind2 = (ind2 << i*8*subsize_vp.get_port_width()) | ssize;
  //     }*/

  //     stream._sstream_size = ind2; // should be done only in the first size
  //     if(SS_DEBUG::SCR_ACC || SS_DEBUG::MEM_WR) {
  //       cout << "Current ssize: " << ind2 << endl;
  //     }
  //     
  //     // If sub-stream size is 0, then pop the corresponding element from ind
  //     // port, also again set first ss access
  //     if(ind2==0) {
  //       stream._sstream_size = 1; // hack, assume 1 element done so we could pop this element from stream
  //       stream.pop_elem();
  //       ind_vp.pop_out_data(stream._index_bytes);
  //       subsize_vp.pop_out_data(stream._index_bytes);
  //       continue;
  //     } else {
  //       stream._first_ss_access=false;
  //     }
  //   }


  //   uint64_t ind=0;
  //   for(int i=0; i<stream._index_bytes; ++i) {
  //     uint8_t temp = ind_vp.peek_out_data(i);
  //     ind = ind | (temp << i*8);
  //   }
  //   addr_t addr = stream.cur_addr(ind);
  //   // addr_t addr = stream.cur_addr(ind_vp.peek_out_data());
 
  //   if (first) {
  //     first = false;
  //     // base_addr = addr & MEM_MASK;
  //     init_addr = addr;
  //   } else { // not first
  //     if (prev_addr + stream_size !=
  //         addr) { // addr > max_addr || addr < base_addr) {
  //       // cout <<"prev:" << prev_addr << " new:" << addr << "\n";
  //       break;
  //     }
  //   }

  //   SBDT val = out_vp.peek_out_data();

  //   prev_addr = addr;
  //   data64[index++] = val;

  //   out_vp.pop_out_data();
  //   bytes_written += stream._data_bytes; // sizeof(SBDT);

  //   bool pop_ind_vp = stream.pop_elem();
  //   if (pop_ind_vp) {
  //     stream._first_ss_access=true;
  //     ind_vp.pop_out_data(stream._index_bytes);
  //     /*for(int i=0; i<stream._index_bytes; ++i) {
  //       ind_vp.pop_out_data();
  //     }*/
  //     if(stream._is_2d_stream) {
  //       subsize_vp.pop_out_data(stream._index_bytes);
  //       /*for(int i=0; i<stream._index_bytes/subsize_vp.get_port_width(); ++i) {
  //         subsize_vp.pop_out_data();
  //       }*/
  //     }
  //   }
  // }

  // if(bytes_written==0) return;

  // _accel->lsq()->sd_transfers[MEM_WR_STREAM].reserve();

  // SSMemReqInfoPtr sdInfo =
  //     new SSMemReqInfo(stream.id(), _accel->_accel_index, MEM_WR_STREAM);
  
  // // make store request
  // _accel->lsq()->pushRequest(stream.minst(), false /*isLoad*/, data8,
  //                           bytes_written, init_addr, 0 /*flags*/, 0 /*res*/,
  //                           nullptr /*atomic op*/,
  //                           std::vector<bool>() /*byte enable*/, sdInfo);

  // if(SS_DEBUG::MEM_WR) {
  //   cout << "bytes written: " << bytes_written << "addr: " << std::hex <<
  //   init_addr
  //   << std::dec << " first elem: " << *data64 << "\n";
  // }

  // if (_accel->_ssim->in_roi()) {
  //   add_bw(stream.src(), stream.dest(), 1, bytes_written);

  //   _accel->_stat_mem_bytes_wr += bytes_written;
  //   _accel->_stat_tot_stores++;
  // }

  // bool is_empty = stream.check_set_empty();
  // if (is_empty) {
  //   _accel->process_stream_stats(stream);

  //   if (SS_DEBUG::VP_SCORE2) {
  //     cout << "SOURCE: INDIRECT PORT -> DMA\n";
  //   }
  //   out_vp.set_status(port_data_t::STATUS::FREE);
  //   ind_vp.set_status(port_data_t::STATUS::FREE);
  // }
}

// static bool addr_valid_dir(int64_t acc_size, uint64_t addr, uint64_t prev_addr,
//                            uint64_t base_addr, uint64_t max_addr) {
//   if (acc_size >= 0) {
//     return (addr < max_addr) && (addr > prev_addr || prev_addr == SCRATCH_SIZE);
//   } else { // reverse access enabled
//     return (addr >= base_addr) && (addr < prev_addr);
//   }
// }

// serve only when it comes from the requested location, is_banked used only in
// assert
// TODO: add support for mapping schemes here
vector<uint8_t> scratch_read_controller_t::read_scratch(LinearReadStream &stream, bool is_banked) {
  return {};
//  vector<uint8_t> res;
//  addr_t prev_addr = SCRATCH_SIZE;
//  addr_t addr = stream.cur_addr(); // this is scratch addr
//  addr_t base_addr = addr & SCR_MASK;
//  addr_t max_addr = base_addr + SCR_WIDTH;
//
//  if (SS_DEBUG::SCR_BARRIER) {
//    _accel->timestamp();
//    cout << "scr_rd " << hex << addr << " -> " << max_addr << "\n";
//  }
//  int data_width = stream.data_width(); // should be 2 for my case
//
//  std::fill(mask.begin(), mask.end(), 0);
//
//  uint8_t val[8]; // maximum value that data width can be
//  for (int i = 0; i < 8; ++i) {
//    val[i] = 0;
//  }
//
//  // cout << "CAME FROM BANKED PARTITION? " << is_banked
//  //      << " Is the address linear? " << _accel->isLinearSpad(addr)
//  //      << " data_width: " << stream.data_width()
//  //      << " addr is: " << addr << "\n";
//
//  // go while stream and port does not run out
//  while (stream.stream_active() &&
//         // (!_accel->_linear_spad || (is_banked ^ _accel->isLinearSpad(addr))) &&
//         addr_valid_dir(stream.access_size(), addr, prev_addr, base_addr,
//                        max_addr)) {
//    for (int i = 0; i < data_width; ++i) {
//      val[i] = 0;
//    }
//    if (is_banked) {
//      assert(addr + data_width <= SCRATCH_SIZE);
//    } else {
//      if(!(addr + data_width >= SCRATCH_SIZE &&
//             addr + data_width <= SCRATCH_SIZE + LSCRATCH_SIZE)) {
//        // cout << "Addr: " << addr << " data_width: " << data_width << endl;
//      }
//      assert(addr + data_width >= SCRATCH_SIZE &&
//             addr + data_width <= SCRATCH_SIZE + LSCRATCH_SIZE);
//    }
//
//    SBDT check_value=0;
//    _accel->read_scratchpad(&check_value, addr, data_width, stream.id());
//    // check_value = check_value & ((1<<8*data_width)-1);
//    // cout << "COrrected value read from scratchpad at addr: " << addr << " with data_width: " << data_width << " is: " << check_value << endl;
//    _accel->read_scratchpad(&val[0], addr, data_width, stream.id());
//
//
//    // _accel->read_scratchpad(&check_value, scr_addr, request._output_bytes, 0);
//
//    if (SS_DEBUG::SCR_ACC) {
//      // assume 16-bit to debug just for now
//      cout << "scr_addr:" << hex << addr << " read " << (val[1] << 8 | val[0])
//           << " to port " << stream.first_in_port()
//           << " with input port width: " << data_width << "\n";
//    }
//    for (int i = 0; i < data_width; ++i) {
//      data.push_back(val[i]);
//    }
//    prev_addr = addr;
//    for (int i = 0; i < data_width; ++i) {
//      mask[addr - base_addr + i] = 1;
//    }
//
//    addr = stream.pop_addr();
//
//    if (stream.stride_fill() && stream.stride_hit()) {
//      break;
//    }
//  }
//  if (SS_DEBUG::VERIF_SCR) {
//    _accel->scr_rd_verif << hex << setw(8) << setfill('0') << base_addr << " ";
//    for (uint64_t i = base_addr; i < max_addr; ++i) {
//      _accel->scr_rd_verif << setw(2) << setfill('0')
//                           << (unsigned)_accel->scratchpad[i];
//    }
//    _accel->scr_rd_verif << " ";
//    for (unsigned i = 0; i < SCR_WIDTH / data_width; ++i) {
//      _accel->scr_rd_verif << setw(1) << setfill('0') << (unsigned)mask[i];
//    }
//    _accel->scr_rd_verif << "\n";
//  }
//
//  _accel->_stat_scr_bytes_rd += data.size() * data_width;
//  _accel->_stat_scratch_read_bytes += data.size() * data_width;
//
//  if (_accel->_ssim->in_roi()) {
//    add_bw(stream.src(), stream.dest(), 1, data.size() * data_width);
//    _accel->_stat_scratch_reads++;
//  }
//  return res;
}

// Figure out which port is the most needy
// this has linear read streams as well!
float scratch_read_controller_t::calc_min_port_ready(bool is_banked) {
  float min_port_ready = MAX_PORT_READY;
  for (int i = 0; i < _scr_port_streams.size(); ++i) {
    auto &stream = *_scr_port_streams[i];
    auto is = dynamic_cast<IPortStream*>(&stream);
    if (!is) {
      continue;
    }
    if (stream.stream_active() && (is_banked ^ checkLinearSpadStream(stream))) {
      auto &in_vp = _accel->port_interf().in_port(is->pes[0].port);
      int data_width = stream.data_width();
      if (in_vp.can_push_vp(SCR_WIDTH / data_width)) {
        float num_ready = in_vp.instances_ready();
        if (num_ready < min_port_ready) {
          min_port_ready = num_ready;
        }
      }
    }
  }
  return min_port_ready;
}

// considering ind banked read streams
float scratch_read_controller_t::calc_min_ind_port_ready() {
  float min_port_ready = MAX_PORT_READY;
  for (int i = 0; i < _ind_port_streams.size(); ++i) {
    auto &stream = *_ind_port_streams[i];
    port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
    auto is = dynamic_cast<IPortStream*>(&stream);
    if (!is) {
      continue;
    }
    if ((ind_vp.mem_size() && stream.stream_active() && !checkLinearSpadStream(stream)) || indirect_scr_read_requests_active()) {
      auto &in_vp = _accel->port_interf().in_port(is->pes[0].port);
      int data_width = stream.data_width();
      if (in_vp.can_push_vp(SCR_WIDTH / data_width)) {
        float num_ready = in_vp.instances_ready();
        if (num_ready < min_port_ready) {
          min_port_ready = num_ready;
        }
      }
    }
  }
  return min_port_ready;
}

float dma_controller_t::calc_min_port_ready() {
  float min_port_ready = MAX_PORT_READY;
  for (int i = 0; i < _read_streams.size(); ++i) {
    auto &stream = *_read_streams[i];
    auto is = dynamic_cast<IPortStream*>(&stream);
    if (!is) {
      continue;
    }
    if (stream.stream_active()) {
      auto &in_vp = _accel->port_interf().in_port(is->pes[0].port);
      int data_width = stream.data_width();
      if (in_vp.can_push_vp(SCR_WIDTH / data_width)) {
        float num_ready = in_vp.instances_ready();
        if (num_ready < min_port_ready) {
          min_port_ready = num_ready;
        }
      }
    }
  }
  return min_port_ready;
}

// FIXME: test when the input and output port is 1-byte
void network_controller_t::multicast_data(
    remote_port_multicast_stream_t &stream, int message_size) {

  // cout << "Came here for multicast data of size: " << message_size << endl;

  port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
  int remote_in_port = stream._remote_port;
  uint64_t bytes_written = 0;
  uint64_t remote_elem_sent = 0;

  int data_width = out_vp.get_port_width(); // in bytes
  int8_t val[64]; // number of 8-byte elements to send


  // cout << "stream active? " << stream.stream_active() << " out vp mem size: " << out_vp.mem_size() << " message size: " << message_size << endl;

  if (stream.stream_active() && out_vp.mem_size() >= message_size && out_vp.mem_size()) {
    while (stream.stream_active() // enough in dest
           && out_vp.mem_size() &&
           bytes_written < 64) { // enough in source (64-bytes)
      // TODO: peek out and pop later if message buffer size was full?
      
      // POP data_width amount of data
      SBDT data = out_vp.pop_out_data();
      for(int i=0; i<data_width; i++){
        val[i+remote_elem_sent*data_width] = (data >> (i*8)) & 255;
      }

      bytes_written += data_width;
      remote_elem_sent+=1;
      stream._num_elements--;

      if (SS_DEBUG::NET_REQ) {
        timestamp();
        cout << "POPPED b/c port->remote port WRITE: " << out_vp.port() << " "
             << out_vp.mem_size() << " data: " << std::hex << data << "\n";
        printf("After issue: ");
        stream.print_status();
      }
    }
    // cout << "Sending message of size: " << (remote_elem_sent*data_width) << endl;
    // if(stream._out_port==7) { cout << "Core id: " << _accel->get_core_id() << " REMOTE BYTES SENT: " << (remote_elem_sent*data_width) << endl; }
    _accel->lsq()->push_spu_req(stream._out_port, remote_in_port, val, remote_elem_sent*data_width, stream._core_mask);
  }
  // TODO: do we need n/w ctrl bus bandwidth util
  if (_accel->_ssim->in_roi()) {
    add_bw(stream.src(), stream.dest(), 1, bytes_written);
    _accel->_stat_port_multicast += remote_elem_sent;
  }
}

// indirect stream
// TODO: incomplete
void network_controller_t::write_remote_scr(remote_scr_stream_t &stream) {
  /*
  port_data_t& val_vp = _accel->port_interf().out_port(stream._out_port);
  port_data_t& addr_vp = _accel->port_interf().out_port(stream._addr_port);
  int64_t base_addr = stream._remote_scr_base_addr;
  uint64_t bytes_written=0;
  uint64_t remote_elem_sent=0;
  while(stream.stream_active() //enough in dest
            && val_vp.mem_size() && addr_vp.mem_size() && bytes_written<64) {
  //enough in source (64-bytes)
    // peek out and pop later if message buffer size was full?
    SBDT val = val_vp.pop_out_data();
    SBDT index = addr_vp.pop_out_data();

          addr_t final_scr_addr = base_addr + index;
          // TODO:FIXME: decide on addressing bits
          // printf("val: %ld and addr: %ld\n",val,final_scr_addr);
    // Total address bits = log((linear_spad_size*C),2)
    int spad_offset_bits = 0;
          uint64_t scr_offset=0;

    spad_offset_bits = (int)log2(SCRATCH_SIZE+LSCRATCH_SIZE);
          scr_offset = final_scr_addr & (SCRATCH_SIZE+LSCRATCH_SIZE-1);
    // TODO: can i get num_cores info here somehow?
    int cores = 64;
        int dest_core_id = (final_scr_addr >> spad_offset_bits) & (cores-1); //
  last bits? if(SS_DEBUG::NET_REQ){ std::cout << "dest_core_id: " <<
  dest_core_id << "\n"; std::cout << "scr_offset: " << scr_offset << "\n";
        }
    // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset,
  dest_core_id, stream.id());
    // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset,
  dest_core_id, stream.id());

    bytes_written += DATA_WIDTH;
    remote_elem_sent+=1;
    stream._num_elements--;

        if(SS_DEBUG::NET_REQ){
      timestamp(); cout << "POPPED b/c port->remote scr WRITE: " <<
  addr_vp.port() << " " << addr_vp.mem_size() << " and " << val_vp.port() << " "
  << val_vp.mem_size() << "\n";

          printf("After issue: ");
          stream.print_status();
    }
  }
  if(_accel->_ssim->in_roi()) {
    // add_bw(stream.src(), stream.dest(), 1, bytes_written);
    _accel->_stat_remote_scratch_writes+=remote_elem_sent;
  }
  */
}

void network_controller_t::write_direct_remote_scr(
    direct_remote_scr_stream_t &stream) {
  port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
  uint64_t bytes_written = 0;
  uint64_t remote_elem_sent = 0;

  int8_t val[64]; // there will be only 1 base address
  int message_size = 64/stream.data_width(); // num of data_width elements to be written
  if(stream._num_elements<64/stream.data_width()){
    message_size=stream._num_elements;
  }
  if (stream.stream_active() && val_vp.mem_size() >= message_size) {
    addr_t final_scr_addr = stream.cur_addr();
    addr_t base_addr = final_scr_addr & SCR_MASK;
    addr_t max_addr = base_addr + SCR_WIDTH;

    int spad_offset_bits = 0;
    uint64_t scr_offset=0;
    spad_offset_bits = (int)log2(SCRATCH_SIZE+LSCRATCH_SIZE);
    int cores = _accel->_ssim->num_active_threads();
    int dest_core_id = (final_scr_addr >> spad_offset_bits) & (cores-1); // last bits?
    if(SS_DEBUG::NET_REQ){
      std::cout << "dest_core_id: " << dest_core_id << "\n";
      std::cout << "scr_offset: " << scr_offset << "\n";
    }
    while (final_scr_addr < max_addr && stream.stream_active() // enough in dest
           && val_vp.mem_size() &&
           bytes_written < 64) { // enough in source (64-bytes)
      // peek out and pop later if message buffer size was full?
      SBDT data = val_vp.pop_out_data();
      for(int i=0; i<stream.data_width(); i++){
        val[remote_elem_sent*stream.data_width()+i] = (data >> (i*8)) & 255;
      }

      // addr_t final_scr_addr = base_addr + index;
      // printf("val: %ld and addr: %ld\n",val,final_scr_addr);
      // FIXME: check if this is correct
      final_scr_addr = stream.cur_addr();
      bytes_written += stream.data_width();
      remote_elem_sent+=1;
      // stream._num_elements--;

      if (SS_DEBUG::NET_REQ) {
        timestamp();
        cout << "POPPED b/c direct port->remote scr WRITE: " << val_vp.port()
             << " " << val_vp.mem_size() << "\n";
        printf("After issue: ");
        stream.print_status();
      }
    }
    // _accel->lsq()->push_spu_scr_wr_req(stream._scr_type, val, scr_offset, dest_core_id, stream.id());
    // num_bytes = remote_elem_sent*8
    _accel->lsq()->push_spu_scr_wr_req(&val[0], remote_elem_sent*stream.data_width(), 
                                      scr_offset, dest_core_id, stream.id());
  }
  if (_accel->_ssim->in_roi()) {
    add_bw(stream.src(), stream.dest(), 1, bytes_written);
    _accel->_stat_remote_scratch_writes += remote_elem_sent;
  }
}

void network_controller_t::serve_pending_net_req() {
  _accel->lsq()->serve_pending_net_req();
}

void network_controller_t::check_cpu_response_queue() {
  _accel->lsq()->check_cpu_response_queue();
}

void network_controller_t::cycle() {
  if(!_accel->_dfg) return;
  serve_pending_net_req();
  check_cpu_response_queue();
  int i = 0;
  // cycle over all the streams
  for (i = 0; i < _remote_streams.size(); ++i) {
    _which_remote =
        (_which_remote + 1) >= _remote_streams.size() ? 0 : _which_remote + 1;
    base_stream_t *s = _remote_streams[_which_remote];

    // derivative should always come first
    if (auto *sp = dynamic_cast<direct_remote_scr_stream_t *>(s)) {
      auto &stream = *sp;
      if (stream.stream_active()) {
        port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
        // we can send max of message_size*bus_width messages in 1 cycle or till
        // the queue is full (Oh, this bus is also 512-bit bus)
        write_direct_remote_scr(stream);

        bool is_empty = stream.check_set_empty();
        if (is_empty) {
          _accel->process_stream_stats(stream);
          if (SS_DEBUG::VP_SCORE2) {
            cout << "SOURCE: DIRECT PORT->REMOTE SCR\n";
          }
          if (SS_DEBUG::NET_REQ) {
            printf("Direct remote scratchpad write stream empty\n");
          }
          val_vp.freeStream();
          delete_stream(_which_remote, sp);
        }
        break;
      }
    }
    if (auto *sp = dynamic_cast<remote_scr_stream_t *>(s)) {
      auto &stream = *sp;
      if (stream.stream_active()) {
        port_data_t &val_vp = _accel->port_interf().out_port(stream._out_port);
        port_data_t &addr_vp =
            _accel->port_interf().out_port(stream._addr_port);
        // we can send max of message_size*bus_width messages in 1 cycle or till
        // the queue is full (Oh, this bus is also 512-bit bus)
        write_remote_scr(stream);

        bool is_empty = stream.check_set_empty();
        if (is_empty) {
          _accel->process_stream_stats(stream);
          if (SS_DEBUG::VP_SCORE2) {
            cout << "SOURCE: INDIRECT PORT->REMOTE SCR\n";
          }
          if (SS_DEBUG::NET_REQ) {
            printf("Indirect remote scratchpad write stream empty\n");
          }
          val_vp.freeStream();
          addr_vp.freeStream();
          delete_stream(_which_remote, sp);
        }
        break;
      }
    } else if (auto *sp = dynamic_cast<remote_port_multicast_stream_t *>(s)) {
      auto &stream = *sp;
      port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
      int message_size=100000; // infinity
      if (stream.stream_active()) {

        int data_width = out_vp.get_port_width(); // in bytes
        message_size = 64/data_width; // num of data_width elements to be written
        // cout << "Message size original: " << message_size << endl;
        if(stream._num_elements<64/data_width){
          message_size=stream._num_elements;
        }
        // cout << "Message size after comp: " << message_size << " stream elements: " << stream._num_elements <<  endl;
       if(stream.timeout()) {
          message_size = out_vp.mem_size() < message_size ? out_vp.mem_size() : message_size;
        } else if(out_vp.mem_size() < message_size) { // this should be here if no timeout
          stream.inc_wait_cycles();
          continue;
        }
      }
      if (stream.stream_active() && out_vp.mem_size() >= message_size) {

        // we can send max of message_size*bus_wisth messages in 1 cycle or till
        // the queue is full (Oh, this bus is also 512-bit bus)
        multicast_data(stream, message_size);
        
        bool is_empty = stream.check_set_empty();
        if (is_empty) {
          _accel->process_stream_stats(stream);
          if (SS_DEBUG::VP_SCORE2) {
            cout << "SOURCE: PORT->REMOTE PORT\n";
          }
          if (SS_DEBUG::NET_REQ) {
            printf("Multicast stream empty\n");
          }
          out_vp.freeStream();
          delete_stream(_which_remote, sp);
        }
        break;
      }
    }
  }
}

void network_controller_t::finish_cycle() {}

void network_controller_t::print_status() {
  for (auto &i : _remote_streams) {
    if (!i->empty()) {
      i->print_status();
    }
  }
}

bool scratch_read_controller_t::checkLinearSpadStream(indirect_stream_t& stream) {
  if(!_accel->_linear_spad) return false; // could be a remote address
  port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
  addr_t addr = stream.cur_addr(ind_vp.peek_out_data());
  return _accel->isLinearSpad(addr);
}

bool scratch_read_controller_t::checkLinearSpadStream(LinearReadStream& stream) {
  // TODO(@were): fit this
  // addr_t addr = stream.cur_addr(); // this is scratch addr
  return _accel->isLinearSpad(0);
}

void scratch_read_controller_t::cycle(bool &performed_read) {
//  float min_port_ready = -2;
//  float min_ind_port_ready = -2;
//
//  cycle_read_queue();
//  serve_ind_read_banks();
//
//  int i = 0;
//  // FOR BANKED SCRATCHPAD
//  for (i = 0; i < _read_streams.size(); ++i) {
//    _which_rd = (_which_rd + 1) >= _read_streams.size() ? 0 : _which_rd + 1;
//    base_stream_t *s = _read_streams[_which_rd];
//
//    if (auto *sp = dynamic_cast<LinearReadStream *>(s)) {
//      auto &stream = *sp;
//
//      if (stream.stream_active()) {
//        int data_width = stream.data_width();
//        bool skip_check = SS_DEBUG::UNREAL_INPUTS;
//
//        if (min_port_ready == -2) {
//          min_port_ready = calc_min_port_ready(true);
//        }
//        if (!skip_check && min_port_ready >= MAX_PORT_READY)
//          continue;
//
//        auto &first_in_vp =
//            _accel->port_interf().in_port(stream.first_in_port());
//        float num_ready = first_in_vp.instances_ready();
//
//        // if(skip_check || (first_in_vp.can_push_vp(SCR_WIDTH/DATA_WIDTH)
//        if ((skip_check || (first_in_vp.can_push_vp(SCR_WIDTH / data_width) &&
//                           num_ready == min_port_ready)) && !checkLinearSpadStream(stream)) {
//          // vector<SBDT> data = read_scratch(stream, true); // true means it is
//          // banked
//          vector<uint8_t> data = read_scratch(stream, true);
//
//          for (int in_port : stream.in_ports()) {
//            auto &in_vp = _accel->port_interf().in_port(in_port);
//            // data is a vector of bytes
//            in_vp.push_data(data); // just push in the vector
//            /*for(auto d : data) {
//              in_vp.push_data_byte(d);
//            }*/
//
//            if (stream.stride_fill() && stream.stride_hit()) {
//              in_vp.fill(true);
//            }
//          }
//
//          bool is_empty = stream.check_set_empty();
//          if (is_empty) {
//            _accel->process_stream_stats(stream);
//
//            if (SS_DEBUG::VP_SCORE2) {
//              cout << "SOURCE: SCR->PORT\n";
//            }
//            for (int in_port : stream.in_ports()) {
//              auto &in_vp = _accel->port_interf().in_port(in_port);
//              in_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE,
//                               stream.fill_mode());
//            }
//            delete_stream(_which_rd, sp);
//          }
//          break;
//        }
//      }
//
//    } else if (auto *sp = dynamic_cast<indirect_stream_t *>(s)) {
//      auto &stream = *sp;
//
//      if (stream.empty()) { // delete when all ports are free
//        delete_stream(_which_rd, sp);
//        continue;
//      }
//
//      port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
//
//      // bool x1 = (ind_vp.mem_size() >= stream._index_bytes/ind_vp.get_port_width());
//      // bool x2 = (stream.stream_active());
//      // bool x3 = x1 && x2 && (checkLinearSpadStream(stream));
//      // cout << "Reason for ind scratch stall: mem size? " << x1 << 
//      //   " stream active? " << x2 << " is linear spad?: " << x3 << " space in irob? " << (_ind_ROB.size() < _accel->_ind_rob_size) << " num elem: " << stream._num_elements << " total size: " << ind_vp.mem_size() << endl;
//
//      if ((stream.stream_active() && ind_vp.mem_size() >= stream._index_bytes/ind_vp.get_port_width()  && !checkLinearSpadStream(stream)) && _ind_ROB.size() < _accel->_ind_rob_size) {
//        
//        bool skip_check = SS_DEBUG::UNREAL_INPUTS;
//
//        if (min_ind_port_ready == -2) {
//          min_ind_port_ready = calc_min_ind_port_ready();
//        }
//        if (!skip_check && min_ind_port_ready >= MAX_PORT_READY)
//          continue;
//
//        // if(checkLinearSpadStream(stream)) continue;
//        read_scratch_ind(stream, -1 /*scratch*/);
//        /*if (stream.empty()) {
//          delete_stream(_which_rd, sp);
//        }*/
//        break;
//      }
//    }
//    } else if(auto* sp = dynamic_cast<scr_scr_stream_t*>(s)) {
//      auto& stream = *sp;
//      if(stream.stream_active()) {
//        if(is_shared()) { //destination is remote write buf
//          if(get_ssim()->can_push_shs_buf(8,stream._scratch_addr,
//                                          stream._remote_bitmask)) {
//            //get data, push it to all scratchpads
//            vector<SBDT> data = read_scratch(stream);
//
//            for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
//              if(stream._remote_bitmask & b) {
//                auto& buf = get_ssim()->get_acc(i)->scr_w_c()->_buf_shs_write;
//                buf.push_data(stream._scratch_addr, data);
//                buf.set_last_stream_id(stream.id());
//              }
//            }
//
//            bool is_empty = stream.check_set_empty();
//            if(is_empty) {
//              _accel->process_stream_stats(stream);
//              stream._remote_stream->reset(); //free for later
//            }
//    
//            stream._scratch_addr+=data.size()*DATA_WIDTH;
//            break;
//          }
//        } else { //destination is local read buf
//          if(stream.stream_active() && _buf_shs_read.can_push_addr(8,
//                                                    stream._scratch_addr)) {
//            bool last =
//            xfer_stream_buf(stream,_buf_shs_read,stream._scratch_addr);
//            if(last) {
//              stream._remote_stream->reset(); //free for later
//            }
//            break;
//          }
//        }
//      }
//    }
//  }
//  if (i < _read_streams.size()) {
//    // performed_read = false;
//    performed_read = true;
//  } else {
//    // performed_read = true;
//    performed_read = false;
//  }
}

// FIXME: have to do this because of weird atomic scr logic, TODO: combine scratch
// read and write controllers
void scratch_read_controller_t::linear_scratch_cycle() {
  // FOR LINEAR SCRATCHPAD
//  if (_accel->_linear_spad) {
//    float min_port_ready=-2;
//    for (int i = 0; i < _read_streams.size(); ++i) {
//      _which_rd = (_which_rd + 1) >= _read_streams.size() ? 0 : _which_rd + 1;
//      // _which_rd=(_which_rd+1)>=_scr_port_streams.size() ? 0:_which_rd+1;
//      base_stream_t *s = _read_streams[_which_rd];
//
//      if (auto *sp = dynamic_cast<LinearReadStream *>(s)) {
//        auto &stream = *sp;
//
//        if (stream.stream_active()) {
//          bool skip_check = SS_DEBUG::UNREAL_INPUTS;
//
//          if (min_port_ready == -2) {
//            min_port_ready = calc_min_port_ready(false);
//          }
//          if (!skip_check && min_port_ready >= MAX_PORT_READY)
//            continue;
//
//          auto &first_in_vp =
//              _accel->port_interf().in_port(stream.first_in_port());
//          float num_ready = first_in_vp.instances_ready();
//          int data_width = first_in_vp.get_port_width();
//
//          if ((skip_check || (first_in_vp.can_push_vp(SCR_WIDTH / data_width) &&
//                             num_ready == min_port_ready)) && checkLinearSpadStream(stream)) {
//            vector<uint8_t> data = read_scratch(stream, false);
//
//            for (int in_port : stream.in_ports()) {
//              auto &in_vp = _accel->port_interf().in_port(in_port);
//              in_vp.push_data(data);
//              if (stream.stride_fill() && stream.stride_hit()) {
//                in_vp.fill(true);
//              }
//            }
//
//            bool is_empty = stream.check_set_empty();
//            if (is_empty) {
//              _accel->process_stream_stats(stream);
//
//              if (SS_DEBUG::VP_SCORE2) {
//                cout << "SOURCE: LINEAR SCR->PORT\n";
//              }
//              for (int in_port : stream.in_ports()) {
//                auto &in_vp = _accel->port_interf().in_port(in_port);
//                in_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE,
//                                 stream.fill_mode());
//              }
//              delete_stream(_which_rd, sp);
//            }
//            break;
//          }
//        }
//      } else if (auto *sp = dynamic_cast<indirect_stream_t *>(s)) {
//        auto &stream = *sp;
//        port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
//        auto &subsize_vp = _accel->port_interf().out_port(stream._sn_port);
//
//        if (ind_vp.mem_size() && stream.stream_active() && checkLinearSpadStream(stream)
//     && (!stream._is_2d_stream || (stream._is_2d_stream && subsize_vp.mem_size() >= stream._index_bytes))) {
//          // || indirect_scr_read_requests_active()) {
//          // TODO: check the address belongs to the linear spad
//          // if(!checkLinearSpadStream(stream)) continue;
//          // cout << "Detected as a linear scratchpad\n";
//          read_linear_scratch_ind(stream, -1 /*scratch*/);
//
//          bool is_empty = stream.check_set_empty();
//          if (is_empty) {
//            _accel->process_stream_stats(stream);
//
//            if (SS_DEBUG::VP_SCORE2) {
//              cout << "SOURCE: INDIRECT LINEAR SCR->PORT\n";
//            }
//      
//            // free ind port
//            port_data_t &ind_vp = _accel->port_interf().out_port(stream._ind_port);
//            ind_vp.set_status(port_data_t::STATUS::FREE);
//            // free input ports
//            for (int in_port : stream.in_ports()) {
//              auto &in_vp = _accel->port_interf().in_port(in_port);
//              in_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE,
//                               stream.fill_mode());
//            }
//            delete_stream(_which_rd, sp);
//          }
//          /*
//          if (stream.empty()) {
//            delete_stream(_which_rd, sp);
//          }
//          */
//          break;
//        }
//      }
//    }
//  }
}

void scratch_read_controller_t::finish_cycle() {}

void scratch_read_controller_t::print_status() {
  for (auto &i : _read_streams) {
    if (!i->empty()) {
      i->print_status();
    }
  }
}

bool scratch_write_controller_t::crossbar_backpressureOn() {
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (_atomic_scr_issued_requests[i].size() == MAX_BANK_BUFFER_SIZE)
      return true;
  }
  return false;
}

// TODO: Do it in a neater way!
void scratch_write_controller_t::scr_write(addr_t addr,
                                           LinearWriteStream &stream,
                                           port_data_t &out_vp) {
//  int data_width = out_vp.get_port_width();
//  if (data_width == 1) {
//    uint8_t val =
//        out_vp.pop_out_custom_data<uint8_t>(); // this is te SBDT form of data
//    // cout << "data to be written to scratch: " << (unsigned)(val) << " at addr: " << addr << endl;
//    _accel->write_scratchpad(addr, &val, data_width, stream.id());
//  } else if (data_width == 2) {
//    uint16_t val =
//        out_vp.pop_out_custom_data<uint16_t>(); // this is te SBDT form of data
//    _accel->write_scratchpad(addr, &val, data_width, stream.id());
//  } else if (data_width == 4) {
//    uint32_t val =
//        out_vp.pop_out_custom_data<uint32_t>(); // this is te SBDT form of data
//    _accel->write_scratchpad(addr, &val, data_width, stream.id());
//  } else {
//    uint64_t val =
//        out_vp.pop_out_custom_data<uint64_t>(); // this is te SBDT form of data
//    _accel->write_scratchpad(addr, &val, data_width, stream.id());
//  }
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

// Step1: check atomic request queue? 
// Step2: if no conflict and stream, copy the data from queue to the corresponding ports
// (but this new dimension should be associated with the hardware and not anu
void scratch_write_controller_t::serve_atomic_requests(bool &performed_atomic_scr) {
  addr_t scr_addr; 
  int num_bytes=0;
  SBDT bytes_written=0;
  port_data_t &atomic_in = _accel->port_interf().in_port(ATOMIC_ADDR_PORT);
  port_data_t &bytes_prt = _accel->port_interf().in_port(ATOMIC_BYTES_PORT);
  port_data_t &atomic_cgra_in = _accel->port_interf().in_port(_atomic_cgra_addr_port);
  port_data_t &atomic_in_val = _accel->port_interf().in_port(_atomic_cgra_val_port);
  // cout << "Found pending queue size: " << _pending_request_queue.size() << endl;

  // cout << "core: " << _accel->lsq()->getCpuId() << " Memory size in atomic input port: " << atomic_in.mem_size() << " bytes port: " << bytes_prt.mem_size() << endl;
  // TODO: can we push part of bytes also?
  while (atomic_in.mem_size()>=ATOMIC_ADDR_DATA_WIDTH/atomic_in.get_port_width() && bytes_prt.mem_size()>=BYTES_PORT_DATA_WIDTH/bytes_prt.get_port_width() && bytes_written<CGRA_FIFO_LEN*64) {
    num_bytes = bytes_prt.peek_out_data(0, BYTES_PORT_DATA_WIDTH);
    scr_addr = atomic_in.peek_out_data(0, ATOMIC_ADDR_DATA_WIDTH);

    vector<uint8_t> read_value;
    vector<uint8_t> data_value;
    read_value.resize(num_bytes);
    bytes_written += num_bytes;

    bool can_push_addr = atomic_cgra_in.can_push_bytes_vp(num_bytes);
    bool can_push_val = atomic_in_val.can_push_bytes_vp(num_bytes);

    bool check = reorder_buffer[_cur_drob_pop_ptr].second.second;
    // cout << "Checking index i: " << _cur_drob_pop_ptr << " output: " << check << endl;
    int tid = reorder_buffer[_cur_drob_pop_ptr].first;
    if(check) { // push in data value
      auto it = _atom_val_store.find(tid);
      for(int i=0; i<it->second.size(); ++i) {
        data_value.push_back(it->second[i]);
      }
    }

    // cout << "Received addr: " << scr_addr << " and num bytes: " << num_bytes << endl;
    // update check_conflict here
    // FIXME: should push 1 vector lane in a cycle
    // if(!is_conflict(scr_addr, num_bytes) && !_atom_val_store.empty() && (can_push_addr && can_push_val)) {
    if(check && can_push_addr && can_push_val && !is_conflict(scr_addr, num_bytes)) {
      // TODO: pop if repeat times is 0
      reorder_buffer[_cur_drob_pop_ptr].second.first--;
      if(reorder_buffer[_cur_drob_pop_ptr].second.first==0) {
        reorder_buffer[_cur_drob_pop_ptr]=make_pair(-1,make_pair(0,false));
        _cur_drob_pop_ptr = (_cur_drob_pop_ptr+1)%MAX_ATOM_REQ_QUEUE_SIZE;
        // delete in atom val store
        _atom_val_store.erase(tid);
        _drob_size_used--;
      }
      bytes_prt.pop_out_data(BYTES_PORT_DATA_WIDTH);
      atomic_in.pop_out_data(ATOMIC_ADDR_DATA_WIDTH);

      _conflict_detection_queue.push_back(make_pair(scr_addr, num_bytes));
      _accel->read_scratchpad(&read_value[0], scr_addr, num_bytes, 0);


      _accel->_num_atomic_sent+=num_bytes;

      // cout << "Read value size: " << read_value.size() << " and atom size: " << _atom_val_store.front().size() << endl;
      assert(data_value.size()==read_value.size() && "same number of bytes from scratch and update data");

      atomic_cgra_in.push_data(read_value);
      atomic_in_val.push_data(data_value);
      // atomic_in_val.push_data(_atom_val_store.front());
      // _atom_val_store.erase(tid);
      
      LOG(COMP)
        << "Atomic update value being read from scratchpad with first value: "
        << (read_value[0] | read_value[1]<<8) << " OR: "
        << (read_value[0]<<8 | read_value[1]) << " with atomic sent: "
        << _accel->_num_atomic_sent << " at addr: " << scr_addr << " core: "
        << _accel->lsq()->getCpuId();
    } else break;
  }

  bytes_written=0;

  // consume data from DGRA to update in scratch and update conflict queue
  port_data_t &atomic_cgra_out = _accel->port_interf().out_port(_atomic_cgra_out_port);
  int data_bytes = atomic_cgra_out.get_port_width();
  // cout << "Data in cgra out port: " << atomic_out.mem_size() << endl;
  // TODO: should we pop whole vector at once?
  while(atomic_cgra_out.mem_size()>0) {
    // pop out data and write to scratch (how much is written?)
    int scr_addr = _conflict_detection_queue.front().first;
    SBDT correct_val = atomic_cgra_out.pop_out_data();
    _accel->write_scratchpad(scr_addr,
                     &correct_val,
                     data_bytes, 0);

    if(_conflict_detection_queue.front().second==_accel->_assumed_bytes) {
      cout << "Value written is: " << correct_val << " scr addr: " << scr_addr
           << " core: " << _accel->lsq()->getCpuId() << " and bytes: " << data_bytes;
    }
    // cout << "Number of bytes written from atomic update: " << data_bytes << endl;
    // fix the writing scr addr
    _conflict_detection_queue.front().first += data_bytes;
    _conflict_detection_queue.front().second -= data_bytes;
    if(_conflict_detection_queue.front().second==0) {
      // cout << "Popped from conflict detection\n";
      _conflict_detection_queue.pop_front();
    }

    _accel->_stat_tot_updates+=data_bytes;

    if(SS_DEBUG::COMP) {
      cout << "Atomic update value being written back to scratchpad with updates: " << _accel->_stat_tot_updates << "\n";
    }
    bytes_written += data_bytes;
    _accel->_stat_scratch_bank_requests_executed++;
    _accel->_stat_scr_bytes_wr += bytes_written;
    _accel->_stat_scratch_write_bytes += bytes_written;
  }

  if(_accel->_num_atomic_sent==_accel->_stat_tot_updates) {
    assert(_conflict_detection_queue.empty());
  }

  if (_accel->_ssim->in_roi()) {
    // add_bw(stream.src(), stream.dest(), 1, bytes_written);
    add_bw(LOC::PORT, LOC::SCR, 1, bytes_written);
    _accel->_stat_scratch_writes += 1;
    _accel->_stat_cycles_atomic_scr_executed++;
  }
  // to implement the functionality of common scratch controller
  // performed_atomic_scr = true;
}

void scratch_write_controller_t::serve_atomic_requests_local(bool &performed_atomic_scr) {
  SBDT input_val=0;
  addr_t scr_addr; 
  int opcode = 0;
  SBDT inc, bytes_written=0;
  // int logical_banks = NUM_SCRATCH_BANKS / stream._value_bytes;
  for (int i = 0; i < _logical_banks; ++i) {
    if (!_atomic_scr_issued_requests[i].empty()) {
      atomic_scr_op_req request = _atomic_scr_issued_requests[i].front();
      input_val = 0; // reset
      scr_addr = request._scr_addr;
      inc = request._inc;
      opcode = request._opcode;

      // cout << "Initial input val of atomic scr: " << input_val << endl;
      // _accel->read_scratchpad(&input_val+8-request._output_bytes, scr_addr, request._output_bytes, stream.id());
      // FIXME: confirm if I can fix the ID of atomic scr
      // _accel->read_scratchpad(&input_val+8-request._output_bytes, scr_addr, request._output_bytes, 0);
      SBDT check_value=0;
      _accel->read_scratchpad(&check_value, scr_addr, request._output_bytes, 0);
      // cout << "ATOMIC SCR: addr it rd/writes to: " << scr_addr << " the value: " << input_val << " and inc: " << inc << endl;
      // cout << "CHECKED CORRECT VALUE: " << check_value << endl;
      input_val = check_value & ((1<<request._output_bytes*8)-1);

      LOG(COMP) << "ACCEL ID: " <<  _accel->lsq()->getCpuId();
      LOG(COMP)
        << _accel->get_ssim()->CurrentCycle()
        << " REAL EXECUTION, update at scr_addr: " << scr_addr
        << " at bankid: " << i << " with inc value: " << inc
        <<  " and old value: " << input_val;

      switch (opcode) {
        case 0: input_val += inc;
                break;
        case 1: input_val = std::max(input_val, inc);
                break;
        case 2: input_val = std::min(input_val, inc);
                break;
        case 3: input_val = inc; // update
                break;
        default: cout << "Invalid opcode\n";
                 break;
      }

      _accel->_stat_tot_updates++;

      uint16_t correct_val = input_val & ((1<<request._output_bytes*8)-1);

      // cout << "FInal output of atomic scr: " << correct_val << endl;
      /*
      _accel->write_scratchpad(scr_addr,
                               &input_val + 8 - request._output_bytes,
                               request._output_bytes, 0);
      */
                               // request._output_bytes, stream.id());

      _accel->write_scratchpad(scr_addr,
                               &correct_val,
                               request._output_bytes, 0);
      
      check_value=0;
      _accel->read_scratchpad(&check_value, scr_addr, request._output_bytes, 0);
      // cout << "Updated value at the address: " << scr_addr << " is: " << check_value << endl;
      _accel->_stat_scratch_bank_requests_executed++;
      bytes_written += request._value_bytes;
      _accel->_stat_scr_bytes_wr += request._value_bytes;
      _accel->_stat_scratch_write_bytes += request._value_bytes;
      _atomic_scr_issued_requests[i].pop();
    }
  }

  if (_accel->_ssim->in_roi()) {
    // add_bw(stream.src(), stream.dest(), 1, bytes_written);
    add_bw(LOC::PORT, LOC::SCR, 1, bytes_written);
    _accel->_stat_scratch_writes += 1;
    _accel->_stat_cycles_atomic_scr_executed++;
  }
  // to implement the functionality of common scratch controller
  performed_atomic_scr = true;
}

void scratch_write_controller_t::push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc) {
  struct atomic_scr_op_req temp_req;
  temp_req._scr_addr = scr_addr;
  temp_req._inc = inc; 
  temp_req._opcode = opcode;
  temp_req._value_bytes = val_bytes;
  temp_req._output_bytes = out_bytes;
    
  // FIXME: not sure if this is correct!
  int logical_banks = NUM_SCRATCH_BANKS / val_bytes;

  int bank_id = 0;

  // by default is row interleaving for now
  if (_accel->_banked_spad_mapping_strategy &&
      (strcmp(_accel->_banked_spad_mapping_strategy, "COL") == 0)) {
    // bank_id = (scr_addr >> 9) & (logical_banks - 1);
    bank_id = (scr_addr >> 4) & (logical_banks - 1); // log64 = 6 = log2(k)
  } else {
    bank_id = (scr_addr >> 1) & (logical_banks - 1);
  }

  assert(bank_id < logical_banks);
  _atomic_scr_issued_requests[bank_id].push(temp_req);
}

void scratch_write_controller_t::atomic_scratch_update(atomic_scr_stream_t &stream) {
  SBDT loc;
  addr_t scr_addr=0, max_addr=0;
  // int logical_banks = NUM_SCRATCH_BANKS;
  // int bank_id = 0;
  // added to correct bank conflict calculation
  int num_addr_pops = 0;
  int num_val_pops = 0;

  port_data_t &out_addr = _accel->port_interf().out_port(stream._out_port);
  port_data_t &out_val = _accel->port_interf().out_port(stream._val_port);

  // cout << "Came in for atomic update streams with cnt? " << stream._is_update_cnt_port << " and num updates: " << stream._num_updates << "\n";

  // strides left in the stream or requests left in the queue
  // if(stream.stream_active() || atomic_scr_issued_requests_active()) {
  if (stream.stream_active()) {
    // logical_banks = NUM_SCRATCH_BANKS / stream._value_bytes;

    addr_t base_addr = stream._mem_addr; // this is like offset
    int n=0;

    // for the case when num updates is 0
    if(stream._is_update_cnt_port && stream._sstream_left==stream._val_num && stream._num_updates==-1) { // there should be port_id and real_num_updates
      port_data_t &update_cnt = _accel->port_interf().out_port(stream._num_update_port);
      if(update_cnt.mem_size()>0) { // same data-type
        stream._num_updates = update_cnt.pop_out_data();
        stream._val_sstream_left=stream._num_updates;
        // cout << " Num updates this round: " << stream._num_updates << endl;
        assert(stream._num_updates>=0 && stream._num_updates<10000);
        if(stream._num_updates==0) {
          stream._num_updates=-1;
          stream._val_sstream_left=-1;
          stream._num_strides--; // should not consider this stride
          stream._sstream_left=stream._val_num;
        }
      } 
      if(stream._num_updates==-1) return; // can't do anything
    }

    // TODO: coalesce remote requests in this streak (split at the remote end)
    // Can we serve in another chunk?
    if((stream._val_sstream_left<=0 || out_addr.mem_size() >= (stream._addr_bytes/out_addr.get_port_width())) && (stream._sstream_left<=0 || out_val.mem_size() >= (stream._value_bytes/out_val.get_port_width())) && 
        stream._value_bytes*n < 64 && // number of pushes should be max 
        !crossbar_backpressureOn() && stream._val_sstream_left!=-1) { // enough in src and dest
      n++;

      // hopefully it pushes data here
      if (_accel->_ssim->in_roi()) {
        _accel->_stat_cycles_atomic_scr_pushed++;
      }

      // FIXME: this might not be required if in the other dimension
      num_addr_pops = 0;
      num_val_pops = 0;

      if(stream._val_sstream_left>0) {
        loc = out_addr.peek_out_data(0, stream._addr_bytes); // ;output_bytes);
        // cout << "value popped from port: " << loc << " ";
        if (SS_DEBUG::COMP) {
          std::cout << "1 cycle execution : " << std::endl;
          std::cout << "64-bit value at the addr port is: " << loc;
        }
        loc = stream.cur_addr(loc);
        // cout << "cur addr index: " << stream._cur_addr_index << " addr bytes: " << stream._addr_bytes << endl;

        base_addr = stream.cur_offset() * stream._value_bytes; // _output_bytes;
        scr_addr = base_addr + loc;
        max_addr = (scr_addr & SCR_MASK)+SCR_WIDTH;
        // std::cout << "updated loc: " << loc << " and stream op bytes: " << stream._output_bytes << " base_addr: " << base_addr << "\n";
      }

      // TODO: num_value_pops should be less than 64 bytes?
      // Stage 1: push requests in scratch banks
      while (scr_addr <= max_addr && stream._num_strides > 0 &&
             (stream._val_sstream_left<=0 || out_addr.mem_size() >= (stream._addr_bytes/out_addr.get_port_width())) && (stream._sstream_left<=0 || out_val.mem_size()>=stream._value_bytes/out_val.get_port_width()) &&
             num_val_pops < 64/stream._value_bytes && 
             stream._val_sstream_left!=-1) {
             // (stream._val_sstream_left>0 || stream._sstream_left>0)) {
             // num_val_pops < 64) {
        if (SS_DEBUG::COMP) {
          std::cout << "\tupdate at index location: " << loc
                    << " and scr_addr: " << scr_addr
                    << " and scr_size is: " << SCRATCH_SIZE
                    << " input num strides left: " << stream._num_strides
                    << endl;
        }

        /*struct atomic_scr_op_req temp_req;
        SBDT cur_value = out_val.peek_out_data(0, stream._value_bytes);
        if (SS_DEBUG::COMP) {
          cout << "64-bit value popped from out_val port: " << cur_value << endl;
        }
        temp_req._scr_addr = scr_addr;
        temp_req._inc = stream.cur_val(cur_value);
        temp_req._opcode = stream._op_code;
        temp_req._value_bytes = stream._value_bytes;
        temp_req._output_bytes = stream._output_bytes;

        // by default is row interleaving for now
        if (_accel->_banked_spad_mapping_strategy &&
            (strcmp(_accel->_banked_spad_mapping_strategy, "COL") == 0)) {
          bank_id = (scr_addr >> 5) & (logical_banks - 1); // log64 = 6 = log2(k)
        } else {
          bank_id = (scr_addr >> 1) & (logical_banks - 1);
        }

        assert(bank_id < logical_banks);

        int local_core_id = (scr_addr + stream._output_bytes)/SCRATCH_SIZE;
        local_core_id = scr_addr >> 14;
        if(SS_DEBUG::NET_REQ) {
          timestamp();
          cout << " Sending remote atomic update for scratch_addr: "
               << scr_addr << " from own core id: " << _accel->_ssim->get_core_id()
               << " to remote core id: " << local_core_id <<  endl; 
        }
        uint64_t x = temp_req._inc;

        if (SS_DEBUG::COMP) {
          std::cout << "\tvalues input: " << temp_req._inc << "\tbank_id: " << bank_id
                    << "\n";
          std::cout << "stream strides left are: " << stream._num_strides
                    << "\n";
        }*/

       // strides should reduce only when substream is done
        // cout << "strides after inc update: " << stream._num_strides << " and sstream left: " << stream._sstream_left << endl;
   
        // stream.inc_val_index();
        // stream.inc_addr_index();

        // true when all addresses are already popped
        bool pop_val = stream._sstream_left>0; // stream.can_pop_val();
        // true when all values are already popped
        bool pop_addr = stream._val_sstream_left>0; // stream.can_pop_addr();

        uint64_t x=-1;

        // cout << "can pop val: " << pop_val << " can pop addr: " << pop_addr << endl;

        // int local_scr_addr = scr_addr & (SCRATCH_SIZE-1);

        // pop only if index in word is last?
        if (pop_val) {
          SBDT cur_value = out_val.pop_out_data(stream._value_bytes);
          // out_val.peek_out_data(0, stream._value_bytes);
          x = stream.cur_val(cur_value);
          _update_coalesce_vals.push_back(x);
          stream._cur_val_index = 0;
          num_val_pops++;
          // out_val.pop_out_data(stream._value_bytes);
          if (SS_DEBUG::COMP) {
            std::cout << "\tpopped data from val port";
          }
        }
        if (pop_addr) {
          _update_broadcast_dest.push_back(scr_addr);
          stream._cur_addr_index = 0;
          // for bank conflicts calc purposes
          num_addr_pops++;
          out_addr.pop_out_data(stream._addr_bytes);
          if (SS_DEBUG::COMP) {
            std::cout << "\tpopped data from addr port";
          }
        }


        // cout << "Old" << " sstream left: " << stream._sstream_left << " val ssteam: " << stream._val_sstream_left << " value count: " << _update_coalesce_vals.size() << " broadcast vals: " << _update_broadcast_dest.size() << endl;
 
/*
        // and if we popped both? (NOT POSSIBLE?)
        if(stream._val_sstream_left==stream._num_updates) stream.inc_done_val(); // should be allowed when you popped something at least?
        if(stream._sstream_left==stream._val_num) stream.inc_done_addr(); // should be allowed when you popped something at least?
*/

        if(pop_val) stream.inc_done_val();
        if(pop_addr) stream.inc_done_addr();

        bool can_send = (_update_coalesce_vals.size()==stream._val_num) && (_update_broadcast_dest.size()==stream._num_updates);

        if(can_send) stream.update_strides();
        // cout << "Can send? " << can_send << " sstream left: " << stream._sstream_left << " val ssteam: " << stream._val_sstream_left << " value count: " << _update_coalesce_vals.size() << " broadcast vals: " << _update_broadcast_dest.size() << endl;
        // cout << "pop val: " << pop_val << " pop addr: " << pop_addr << " strides: " << stream._num_strides << endl;
        // stream.print_status();

       if(can_send) {
          _accel->_stat_scratch_bank_requests_pushed++;
          _accel->lsq()->push_rem_atom_op_req(x, _update_broadcast_dest, _update_coalesce_vals, stream._op_code, stream._value_bytes, stream._output_bytes);
          _update_broadcast_dest.clear();
          _update_coalesce_vals.clear();
        }

        if (SS_DEBUG::COMP) {
          std::cout << "\n";
        }
        // should be decremented after every computation
        // new loop added for vector ports
        if(stream._val_sstream_left>0 && out_addr.mem_size() >= (stream._addr_bytes/out_addr.get_port_width())) {
        // if (out_addr.mem_size() > 0 && out_val.mem_size() > 0) {
          loc = out_addr.peek_out_data(0, stream._addr_bytes);
          if (SS_DEBUG::COMP) {
            std::cout << "64-bit value at the addr port is: " << loc;
          }
          loc = stream.cur_addr(loc);
          // std::cout << "loc: " << loc << "\n";
          // making offset also configurable (same configuration as the
          // address)
          base_addr = stream.cur_offset() * stream._value_bytes;

          // scr_addr = base_addr + loc*stream._value_bytes;
          scr_addr = base_addr + loc; //  * stream._value_bytes;
          scr_addr = base_addr + loc;
          // max_addr = (scr_addr & SCR_MASK)+SCR_WIDTH;
          max_addr = (scr_addr & stream._value_mask) + stream._value_bytes;
        }
      }
    }
  }
  if (_accel->_ssim->in_roi()) {
    _accel->_num_cycles_issued++;
  }
  // this should be the case, right?
  // break;
}

void scratch_write_controller_t::atomic_scratch_update_local(atomic_scr_stream_t &stream) {
  SBDT loc;
  addr_t scr_addr, max_addr;
  int logical_banks = NUM_SCRATCH_BANKS;
  int bank_id = 0;
  // added to correct bank conflict calculation
  int num_addr_pops = 0;
  int num_val_pops = 0;

  port_data_t &out_addr = _accel->port_interf().out_port(stream._out_port);
  port_data_t &out_val = _accel->port_interf().out_port(stream._val_port);

  // strides left in the stream or requests left in the queue
  // if(stream.stream_active() || atomic_scr_issued_requests_active()) {
  if (stream.stream_active()) {
    logical_banks = NUM_SCRATCH_BANKS / stream._value_bytes;

    addr_t base_addr = stream._mem_addr; // this is like offset
    int n=0;

    if (out_addr.mem_size() > 0 && out_val.mem_size() > 0 &&
        stream._value_bytes*n < 64 && // number of pushes should be max
        !crossbar_backpressureOn()) { // enough in src and dest
      n++;

      // hopefully it pushes data here
      if (_accel->_ssim->in_roi()) {
        _accel->_stat_cycles_atomic_scr_pushed++;
      }

      num_addr_pops = 0;
      num_val_pops = 0;
      loc = out_addr.peek_out_data(0, stream._output_bytes);
      if (SS_DEBUG::COMP) {
        std::cout << "1 cycle execution : " << std::endl;
        std::cout << "64-bit value at the addr port is: " << loc;
      }
      loc = stream.cur_addr(loc);
      //cout << "cur addr index: " << stream._cur_addr_index << " addr bytes: " << stream._addr_bytes << endl;
      //std::cout << "updated loc: " << loc << "\n";

      base_addr = stream.cur_offset() * stream._output_bytes;
      scr_addr = base_addr + loc * stream._output_bytes;
      max_addr = (scr_addr & SCR_MASK)+SCR_WIDTH;


      //cout << "SCR ADDR: " << scr_addr << " BASE ADDR: " << base_addr << "\n";

      // FIXME: check if this correct
      // max_addr = (scr_addr & stream._value_mask) + stream._value_bytes;

      // go while stream and port does not run out
      // number of iterations of this loop cannot be more than 8*64-bits
      // => cannot pop more than 8 values from the port

      // num_value_pops should be less than 64 bytes?
      // Stage 1: push requests in scratch banks
      int remote_update_this_cycle=0;
      while (scr_addr < max_addr && stream._num_strides > 0 &&
             out_addr.mem_size() && out_val.mem_size() &&
             num_val_pops < 64/stream._value_bytes && remote_update_this_cycle<1) {
             // num_val_pops < 64) {
        if (SS_DEBUG::COMP) {
          std::cout << "\tupdate at index location: " << loc
                    << " and scr_addr: " << scr_addr
                    << " and scr_size is: " << SCRATCH_SIZE
                    << " input num strides: " << stream._num_strides;
        }

        // TODO: instead of assert, if this condition holds true, send a remote
        // request (for now let's send a complete packet -- I don't think we
        // want to say about decomposable n/w but we can study that by
        // increasing the network bandwidth)

        // if(scr_addr + stream._output_bytes > SCRATCH_SIZE) { cout << "scratch_addr: " << scr_addr << endl; }
        // assert(scr_addr + stream._output_bytes <= SCRATCH_SIZE);

        struct atomic_scr_op_req temp_req;
        SBDT cur_value = out_val.peek_out_data(0, stream._value_bytes);
        if (SS_DEBUG::COMP) {
          cout << "64-bit value popped from out_val port: " << cur_value << endl;
        }
        temp_req._scr_addr = scr_addr;
        temp_req._inc = stream.cur_val(cur_value);
        temp_req._opcode = stream._op_code;
        temp_req._value_bytes = stream._value_bytes;
        temp_req._output_bytes = stream._output_bytes;

        // by default is row interleaving for now
        if (_accel->_banked_spad_mapping_strategy &&
            (strcmp(_accel->_banked_spad_mapping_strategy, "COL") == 0)) {
          // bank_id = (scr_addr >> 9) & (logical_banks - 1);
          // bank_id = (scr_addr >> 6) & (logical_banks - 1); // log64 = 6 = log2(k)
          bank_id = (scr_addr >> 5) & (logical_banks - 1); // log64 = 6 = log2(k)
        } else {
          bank_id = (scr_addr >> 1) & (logical_banks - 1);
        }

        assert(bank_id < logical_banks);

        int local_core_id = (scr_addr + stream._output_bytes)/SCRATCH_SIZE;
        local_core_id = scr_addr >> 14;
        if(SS_DEBUG::NET_REQ) {
          timestamp();
          cout << " Sending remote atomic update for scratch_addr: "
               << scr_addr << " from own core id: " << _accel->_ssim->get_core_id()
               << " to remote core id: " << local_core_id <<  endl;
        }
        int local_scr_addr = scr_addr;
        uint64_t x = temp_req._inc;
       
        // FIXME: currently it does not send scalar atomic update requests
        // TODO: need the network function to work for non-multicast scalar
        // requests as well (currently it works only for 64-bytes)
        push_atomic_update_req(local_scr_addr, temp_req._opcode,
                                    temp_req._value_bytes, temp_req._output_bytes, x);
        bool rem=false;
        if(rem) ++remote_update_this_cycle;

        _accel->_stat_scratch_bank_requests_pushed++;
        stream._num_strides--;
        // FIXME: this is to prevent garbage value
        assert(stream._num_strides<1000000);
        // assert(stream._num_strides>=0);

        if (SS_DEBUG::COMP) {
          std::cout << "\tvalues input: " << temp_req._inc << "\tbank_id: " << bank_id
                    << "\n";
          std::cout << "stream strides left are: " << stream._num_strides
                    << "\n";
        }

        stream.inc_val_index();
        stream.inc_addr_index();

        // pop only if index in word is last?
        if (stream.can_pop_val()) {
          stream._cur_val_index = 0;
          num_val_pops++;
          out_val.pop_out_data(stream._value_bytes);
          if (SS_DEBUG::COMP) {
            std::cout << "\tpopped data from val port";
          }
        }
        if (stream.can_pop_addr()) {
          stream._cur_addr_index = 0;
          // for bank conflicts calc purposes
          num_addr_pops++;
          out_addr.pop_out_data(stream._addr_bytes);
          if (SS_DEBUG::COMP) {
            std::cout << "\tpopped data from addr port";
          }
        }
        if (SS_DEBUG::COMP) {
          std::cout << "\n";
        }
        // should be decremented after every computation
        // new loop added for vector ports
        if (out_addr.mem_size() > 0 && out_val.mem_size() > 0) {
          loc = out_addr.peek_out_data();
          if (SS_DEBUG::COMP) {
            std::cout << "64-bit value at the addr port is: " << loc;
          }
          loc = stream.cur_addr(loc);
          // std::cout << "loc: " << loc << "\n";
          // making offset also configurable (same configuration as the
          // address)
          base_addr = stream.cur_offset() * stream._value_bytes;

          // scr_addr = base_addr + loc*stream._value_bytes;
          scr_addr = base_addr + loc * stream._value_bytes;
          // max_addr = (scr_addr & SCR_MASK)+SCR_WIDTH;
          max_addr = (scr_addr & stream._value_mask) + stream._value_bytes;
        }
      }
    }
  }
  if (_accel->_ssim->in_roi()) {
    _accel->_num_cycles_issued++;
  }
  // this should be the case, right?
  // break;
}



void scratch_write_controller_t::cycle(bool can_perform_atomic_scr,
                                       bool &performed_atomic_scr) {

  // bool issued_linear_on_banked_scr = false;
  // bool is_linear_addr_banked = false;

   // FOR THE BANKED SCRATCHPAD
  for (unsigned i = 0; i < _write_streams.size(); ++i) {
    _which_wr = (_which_wr + 1) >= _write_streams.size() ? 0 : _which_wr + 1;
    base_stream_t *s = _write_streams[_which_wr];

    if (auto *sp = dynamic_cast<LinearWriteStream *>(s)) { // write from
      (void) sp;
//      auto &stream = *sp;
//
//      if (stream.stream_active()) {
//        port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);
//
//        if (out_vp.mem_size() > 0) {
//          addr_t addr = stream.cur_addr();
//          addr_t base_addr = addr & SCR_MASK;
//          addr_t max_addr = base_addr + SCR_WIDTH;
//
//          // go while stream and port does not run out
//          uint64_t bytes_written = 0;
//          while (!_accel->isLinearSpad(addr) && addr < max_addr &&
//                 stream.stream_active()
//                 && out_vp.mem_size()) { // enough in source
//
//            int data_width = out_vp.get_port_width();
//            assert(((int)addr >= 0) && (addr + data_width <= SCRATCH_SIZE));
//            scr_write(addr, stream, out_vp);
//
//            if (SS_DEBUG::SCR_ACC) {
//              timestamp();
//              cout << "POPPED b/c port->scratch WRITE: " << out_vp.port() << " "
//                   << out_vp.mem_size() << "\n";
//            }
//
//            // _accel->write_scratchpad(addr, &val, data_width, stream.id());
//
//            addr = stream.pop_addr();
//
//            bytes_written += data_width;
//            _accel->_stat_scr_bytes_wr += data_width;
//            _accel->_stat_scratch_write_bytes += data_width;
//          }
//          if (_accel->_ssim->in_roi()) {
//            add_bw(stream.src(), stream.dest(), 1, bytes_written);
//            _accel->_stat_scratch_writes += 1;
//          }
//
//          bool is_empty = stream.check_set_empty();
//          if (is_empty) {
//            _accel->process_stream_stats(stream);
//            if (SS_DEBUG::VP_SCORE2) {
//              cout << "SOURCE: PORT->SCR\n";
//            }
//            // to debug
//            // _accel->print_spad_addr(0,16);
//            out_vp.set_status(port_data_t::STATUS::FREE);
//            delete_stream(_which_wr, sp);
//          }
//          break;
//        }
//      }

    } else if (auto *sp = dynamic_cast<indirect_wr_stream_t *>(s)) {
      auto &stream = *sp;
      if (stream.stream_active()) {
        port_data_t &out_port =
            _accel->port_interf().out_port(stream._out_port);
        port_data_t &ind_port =
            _accel->port_interf().out_port(stream._ind_port);

        if (out_port.mem_size() > 0 && ind_port.mem_size() >= stream._index_bytes) {
          write_scratch_ind(stream);
          if (stream.empty()) {
            delete_stream(_which_wr, sp);
          }
          break;
        }
      }
    } else if (auto *sp = dynamic_cast<atomic_scr_stream_t *>(s)) {
      auto &stream = *sp;
      port_data_t &out_addr = _accel->port_interf().out_port(stream._out_port);
      port_data_t &out_val = _accel->port_interf().out_port(stream._val_port);

      // timestamp();
      // cerr << "own_core_id: " << _accel->_ssim->get_core_id() << " addr mem_size: "
      //      << out_addr.mem_size() << " val mem_size: " << out_val.mem_size()
      //      << " stream_active: " << stream.stream_active()
      //      << " sstream left: " << stream._sstream_left << " val sstream left: " << stream._val_sstream_left 
      //      << " out port: " << stream._out_port << " val port: " << stream._val_port
      //      << " strides left: " << stream._num_strides << endl;


      // Oh the issue is even though the stream is active, and there is no data
      // on the value port but that is not required because num_updates size is
      // going to be 0

      // FIXME: mem_size based on config (this should be greater than
      // addr_bytes/out_addr.data_width
      if (stream.stream_active() &&
          (stream._val_sstream_left <= 0 ||
           out_addr.mem_size() >= (stream._addr_bytes / out_addr.get_port_width()))) {
      // if(stream.stream_active() && (stream._val_sstream_left<=0 || out_addr.mem_size() >= (stream._addr_bytes/out_addr.get_port_width())) && (stream._sstream_left<=0 || out_val.mem_size() >= (stream._value_bytes/out_val.get_port_width()))) {
        // Constraint: This should be same for all active atomic streams
        _logical_banks = NUM_SCRATCH_BANKS / stream._value_bytes;
        if(stream._sstream_left==stream._val_num && stream._num_updates==-1) { // no values left
          assert(stream._is_update_cnt_port);
          port_data_t &update_cnt = _accel->port_interf().out_port(stream._num_update_port);
          if(update_cnt.mem_size()==0) {
            // cout << "Could not issue due to less values in update cnt: " << stream._num_update_port << endl;
            continue;
          }
        }
        if(_accel->_ssim->num_active_threads()==1) {
          atomic_scratch_update_local(stream);
        } else {
          atomic_scratch_update(stream);
        }
        // cout << "current core: " << _accel->_ssim->get_core_id() << " return number of strides: " << stream._num_strides << endl;
        if (stream.check_set_empty()) {
          // cout << "Setting out port: " << stream._out_port << " and val port: " << stream._val_port << " empty\n";
          assert(_update_broadcast_dest.size()==0 && _update_coalesce_vals.size()==0 && "all pending requests should have been served");
          _accel->process_stream_stats(stream);
          if (SS_DEBUG::VP_SCORE2) {
            cout << "SOURCE: PORT->SCR\n";
          }
          out_addr.freeStream();
          out_val.freeStream();
          delete_stream(_which_wr, sp);
        }
        // break;
      } 
      /*if(!(stream.stream_active() || atomic_scr_issued_requests_active())) {
        delete_stream(_which_wr, sp); // if both are done
      }*/
      /*
      if(atomic_scr_issued_requests_active()) {
        if(can_perform_atomic_scr) {
          serve_atomic_requests(stream, performed_atomic_scr);
          break;
        }
      }*/
    } else if (auto *sp = dynamic_cast<remote_core_net_stream_t *>(s)) {
      auto &stream = *sp;
      // int bytes_written=0;
      if (stream.stream_active()) { // should always be true
        port_data_t &addr_port =
            _accel->port_interf().out_port(stream._addr_port);
        port_data_t &val_port =
            _accel->port_interf().out_port(stream._val_port);

        if (addr_port.mem_size() > 0 && val_port.mem_size() > 0) {
          // printf("COmes to check remote core net stream\n");
          write_scratch_remote_ind(stream);
          /*
          if(stream.empty()) {
            delete_stream(_which_wr,sp);
          }
          */
          break;
        }
      }
    }
  }

  if(atomic_scr_issued_requests_active()) {
    if(can_perform_atomic_scr) { 
      if(_accel->_ssim->num_active_threads()>1) {
        serve_atomic_requests(performed_atomic_scr);
      } else {
        serve_atomic_requests_local(performed_atomic_scr);
      }
    }
  }

  // for (int j = 0; j < (int) buffets.size(); ) {
  //   int i = (j + which_buffet) % buffets.size();
  //   auto &elem = buffets[i];
  //   auto &out_vp = _accel->port_interf().out_port(elem.src_port);
  //   std::vector<uint8_t> to_write;
  //   int start = elem.end - (elem.end & 63);
  //   int end = start + 64;
  //   //std::cout << "[" << start << ", " << end << ") afterwriting: " << elem.end - elem.front + to_write.size()
  //   //          << ", buffersize: " << elem.buffer_size << std::endl;
  //   while (start + to_write.size() < end && elem.total > to_write.size() &&
  //          elem.end + to_write.size() - elem.front < elem.buffer_size &&
  //          out_vp.mem_size()) {
  //     //std::cout << elem.src_port << " pops, mem_size: " << out_vp.mem_size() << ", total"
  //     //          << elem.total << std::endl;
  //     auto val = out_vp.pop_out_data();
  //     for (int i = 0; i < out_vp.get_port_width(); ++i) {
  //       to_write.push_back(val & 255);
  //       val >>= 8;
  //     }
  //   }
  //   if (!to_write.empty()) {
  //     _accel->write_scratchpad(elem.end % elem.buffer_size, &to_write[0], to_write.size(), elem.id());
  //     assert(to_write.size() <= elem.total);
  //     elem.total -= to_write.size();
  //     elem.end += to_write.size();
  //     //std::cout << "write: " << to_write.size() << "bytes, [" << elem.front << ", "
  //     //          << elem.end << ") " << out_vp.mem_size() << std::endl;
  //   }
  //   if (!can_perform_atomic_scr) {
  //     continue;
  //   }
  //   auto &shadow_vp = _accel->port_interf().out_port(elem.shadow);
  //   auto &input_vp = _accel->port_interf().in_port(elem.in_ports()[0]);
  //   std::vector<uint8_t> to_read;
  //   start = elem.cur_addr();
  //   start = start - (start & 63);
  //   end = start + 64;
  //   //std::cout << "next: " << elem.cur_addr() << ", line end: " << end << ", buffered end: "
  //   //          << elem.end << ", shadow: " << shadow_vp.mem_size() << std::endl;
  //   while (can_perform_atomic_scr && elem.affine_base_stream_t::stream_active() &&
  //          elem.cur_addr() < end && elem.cur_addr() >= elem.front &&
  //          elem.cur_addr() < elem.end && shadow_vp.mem_size()) {
  //     int append_idx = to_read.size();
  //     //std::cout << "read: " << elem.cur_addr() << " " << shadow_vp.peek_out_data()
  //     //          << "(" << shadow_vp.mem_size() << ")" << std::endl;
  //     to_read.resize(to_read.size() + input_vp.get_port_width());
  //     _accel->read_scratchpad(&to_read[append_idx], elem.cur_addr() % elem.buffer_size,
  //                             input_vp.get_port_width(), elem.id());
  //     elem.pop_addr();
  //     elem.dont_pop_shadow += input_vp.get_port_width();
  //     elem.dont_pop_shadow %= elem.inport_dtype;
  //     //std::cout << "dont pop: " << elem.dont_pop_shadow << " +" << input_vp.get_port_width()
  //     //          << " %" << elem.inport_dtype << std::endl;
  //     if (!elem.dont_pop_shadow) {
  //       elem.front += shadow_vp.pop_out_data(elem.shadow_dtype);
  //     }
  //   }
  //   // TODO: Support stride padding
  //   if (!to_read.empty()) {
  //     input_vp.push_data(to_read);
  //     //std::cout << "in vp" << elem.in_ports()[0] << ": "<< input_vp.mem_size() << std::endl;
  //   }
  //   if (!elem.stream_active()) {
  //     buffets.erase(buffets.begin() + i);
  //     shadow_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE);
  //     out_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE);
  //     input_vp.set_status(port_data_t::STATUS::FREE, LOC::NONE);
  //     //std::cout << "retired" << std::endl;
  //   } else {
  //     ++j;
  //   }
  //   if (!to_write.empty()) {
  //     return;
  //   }
  //   if (!to_read.empty()) {
  //     can_perform_atomic_scr = false;
  //   }
  // }
  // which_buffet++;



  // FOR LINEAR WR STREAMS ON THE LINEAR SCRATCHPAD (issues only if the address
  // belongs to linear spad)
  // if((!issued_linear_on_banked_scr) || (issued_linear_on_banked_scr &&
  // !is_linear_addr_banked)) {
}

void scratch_write_controller_t::linear_scratch_write_cycle() {
  // if(!_accel->_linear_spad) return;
  // if (_accel->_linear_spad) {
  //   // for(unsigned i = 0; i < (_port_scr_streams.size()); ++i) {
  //   for (unsigned i = 0; i < (_write_streams.size()); ++i) {
  //     // _which_linear_wr=(_which_linear_wr+1)>=(_port_scr_streams.size())?0:_which_linear_wr+1;
  //     _which_linear_wr = (_which_linear_wr + 1) >= (_write_streams.size()) ? 0 : _which_linear_wr + 1;

  //     base_stream_t *s = _write_streams[_which_linear_wr];
  //     if (auto *sp = dynamic_cast<LinearWriteStream *>(s)) {

  //       auto &stream = *sp;

  //       if (stream.stream_active()) {
  //         port_data_t &out_vp = _accel->port_interf().out_port(stream._out_port);

  //         if (out_vp.mem_size() > 0) {
  //           addr_t addr = stream.cur_addr();
  //           addr_t base_addr = addr & SCR_MASK;
  //           addr_t max_addr = base_addr + SCR_WIDTH;

  //           // go while stream and port does not run out
  //           uint64_t bytes_written = 0;
  //           while (_accel->isLinearSpad(addr) && addr < max_addr &&
  //                  stream.stream_active()  // enough in dest
  //                  && out_vp.mem_size()) { // enough in source

  //             int data_width = stream.data_width();
  //             // cout << "LINEAR ADDR: " << addr << " is_linear: " << _accel->isLinearSpad(addr) << endl;
  //             assert(((int)addr >= 0) && (addr + data_width >= SCRATCH_SIZE) &&
  //                    (addr + data_width <= SCRATCH_SIZE + LSCRATCH_SIZE));
  //             scr_write(addr, stream, out_vp);

  //             if (SS_DEBUG::SCR_ACC) {
  //               timestamp();
  //               cout << "POPPED b/c port->linear scratch WRITE: "
  //                    << stream.port
  //                    << " with width: " << stream.data_width()
  //                    << " strides left: " << stream.num_strides() << " "
  //                    << out_vp.mem_size() << "\n";
  //             }

  //             addr = stream.pop_addr();

  //             bytes_written += data_width;
  //             _accel->_stat_scr_bytes_wr += data_width;
  //             _accel->_stat_scratch_write_bytes += data_width;
  //           }
  //           if (_accel->_ssim->in_roi()) {
  //             add_bw(stream.src(), stream.dest(), 1, bytes_written);
  //             _accel->_stat_scratch_writes += 1;
  //           }

  //           bool is_empty = stream.check_set_empty();
  //           if (is_empty) {
  //             _accel->process_stream_stats(stream);
  //             if (SS_DEBUG::VP_SCORE2) {
  //               cout << "SOURCE: PORT->SCR\n";
  //             }
  //             // _accel->print_spad_addr(SCRATCH_SIZE, SCRATCH_SIZE+16);
  //             out_vp.set_status(port_data_t::STATUS::FREE);
  //             delete_stream(_which_wr, sp);
  //           }
  //           break;
  //         }
  //       }
  //     } else if (auto *sp = dynamic_cast<const_scr_stream_t *>(s)) {

  //       auto &stream = *sp;

  //       if (stream.stream_active()) {
  //         int const_width = stream._const_width;
  //         // uint64_t total_pushed=0;
  //         addr_t addr = stream._scratch_addr;
  //         addr_t base_addr = addr & SCR_MASK;
  //         addr_t max_addr = base_addr + SCR_WIDTH;
  //         if(!_accel->isLinearSpad(addr)) continue;
  //         // cout << "IDENTIFIED AS LINEAR SCRATCHPAD STREAM\n";

  //         uint64_t bytes_written = 0;
  //         while (addr < max_addr && stream._iters_left > 0 &&
  //                bytes_written < SCR_WIDTH) {
  //           SBDT val = stream._constant;

  //           assert(addr + const_width >= SCRATCH_SIZE);
  //           assert(addr + const_width <= SCRATCH_SIZE+LSCRATCH_SIZE);
  //           _accel->write_scratchpad(addr, &val, sizeof(const_width), stream.id());

  //           bytes_written += const_width;
  //           _accel->_stat_scr_bytes_wr += const_width;
  //           _accel->_stat_scratch_write_bytes += const_width;
  //           stream._iters_left--;
  //           // total_pushed++;
  //           addr += sizeof(const_width);
  //         }
  //         // }
  //         // add_bw(stream.src(), stream.dest(), 1, total_pushed*DATA_WIDTH);

  //         if (_accel->_ssim->in_roi()) {
  //           add_bw(stream.src(), stream.dest(), 1, bytes_written);
  //           _accel->_stat_scratch_writes += 1;
  //         }

  //         bool is_empty = stream.check_set_empty();
  //         if (is_empty) {
  //           _accel->process_stream_stats(stream);
  //           if (SS_DEBUG::VP_SCORE2) {
  //             cout << "SOURCE: CONST->SCR\n";
  //           }
  //           delete_stream(_which_wr, sp);
  //         }
  //         break;
  //       }
  //     }
  //   }
  // }
}

void scratch_write_controller_t::push_remote_wr_req(uint8_t *val, int num_bytes,
                                                    addr_t scr_addr) {
  port_data_t &addr_vp = _accel->port_interf().out_port(NET_ADDR_PORT);
  port_data_t &val_vp = _accel->port_interf().out_port(NET_VAL_PORT);

  // assert(num_bytes % 8 == 0);
  int64_t meta_info =
      num_bytes << 16 | scr_addr; // Unnecessary encoding/decoding
  // addr_vp.push_mem_data(meta_info);
  addr_vp.push_data(meta_info); // this is by-default 8-bytes
  // addr_vp.push_mem_data(addr);
  for (int i = 0; i < num_bytes; i++) {
    // val_vp.push_data_byte(val[i]);
    val_vp.push_data(val_vp.get_byte_vector(val[i],1));
    // cout << "data_byte being pushed to net_in ports: " << val[i] << "\n";
  }
}

void scratch_write_controller_t::finish_cycle() {}

void scratch_write_controller_t::print_status() {
  for (auto &i : _write_streams) {
    if (dynamic_cast<remote_core_net_stream_t*>(i)) continue;
    if (!i->empty()) {
      i->print_status();
    }
  }
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
//    //   bool is_empty = stream.check_set_empty();
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
  for (int io = 0; io < 2; ++io) {
    for (auto &elem : _soft_config.active_ports(io)) {
      auto &port = port_interf().ports(io)[elem];
      if (port.stream && (port.stream->barrier_mask() & mask)) {
        return false;
      }
      if (mask == -1) {
        static const std::string IO_STR[] = {"Output", "Input"};
        if (port.mem_size()) {
          LOG(WAIT) << IO_STR[io] << "Port: " << port.port() << " data buffer not empty!";
          return false;
        }
        if (port.num_ready()) {
          LOG(WAIT) << IO_STR[io] << "Port: " << port.port() << " data entry not empty!";
          return false;
        }
      }
    }
  }
  return true;

  // bool d = done_internal(show, mask);
  
  // // Is it correct? -- this gives can't free if already free error
  // if(d) {
  //   _cleanup_mode = false;
  // }

  // if (show)
  //   return d;

  // if ((mask == 0) && d) {
  //   // nothing left to clean up
  //   _cleanup_mode = false;
  // }

  // if (SS_DEBUG::CHECK_SCR_ALIAS) {
  //   if (mask == 0 && d) { // WAIT ALL is true
  //     for (int i = 0; i < SCRATCH_SIZE; i += 1) {
  //       scratchpad_readers[i] = 0;
  //       scratchpad_writers[i] = 0;
  //     }
  //   }
  // }


  // if (mask == GLOBAL_WAIT && d && (_ssim->num_active_threads()>1)) {
  //   _lsq->set_spu_done(_lsq->getCpuId());
  //   d = _lsq->all_spu_done(_ssim->num_active_threads());
  // // FIXME: Checking network status (not giving correct results)

  //   // Check network as well
  //   if(d) { // if done, check network as well
  //     if(SS_DEBUG::WAIT) cout << "All cores done, checking network\n";
  //     bool x = lsq()->spu_net_done();
  //     if(!x) d = x;
  //     else {
  //       if(SS_DEBUG::WAIT)
  //       cout << "Network done for number of threads: " << _ssim->num_active_threads() << "\n";
  //     }
  //   }

  //   if(d) {
  //     if(SS_DEBUG::WAIT) {
  //       cout << "GLOBAL WAIT RELEASED for core id: " << lsq()->getCpuId() << "\n";
  //       cout << "CORE ID: " << lsq()->getCpuId() << "\n";
  //     }
  //     // cout << "Called for global barrier for core: " << lsq()->getCpuId() << endl;
  //     lsq()->set_spu_global_wait_released(_lsq->getCpuId());
  //     if(_lsq->is_last_spu(_ssim->num_active_threads())) { // last global wait spu
  //       // cout << "IT WAS LAST SPU TO RELEASE GLOBAL BARRIER with active threads: " << _ssim->num_active_threads()  << "\n";
  //       _lsq->reset_all_spu();
  //       _lsq->reset_all_spu_global_wait();
  //     }
  //   }
  // }

  //   if(d) {
  //     if(SS_DEBUG::WAIT) {
  //     // if(1) {
  //       cout << "GLOBAL WAIT RELEASED for core id: " << _lsq->getCpuId() << "\n";
  //       cout << "CORE ID: " << _lsq->getCpuId() << "\n";
  //     }
  //     // cout << "Called for global barrier for core: " << _lsq->getCpuId() << endl;
  //     _lsq->set_spu_global_wait_released(_lsq->getCpuId());
  //     if(_lsq->is_last_spu(_ssim->num_active_threads())) { // last global wait spu
  //       // cout << "IT WAS LAST SPU TO RELEASE GLOBAL BARRIER\n";
  //       _lsq->reset_all_spu();
  //       _lsq->reset_all_spu_global_wait();
  //     }
  //   }
  // }

  // if (SS_DEBUG::WAIT) {
  //   timestamp();
  //   if (d) {
  //     cout << "Done Check at core " << _lsq->getCpuId() << " -- Done (" << mask << ")\n";
  //   } else {
  //     cout << "Done Check " << _lsq->getCpuId() << " -- NOT DONE: (" << mask << ")\n";
  //     // FIXME: doesn't print for global wait
  //     done_internal(true, mask);
  //   }
  // }

  // return d;
}

// checks only stream engines to see if concurrent operations are done
bool accel_t::done_concurrent(bool show, int mask) {
  bool done = true;

  if (!_dma_c.done(show, mask) || !_scr_r_c.done(show, mask) ||
      !_scr_w_c.done(show, mask) || !_net_c.done(show, mask)) {
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

  if (SS_DEBUG::CHECK_SCR_ALIAS) {
    if (mask & WAIT_SCR_RD && done) {
      for (int i = 0; i < SCRATCH_SIZE; i += 1) {
        scratchpad_readers[i] = 0;
      }
    }
    if (mask & WAIT_SCR_WR && done) {
      for (int i = 0; i < SCRATCH_SIZE; i += 1) {
        scratchpad_writers[i] = 0;
      }
    }
  }

  return done;
}

// checks everything to see if it's done  (includes queues)
bool accel_t::done_internal(bool show, int mask) {
  if (!done_concurrent(show, mask)) {
    return false;
  }

  // TODO: FIX  -- this can be optimized!
  // if (_ssconfig->dispatch_inorder() || mask==0 || mask&WAIT_CMP) {
  if (!_cmd_queue.empty()) {
    if (show) {
      cout << "Main Queue Not Empty\n";
    }
    return false;
  }
  //}

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

bool dma_controller_t::indirect_streams_active() {
  return _indirect_streams.size();
}
bool dma_controller_t::indirect_wr_streams_active() {
  return _indirect_wr_streams.size();
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

bool scratch_write_controller_t::port_scr_streams_active() {
  return _port_scr_streams.size() || buffets.size();
}

bool scratch_write_controller_t::atomic_scr_streams_active() {
  return _atomic_scr_streams.size();
}

bool scratch_write_controller_t::atomic_scr_issued_requests_active() {
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (!_atomic_scr_issued_requests[i].empty()) {
      return true;
    }
  }
  if(!_atom_val_store.empty()) return true;
  // if no data in atomic_in
  port_data_t &atomic_addr_prt = _accel->port_interf().in_port(ATOMIC_ADDR_PORT);
  if(atomic_addr_prt.mem_size()) {
    // cout << "Data available in cgra in\n";
    return true;
  }
  // update pending in the dfg
  if(!_conflict_detection_queue.empty()) {
    // cout << "Data available in conflict detection queue\n";
    return true;
  }
  if(_atomic_cgra_out_port!=-1) {
    port_data_t &atomic_cgra_out = _accel->port_interf().in_port(_atomic_cgra_out_port);
    if(atomic_cgra_out.mem_size()) {
      // cout << "Data available in cgra output port\n";
      return true;
    }
  }

  port_data_t &bytes_prt = _accel->port_interf().in_port(ATOMIC_BYTES_PORT);
  if(bytes_prt.mem_size()) return true;
  if(_atomic_cgra_addr_port!=-1) {
    port_data_t &atomic_cgra_in = _accel->port_interf().in_port(_atomic_cgra_addr_port);
    if(atomic_cgra_in.mem_size()) return true;
  }
   
  if(_atomic_cgra_val_port!=-1) {
    port_data_t &atomic_in_val = _accel->port_interf().in_port(_atomic_cgra_val_port);
    if(atomic_in_val.mem_size()) return true;
  }

  return false;
}

bool scratch_read_controller_t::indirect_scr_read_requests_active() {
  for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
    if (!_indirect_scr_read_requests[i].empty()) {
      return true;
    }
  }
  return false;
}

bool scratch_write_controller_t::done(bool show, int mask) {
//  if (mask & WAIT_SCR_WR_DF) { // wait_all is for local streams only
//    if (!release_df_barrier()) {
//      if (show)
//        cout << "PORT -> REMOTE SCR port Not Empty\n";
//      return false;
//    } else {
//      _remote_scr_writes = 0;
//      _df_count = -1;
//    }
//  }
//  if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_WR || mask == STREAM_WAIT) {
//      // if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_WR || mask == GLOBAL_WAIT || mask == STREAM_WAIT) {
//    if (port_scr_streams_active()) {
//      if (show)
//        cout << "PORT -> SCR Stream Not Empty\n";
//      return false;
//    }
//  }
//  if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_WR ||
//      mask & WAIT_SCR_RD || mask & WAIT_SCR_ATOMIC || mask == GLOBAL_WAIT || mask == STREAM_WAIT) {
//    if (atomic_scr_streams_active()) {
//      if (show)
//        cout << "ATOMIC SCR Stream Not Empty\n";
//      return false;
//    }
//    if (atomic_scr_issued_requests_active()) {
//      if (show) {
//        cout << "ATOMIC SCR bank buffers Not Empty: "; // \n";
//        for (int i = 0; i < NUM_SCRATCH_BANKS; ++i) {
//          if (!_atomic_scr_issued_requests[i].empty()) {
//            cout << _atomic_scr_issued_requests[i].size();
//            break;
//          }
//        }
//        cout << "\n";
//      }
//      return false;
//    }
//  }
//
//  if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_WR ||
//      mask & WAIT_SCR_RD || mask & WAIT_SCR_ATOMIC || mask == STREAM_WAIT) {
//    if (atomic_scr_streams_active()) {
//      if (show)
//        cout << "ATOMIC SCR Stream Not Empty\n";
//      return false;
//    }
//  }
//  if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_WR || mask == STREAM_WAIT) {
//    if (port_scr_streams_active()) {
//      if (show)
//        cout << "PORT -> SCR Stream Not Empty\n";
//      return false;
//    }
//  }

  return true;
}

bool scratch_read_controller_t::scr_port_streams_active() {
  return _scr_port_streams.size();
}

bool scratch_read_controller_t::done(bool show, int mask) {
//  if (mask == 0 || mask & WAIT_CMP || mask & WAIT_SCR_RD || mask == STREAM_WAIT) {
//    if (scr_port_streams_active()) {
//      if (show)
//        cout << "SCR -> PORT Streams Not Empty\n";
//      return false;
//    }
//    if (indirect_scr_read_requests_active()) {
//      if (show)
//        cout << "Banked scratchpad queues Not Empty\n";
//      return false;
//    }
//    if(!_ind_ROB.empty()) {
//      if (show)
//        cout << "Indirect ROB still waiting for requests\n";
//      return false;
//    }
//  }
  return true;
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
    for (unsigned i = 0; i < _soft_config.in_ports_active_plus.size(); ++i) {
      int cur_port = _soft_config.in_ports_active_plus[i];
      auto &in_vp = _port_interf.in_port(cur_port);
      if (in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size()) {
        // if (in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size() ||
        // _dfg->is_busy()) {
        if (show) {
          cout << "In VP: " << cur_port << " Not Empty (";
          cout << "in_use: " << in_vp.in_use()
               << (in_vp.completed() ? "(completed)" : "");
          cout << " num_rdy: " << in_vp.num_ready();
          cout << " mem_size: " << in_vp.mem_size();
          cout << ") \n";
        }

        return false;
      }
    }
  }
  if (mask == 0) {
    for (unsigned i = 0; i < _soft_config.out_ports_active_plus.size(); ++i) {
      int cur_port = _soft_config.out_ports_active_plus[i];
      auto &out_vp = _port_interf.out_port(cur_port);
      if (out_vp.in_use() || out_vp.num_ready() || out_vp.mem_size() ||
          out_vp.num_in_flight()) { //  || _dfg->is_busy()) {
        if (show) {
          cout << "Out VP: " << cur_port << " Not Empty (";
          cout << "in_use: " << out_vp.in_use();
          cout << " num_rdy: " << out_vp.num_ready();
          cout << " mem_size: " << out_vp.mem_size();
          cout << " in_flight: " << out_vp.num_in_flight();
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
  // Slice 0: In Ports Activge
  // Slice 1: Out Ports Active
  // 2,3,4: Reserved for delay  (4 bits each)
  // 5+: switch/fu configuration
  // size - num of 64bit slices

  if (debug && (SS_DEBUG::COMMAND || SS_DEBUG::SCR_BARRIER)) {
    timestamp();
    cout << "dsaURE(response): "
         << "0x" << std::hex << addr << " " << std::dec << size << "\n";
    // for(int i = 0; i < size/8; ++i) {
    //  cout << "0x" << std::hex << bits[i] << " ";
    //}
    // cout << "\n";
  }

  assert(_in_config);
  _in_config = false;

  // std::cout << "before reset\n";
  // for (int i = 0; i < NUM_IN_PORTS; ++i) {
  //   auto &cur_in_port = _port_interf.in_port(i);
  //   std::cout << "port status " << i << ": "
  //             << cur_in_port.in_use() << " "
  //             << cur_in_port.mem_size() << " "
  //             << (int) cur_in_port.status() << " "
  //             << (int) cur_in_port.loc() << std::endl;
  // }

  _soft_config.reset(); // resets to no configuration
  _port_interf.reset();


  if (_sched) {
    delete _sched->ssdfg();
    delete _sched;
  }

  {
    std::string basename = ((char*)(bits) + 9);
    SSDfg *dfg = dsa::dfg::Import("sched/" + basename + ".dfg.json");
    _sched = new Schedule(_ssconfig, dfg);
    _sched->LoadMappingInJson("sched/" + basename + ".sched.json");
  }

  _dfg = _sched->ssdfg();                 // now we have the dfg!

  _soft_config.out_ports_lat.resize(64); // make this bigger if we need

  _soft_config.cgra_in_ports_active.resize(128);

  _soft_config.in_ports_active_group.resize(NUM_GROUPS);
  _soft_config.out_ports_active_group.resize(NUM_GROUPS);
  _soft_config.group_thr.resize(NUM_GROUPS);

  _soft_config.in_ports_name.resize(64);
  _soft_config.out_ports_name.resize(64);

  auto &in_vecs = _dfg->vins;
  for (int ind = 0; ind < in_vecs.size(); ++ind) {
    dsa::dfg::VectorPort *vec_in = &in_vecs[ind];
    int i = _sched->vecPortOf(vec_in);
    CHECK(i != -1);
    _soft_config.in_ports_active.push_back(i); // activate input vector port

    // TODO(@were): Remove this!
    auto *vec_input = dynamic_cast<dsa::dfg::InputPort*>(vec_in);
    CHECK(vec_input);

    _soft_config.in_ports_name[i] = vec_input->name();

    _soft_config.in_ports_active_group[vec_input->group_id()].push_back(i);

    // port mapping of 1 vector port - cgra_port_num: vector offset elements
    port_data_t &cur_in_port = _port_interf.in_port(i);
    cur_in_port.set_dfg_vec(vec_in);

    // cout << "accel: " << _lsq->getCpuId() << "setting port width: " << vec_in->get_port_width() << "\n";
    cur_in_port.set_port_width(vec_in->get_port_width());
 
    // for each mapped cgra port
    for (unsigned port_idx = 0; port_idx < cur_in_port.port_cgra_elem();
         ++port_idx) {
      int cgra_port_num = cur_in_port.cgra_port_for_index(port_idx);
      if (cgra_port_num >= _soft_config.cgra_in_ports_active.size()) {
        _soft_config.cgra_in_ports_active.resize(cgra_port_num + 1);
      }
      _soft_config.cgra_in_ports_active[cgra_port_num] = true;
    }
  }

  //SSDfgNode *dfg_node = vec_output->at(port_idx*vec_output->get_port_width()/64 );

  auto &out_vecs = _dfg->vouts;
  for (int ind = 0; ind < out_vecs.size(); ++ind) {
    // cout << "size of output vector: " << out_vecs.size();
    auto *vec_out = &out_vecs[ind];
    // cout << "output vector name: " << vec_out->name();
    int i = _sched->vecPortOf(vec_out);
    // cout << "Done finding index of output vec port which is: " << i << endl;

    _soft_config.out_ports_active.push_back(i);
    auto *vec_output = dynamic_cast<dsa::dfg::OutputPort*>(vec_out);
    assert(vec_output);

    _soft_config.out_ports_name[i] = vec_output->name();
    _soft_config.out_ports_active_group[vec_output->group_id()].push_back(i);

    // port mapping of 1 vector port - cgra_port_num: vector offset elements
    auto &cur_out_port = _port_interf.out_port(i);
    cur_out_port.set_port_width(vec_out->get_port_width());
    cur_out_port.set_dfg_vec(vec_out);

    _soft_config.out_ports_lat[i] = _sched->latOf(vec_output);
  }

  // int lat, lat_mis;
  // _sched->cheapCalcLatency(lat, lat_mis);
  int max_lat_mis = _sched->max_lat_mis();

  for (int g = 0; g < NUM_GROUPS; ++g) {
    auto &active_ports = _soft_config.in_ports_active_group[g];
    auto &active_out_ports = _soft_config.out_ports_active_group[g];

    if (active_ports.size() > 0) {

      int thr = _sched->group_throughput[g]; // _dfg->maxGroupThroughput(g);

      float thr_ratio = 1 / (float)thr;
      float mis_ratio = ((float)_fu_fifo_len) / (_fu_fifo_len + max_lat_mis);

      // setup the rate limiting structures
      if (thr_ratio < mis_ratio) { // group throughput is worse
        _soft_config.group_thr[g] = make_pair(1, thr);
        _cgra_prev_issued_group[g].resize(max(thr - 1, 0), false);
      } else { // group latency is worse
        _soft_config.group_thr[g] =
            make_pair(_fu_fifo_len, _fu_fifo_len + max_lat_mis);
        _cgra_prev_issued_group[g].resize(
            max(_fu_fifo_len + max_lat_mis - 1, 0), false);
        if (max_lat_mis == 0) {
          _cgra_prev_issued_group[g].resize(0, false);
        }
      }

      if (SS_DEBUG::SHOW_CONFIG) {
        cout << "Core: " << lsq()->getCpuId() << endl;
        for (int i = 0; i < active_ports.size(); ++i) {
          int p = active_ports[i];
          cout << "in vp" << p << " ";
        }
        cout << "\n";
        for (int i = 0; i < active_out_ports.size(); ++i) {
          int p = active_out_ports[i];
          cout << "out vp" << p
               << " has latency:" << _soft_config.out_ports_lat[p] << "\n";
        }
      }
    }
  }

  // compute active_plus
  _soft_config.in_ports_active_plus = _soft_config.in_ports_active;
  _soft_config.out_ports_active_plus = _soft_config.out_ports_active;
  // indirect
  for (unsigned i = START_IND_PORTS; i < STOP_IND_PORTS; ++i) {
    _soft_config.in_ports_active_plus.push_back(i);
    _soft_config.out_ports_active_plus.push_back(i);
  }

  // compute the in ports active for backcgra
  _soft_config.in_ports_active_backcgra = _soft_config.in_ports_active;

}

bool scratch_write_controller_t::schedule_buffet(BuffetStream *buffet) {
  buffets.push_back(*buffet);
  return true;
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

  port_data_t &in_addr = _accel->port_interf().in_port(ATOMIC_ADDR_PORT);
  port_data_t &bytes_prt = _accel->port_interf().in_port(ATOMIC_BYTES_PORT);

  // assert(!in_addr.can_push_bytes_vp(num_addr*_atomic_addr_bytes));
  // assert(!bytes_prt.can_push_bytes_vp(num_addr*BYTES_PORT_DATA_WIDTH));

  // push num_addr bytes while these are addresses in int
  for(int i=0; i<num_addr; ++i) {
    in_addr.push_data(in_addr.get_byte_vector(it->second.second[i], ATOMIC_ADDR_DATA_WIDTH));
    bytes_prt.push_data(bytes_prt.get_byte_vector(num_bytes,BYTES_PORT_DATA_WIDTH));
    // it->second.second[i] += num_bytes; // update address
  }
  _pending_request_queue.erase(tid); // should be completed here
  return num_addr;
}

// FIXME: didn't make sure if the data is in there or not...
void scratch_write_controller_t::push_atomic_inc(int tid, std::vector<uint8_t> inc, int repeat_times) {
  /* port_data_t &in_val = _accel->port_interf().in_port(_atomic_cgra_val_port);
  for(int i=0; i<repeat_times; ++i) {
    in_val.push_data(inc);
  }*/
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

  /*
  cout << "Came in with value: " << inc.size() << " and repeat times: " << repeat_times << endl;
  // SPECIAL CASE WHEN TASKS CANNOT BE INTERLEAVED WITH OTHER TASKS
  for(unsigned i=0; i<inc.size(); ++i) {
    _temp_accum.push_back(inc[i]);
  }
  if(repeat_times!=0) { // pushing in the previous vector
    // cout << "Size of temp accum: " << _temp_accum.size() << endl;
    for(int i=0; i<start_addr.size(); ++i) {
      _a
      // _atom_val_store.push(_temp_accum);
    }
    for(int i=0; i<repeat_times; ++i) {
      // _atom_val_store.push(_temp_accum);
    }
    _temp_accum.clear();
  }
  */
}

void scratch_write_controller_t::push_atomic_val() {
  return;
  /*
  // cout << "Size of atom val: " << _atom_val_store.size() << endl;
  port_data_t &in_val = _accel->port_interf().in_port(_atomic_cgra_val_port);
  while(!_atom_val_store.empty() && in_val.can_push_bytes_vp(_atom_val_store.front().size())) {
    in_val.push_data(_atom_val_store.front());
    _atom_val_store.pop();
  }
  */
}

bool scratch_write_controller_t::atomic_addr_full(int bytes) {
  // return false;
  port_data_t &in_addr = _accel->port_interf().in_port(ATOMIC_ADDR_PORT);
  unsigned num_addr = _accel->_ssim->num_active_threads(); // it->second.second.size();

  int already_elem = in_addr.mem_size();
  if(num_addr-already_elem > MAX_ATOM_REQ_QUEUE_SIZE) return true;
  return false;
  // return in_addr.can_push_bytes_vp(num_addr*ATOMIC_ADDR_DATA_WIDTH);
}

bool scratch_write_controller_t::atomic_val_full(int bytes) { // no space in drob?
  // return false;
  return _drob_size_used > MAX_ATOM_REQ_QUEUE_SIZE; 
  return _atom_val_store.size() > MAX_ATOM_REQ_QUEUE_SIZE;
  return false;
  port_data_t &in_val = _accel->port_interf().in_port(_atomic_cgra_val_port);
  return in_val.can_push_bytes_vp(bytes);
}

bool scratch_write_controller_t::pending_request_queue_full() {
  return false;
  return _pending_request_queue.size()==MAX_ATOM_REQ_QUEUE_SIZE;
}

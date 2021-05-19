#ifndef __SOFTSIM_H__
#define __SOFTSIM_H__

#include <time.h>
#include <cstdint>
#include <iostream>
#include "accel.hh"
#include "state.h"
#include "dsa/spec.h"
#include "spec.h"

//Some utilities:
template<typename T, typename S, typename R>
static void rolling_inc(T& which, S max, R reset) {
  which = ((which+1)==max) ? reset : which + 1; //rolling increment
}


class ssim_t
{
  friend class ticker_t;
  friend class scratch_read_controller_t;
  friend class scratch_write_controller_t;
  friend class network_controller_t;
  friend class dma_controller_t;
  friend class port_port_controller_t;

public:

  const int NUM_ACCEL;
  //Simulator Interface
  ssim_t(Minor::LSQ* lsq);

  uint64_t roi_enter_cycle() { return _roi_enter_cycle; }
  /*! \brief The register file of the CGRA status. */
  dsa::sim::ConfigState rf[DSARF::TOTAL_REG];

  /*! \brief The state register of the input vector ports. */
  dsa::sim::IVPState vps[DSA_MAX_PORTS];

  /*! \brief The technical specification of this accelerator system. */
  dsa::Specification spec;

  /*! \brief Buffet entries. */
  std::vector<BuffetEntry> bes;

  /*!
   * \brief Load the config bitstream.
   */
  void LoadBitStream();

  /*!
   * \brief Load memory to port.
   * \param port The port this stream forwards to.
   * \param source 0: DMA, 1: SPAD.
   * \param dim (dim+1)=dimensions of the stream.
   * \param padding 0: No padding; 1: post-stride zero; 2: pre-stride zero;
   *                3: post-stride predication off; 4: pre-stride predication off.
   */
  void LoadMemoryToPort(int port, int source, int dim, int padding);

  /*!
   * \brief Write port stream to memory.
   * \param port The source of the value.
   * \param operation The operation, can be write or near storage atomic operations.
   * \param dst 0: DMA, 1: SPAD
   * \param dim (dim+1)=dimensions of the stream.
   */
  void WritePortToMemory(int port, int operation, int dst, int dim);

  /*!
   * \brief Read indirect memory to port.
   * \param port The destination port.
   * \param source 0: DMA, 1: SPAD
   * \param ind The indirect memory flag.
   * \param lin The linear memory stream flag.
   */
  void IndirectMemoryToPort(int port, int source, int ind, int lin);

  /*!
   * \brief Dump the current cycle info as prefix of debugging.
   */
  int64_t CurrentCycle();

  /*!
   * \brief Instantiate a barrier.
   * \param mask The mask of the barrier to bar.
   */
  void InsertBarrier(uint64_t mask);

  /*!
   * \brief Instantiate a const stream.
   * \param port The destination port.
   * \param dimension The dimension of the stream.
   */
  void ConstStream(int port, int dim);

  /*!
   * \brief Gather all the ports involved in broadcasting.
   * \param port The given destination port.
   *             Even not set broadcasting, this port should be put in.
   */
  std::vector<PortExecState> gatherBroadcastPorts(int port);

  /*!
   * \brief Reset the state registers after instantiating a stream.
   */
  void resetNonStickyState();

  /*!
   * \brief Expose the affiliated DMA LSQ.
   */
  Minor::LSQ *lsq();

  /*! \brief If this port has enough value to pop. */
  bool CanReceive(int port, int dtype);
  /*! \brief A raw data wrapper for the signature above. */
  bool CanReceive(int imm);

  /*!
   * \brief Directly write a value from a port to the destination register.
   * \param port The output port of data source.
   * \param dtype The data type in bytes of the register.
   */
  uint64_t Receive(int port, int dtype);

  /*!
   * \brief Reroute value from output port to in ports.
   */
  void Reroute(int oport, int iport);
  
  // Interface from instructions to streams
  // IF SB_TIMING, these just send the commands to the respective controllers
  // ELSE, they carry out all operations that are possible at that point
  void set_context(uint64_t context, uint64_t offset);
  void add_port(int in_port);
  // void indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
  //   uint64_t num_elem, int repeat, int repeat_str, uint64_t offset_list,
  //   int dtype, uint64_t ind_mult, bool scratch, bool stream, int sstride, int sacc_size, int sn_port, int val_num, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type);
  void indirect_write(int ind_port, int ind_type, int out_port,
    addr_t index_addr, uint64_t num_elem, uint64_t offset_list,
    int dtype, uint64_t ind_mult, bool scratch, bool is_2d_stream, int sstride, int sacc_size, int sn_port, int val_num);

  void atomic_update_hardware_config(int addr_port, int val_port, int out_port);
  void atomic_update_scratchpad(uint64_t offset, uint64_t iters, int addr_port, int inc_port, int value_type, int output_type, int addr_type, int opcode, int val_num, int num_updates, bool is_update_cnt_port, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type);
  void multicast_remote_port(uint64_t num_elem, uint64_t mask, int out_port, int rem_port, bool dest_flag, bool spad_type, int64_t stride, int64_t access_size);

  void push_in_accel_port(int accel_id, int8_t* val, int num_bytes, int in_port);
  void push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc);
  void push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry);
  void push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry);
  void write_remote_banked_scratchpad(uint8_t* val, int num_bytes, uint16_t scr_addr);

  int get_bytes_from_type(int t);

  bool atomic_addr_full(int bytes) {
    return accel_arr[0]->_scr_w_c.atomic_addr_full(bytes);
  }
  bool atomic_val_full(int bytes) {
    return accel_arr[0]->_scr_w_c.atomic_val_full(bytes);
  }
  bool pending_request_queue_full() {
    return accel_arr[0]->_scr_w_c.pending_request_queue_full();
  }


  void insert_pending_request_queue(int tid, std::vector<int> start_addr, int bytes_waiting) {
    accel_arr[0]->insert_pending_request_queue(tid, start_addr, bytes_waiting);
  }
    
  int push_and_update_addr_in_pq(int tid, int num_bytes) {
  
    return accel_arr[0]->push_and_update_addr_in_pq(tid, num_bytes);
  }
  
  void push_atomic_inc(int tag, std::vector<uint8_t> inc, int repeat_times) {
    accel_arr[0]->push_atomic_inc(tag, inc, repeat_times);
  }


  // We integrate Buffet to achieve double-buffering.
  void instantiate_buffet(int repeat, int repeat_str);

  void print_stats();
  uint64_t forward_progress_cycle();
  void forward_progress(uint64_t c) {_global_progress_cycle=c;}
  bool can_add_stream();
  void add_bitmask_stream(base_stream_t* s);
  void add_bitmask_stream(base_stream_t* s, uint64_t context);
  void set_memory_map_config(base_stream_t* s, uint64_t partition_size, uint64_t active_core_bitvector, int mapping_type);
  bool done(bool show, int mask);
  bool is_in_config();

  // do not want to stall the control core
  void insert_df_barrier(int64_t num_scr_wr, bool spad_type);

  void set_in_use() {
    if(!_in_use) {
      LOG(COMMAND) << now() << "SSIM in use";
      LOG(ROI) << now() << "SSIM in use";
    }
    _in_use=true;
  }
  void set_not_in_use() {
    LOG(COMMAND) << now() << "SSIM not in use";
    LOG(ROI) << now() << "SSIM not in use";
    _in_use=false;
  }
  bool in_use() {return _in_use;}

  uint64_t now();

  uint64_t elpased_time_in_roi() {return _elapsed_time_in_roi;}

  bool in_roi() {return _in_roi;}
  void roi_entry(bool enter);

  /* To get prepared to enter ROI. */
  void setup_stat_cycle();

  /* After entering the ROI, updates the status of starting and ending cycle so that
   * we can ignore those "white bubbles". */
  void update_stat_cycle();

  /* If it does not use any SB code, it will do normal ROI statistics.
   * O.W it ignores those "white bubbles". */
  void cleanup_stat_cycle();

  // TODO(@were): Deprecate this!
  void timestamp(); //print timestamp
  void timestamp_index(int i); //print timestamp
  void timestamp_context(); //print timestamp

  void step();
  void cycle_shared_busses();


  Minor::MinorDynInstPtr cur_minst() {return _cur_minst;}

  void set_cur_minst(Minor::MinorDynInstPtr m) {
    assert(m);
    _cur_minst=m;
  }

  void issued_inst() {
    if(in_roi()) {
      _control_core_insts++;
    }
  }

  // due to stream seq mismatch?
  void issued_discarded_inst() {
    if(in_roi()) {
      _control_core_discarded_insts++;
    }
  }

  void wait_inst(uint64_t mask) {
    if(in_roi()) {
      _wait_map[mask]++;
    }
  }
  void wait_config() {
    if(in_roi()) {
      _config_waits++;
    }
  }
  
  // for global barrier case
  void set_num_active_threads(int num_threads) {
    _num_active_threads = num_threads;
  }

  static bool stall_core(uint64_t mask) {
    return (mask >> DBF_DMAStreams & 1) || (mask >> DBF_WriteStreams & 1) ||
           (mask >> DBF_AtomicStreams & 1) || (mask >> DBF_ComputStreams & 1) ||
           (mask >> DBF_AtomicStreams & 1);

  }

  uint64_t roi_cycles() {return _roi_cycles / 500;}
  uint64_t control_core_insts() {return _control_core_insts;}
  /*uint64_t control_core_bubble_insts() {return _control_core_bubble_insts;}*/
  uint64_t control_core_discarded_insts() {return _control_core_discarded_insts;}
  uint64_t config_waits() {return _config_waits;}

  accel_t* get_acc(int i) {
    assert(i>=0 && i<NUM_ACCEL_TOTAL);
    return accel_arr[i];
  }

  accel_t* shared_acc() { return accel_arr[SHARED_SP]; }

  int get_core_id() {
    return _lsq->getCpuId();
  }

  bool debug_pred() {
    if(_req_core_id==-1) return true;
    else return (_req_core_id==get_core_id());
  }

  int num_active_threads() {
    return _num_active_threads;
  }
   
  bool printed_this_before() { return _printed_this_before; }
  void set_printed_this_before(bool f) { _printed_this_before=f; }

private:

  int _req_core_id=-1;
  bool _printed_this_before=false;
  unsigned _which_shr=0;

  Minor::MinorDynInstPtr _cur_minst;
  uint64_t _context_offset=0; //no offset between addresses
  uint64_t _ever_used_bitmask=1; //bitmask if core ever used

  Minor::LSQ* _lsq;

  std::vector<accel_t*> accel_arr;

  int _num_active_threads=1; // -1; // for global barrier

  bool _prev_done = true;

  bool _in_use=false;

  bool debug=true;

  //Statistics Members
  bool _in_roi = false;
  timespec _start_ts, _stop_ts;
  uint64_t _elapsed_time_in_roi=0;
  uint64_t _times_roi_entered=0;
  uint64_t _orig_stat_start_cycle = 0;
  uint64_t _stat_start_cycle = 0;
  uint64_t _roi_enter_cycle = 0;
  uint64_t _stat_stop_cycle = 0;
  uint64_t _roi_cycles=0;

  uint64_t _control_core_insts=0;
  // uint64_t _control_core_bubble_insts=0;
  uint64_t _control_core_discarded_insts=0;

  uint64_t _config_waits=0;
  std::unordered_map<uint64_t,uint64_t> _wait_map;

  uint64_t _global_progress_cycle=0;
};


#endif

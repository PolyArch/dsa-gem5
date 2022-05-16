#include "accel.hh"
#include "ssim.hh"
#include "statistics.h"

namespace dsa {
namespace stat {

Host::Host(ssim_t &parent_) : parent(parent_) {
}

void Host::countHostInst(StaticInstPtr inst, bool discard) {
  if (!roi()) {
    return;
  }
  if (discard) {
    ++insts_discarded;
    return;
  }
  ++insts_issued;
  if (inst->getName().find("ss_") == 0) {
    ++ctrl_instructions;
  }
}

bool Host::roi() {
  return roi_;
}

bool Host::roi(bool value) {
  DSA_CHECK(roi_ != value) << "ROI can only be flipped!";
  roi_ = value;
  gettimeofday(sim_time + value, nullptr);
  sim_cycle[value] = parent.now();
  DSA_LOG(ROI) << (value ? "Enter" : "Exit") << " ROI @" << parent.now();
  return roi_;
}

double Host::timeElapsed() {
  auto f = [](const timeval &tv) {
    return tv.tv_sec * 1e6 + tv.tv_usec;
  };
  return (f(sim_time[0]) - f(sim_time[1])) / 1e6;
}

double Host::cycleElapsed() {
  double elapsed = (sim_cycle[0] - sim_cycle[1]);
  return cyclesImpl(elapsed);
}

double Host::cyclesImpl(int64_t x) {
  return (double) x / parent.lsq()->get_cpu().clockDomain.clockPeriod();
}

const char *Accelerator::BlameStr[] = {
  #define MACRO(x) #x,
  #include "./blame.def"
  #undef MACRO
};

Accelerator::Accelerator(accel_t &parent_) : parent(parent_) {
  memset(blame_count, 0, sizeof(blame_count));
  memset(mem_lat_brkd, 0, sizeof(mem_lat_brkd));
}

void Accelerator::countDataTraffic(int is_input, LOC unit, int delta) {
  if (!roi()) {
    return;
  }
  DSA_LOG(COUNT) << is_input << " " << LOC_NAME[unit] << " + " << delta;
  traffic[is_input][(int) unit].num_requests++;
  traffic[is_input][(int) unit].traffic += delta;
}

bool Accelerator::roi() {
  return parent.get_ssim()->statistics.roi();
}

void Accelerator::blameCycle() {
  if (!roi()) {
    return;
  }
  base_stream_t *sb = nullptr;
  int io_cnt[2] = {0, 0};
  if (blame == Blame::UNKNOWN) {
    blame = parent.get_ssim()->cmd_queue.empty() ? Blame::HOST : Blame::CMD_QUEUE;
    static std::unordered_map<LOC, Blame> blame_map = {
      {LOC::CONST, Blame::CONST_BW},
      {LOC::DMA, Blame::MEMORY_BW},
      {LOC::SCR, Blame::SPAD_BW},
      {LOC::REC_BUS, Blame::REC_WAIT},
    };
    for (int is_input = 0; is_input < 2; ++is_input) {
      for (auto &port : parent.bsw.ports[is_input]) {
        auto &pi = *parent.port(is_input, port.port);
        if (pi.stream) {
          ++io_cnt[is_input];
          int can_pop = pi.bytesBuffered() < pi.vectorLanes() * pi.scalarSizeInBytes();
          if (can_pop && blame_map.count(pi.stream->unit())) {
            auto to_blame = blame_map[pi.stream->unit()];
            if (to_blame < blame) {
              blame = to_blame;
              sb = pi.stream;
            }
          }
          DSA_LOG(BLAME_PORT)
            << "oi"[is_input] << " " << port.vp->name() << ": " << pi.bytesBuffered() << ", "
            << pi.stream->toString() << ", available_at: "
            << (is_input ? parent.input_ports[port.port].buffer[0].available_at : parent.now());
        }
      }
    }
    if (blame == Blame::HOST) {
      if (io_cnt[1] == 0) {
        auto f = [this] () {
          for (auto &elem : parent.bsw.dfg()->nodes) {
            for (auto &value : elem->values) {
              if (!value.fifo.empty()) {
                blame = Blame::DRAIN_PIPE;
                return;
              }
            }
          }
        };
        f();
        if (blame != Blame::DRAIN_PIPE) {
          DSA_LOG(BLAME) << parent.now() << ": No computation!";
        }
      } else {
        DSA_LOG(BLAME) << parent.now() << ": IO not empty!";
      }
    }
  }
  ++blame_count[blame];
  DSA_LOG(BLAME)
    << parent.now() << " " << BlameStr[blame] << ": " << (sb ? sb->toString() : "(null)")
    << ", Active Out: " << io_cnt[0] << ", Active In: " << io_cnt[1];
}

void Accelerator::countMemoryLatency(int64_t request_cycle, int64_t *breakdown) {
  if (roi()) {
    memory_latency += parent.now() - request_cycle;
    for (int i = 0; i < 11; ++i) {
      mem_lat_brkd[i] += (parent.now() - breakdown[i]);
    }
  }
}

int64_t Accelerator::memoryWriteBoundByXfer(bool inc) {
  if (roi() && inc) {
    blame = MEMORY_BW;
    write_unit_bubble++;
  }
  return write_unit_bubble;
}

double Accelerator::averageImpl(int64_t value) {
  double res = (double) value / parent.freq();
  int norm = traffic[true][LOC::DMA].num_requests;
  return res / norm;
}

double Accelerator::averageMemoryLatency() {
  return averageImpl(memory_latency);
}


}
}

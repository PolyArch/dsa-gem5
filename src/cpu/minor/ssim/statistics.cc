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
  CHECK(roi_ != value) << "ROI can only be flipped!";
  roi_ = value;
  gettimeofday(sim_time + value, nullptr);
  sim_cycle[value] = curTick();
  DSA_LOG(ROI) << (value ? "Enter" : "Exit") << " ROI @" << curTick();
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
  return elapsed / parent.lsq()->get_cpu().clockDomain.clockPeriod();
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
  if (blame == Blame::UNKNOWN) {
    blame = Blame::CMD_QUEUE;
    std::unordered_map<LOC, Blame> blame_map = {
      {LOC::CONST, Blame::CONST_BW},
      {LOC::DMA, Blame::MEMORY_BW},
      {LOC::SCR, Blame::SPAD_BW},
      {LOC::REC_BUS, Blame::REC_WAIT},
    };
    int io_cnt[2] = {0, 0};
    for (int is_input = 0; is_input < 2; ++is_input) {
      for (auto &port : parent.bsw.ports[is_input]) {
        auto &pi = parent.port_interf().ports(is_input)[port.port];
        if (pi.stream) {
          ++io_cnt[is_input];
          if (!pi.any_data() && blame_map.count(pi.stream->unit())) {
            auto to_blame = blame_map[pi.stream->unit()];
            blame = std::min(blame, to_blame);
          }
        }
      }
    }
    if (io_cnt[1] == 0 && blame == Blame::CMD_QUEUE) {
      blame = Blame::DRAIN_PIPE;
    }
  }
  ++blame_count[blame];
}

void Accelerator::countMemoryLatency(int64_t request_cycle, int64_t *breakdown) {
  if (roi()) {
    memory_latency += curTick() - request_cycle;
    for (int i = 0; i < 11; ++i) {
      mem_lat_brkd[i] += (curTick() - breakdown[i]);
    }
  }
}

double Accelerator::averageImpl(int64_t value) {
  double freq = parent.lsq()->get_cpu().clockDomain.clockPeriod();
  auto res = (double) value / freq;
  int norm = traffic[true][LOC::DMA].num_requests;
  return res / norm;
}

double Accelerator::averageMemoryLatency() {
  return averageImpl(memory_latency);
}

}
}

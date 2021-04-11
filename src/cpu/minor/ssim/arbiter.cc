#include "stream.hh"
#include "arbiter.h"
#include "accel.hh"

namespace dsa {
namespace sim {

struct BuffetChecker : dsa::sim::stream::Functor {
    /*!
     * \brief To support buffet stream, we need to check if the read is in the bound.
     */
    void Visit(LinearReadStream *lrs) override {
      if (lrs->be) {
        auto addr = lrs->ls->poll(false);
        if (lrs->be->InRange(addr)) {
          ok = true;
          LOG(SCHEDULE)
            << "Load from Buffet: " << lrs->be->toString() << " Request: " << addr;
        } else {
          ok = false;
        }
      }
    }

    /*!
     * \brief To support buffet stream, we need to check if the write is in the buffer.
     */
    void Visit(LinearWriteStream *lws) {
      if (lws->be) {
        auto addr = lws->ls->poll(false);
        if (lws->be->SpaceAvailable()) {
          ok = true;
          LOG(SCHEDULE)
            << "Write to Buffet: " << lws->be->toString() << " Request: " << addr;
        } else {
          ok = false;
        }
      }
    }

    bool ok{true};
};

std::vector<base_stream_t*> RoundRobin::Arbit(soft_config_t &sc, port_interf_t &pi) {
  std::vector<base_stream_t*> res;
  for (int is_input = 0; is_input < 2; ++is_input) {
    for (int loc = 0; loc < LOC::TOTAL; ++loc) {
      auto &ports = sc.active_ports(is_input);
      for (int j = 1; j <= (int) ports.size(); ++j) {
        int idx = (last_executed[is_input][loc] + j) % ports.size();
        int port = ports[idx];
        if (auto stream = pi.ports(is_input)[port].stream) {
          if (!stream->stream_active()) {
            continue;
          }
          if (std::find(res.begin(), res.end(), stream) != res.end()) {
            continue;
          }
          BuffetChecker bc;
          stream->Accept(&bc);
          if (!bc.ok) {
            continue;
          }
          if (stream->side(is_input) == loc) {
            last_executed[is_input][loc] = idx;
            res.push_back(stream);
            LOG(SCHEDULE) << "Execute Stream: " << stream->toString();
            break;
          }
        }
      }
    }
  }
  return res;
}

}
}
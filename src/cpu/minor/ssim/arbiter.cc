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
          DSA_LOG(SCHEDULE)
            << "Load from Buffet: " << lrs->be->toString() << " Request: " << addr;
        } else {
          ok = false;
          DSA_LOG(SCHEDULE)
            << lrs->be << " "
            << addr << " not in range! " << lrs->be->toString();
        }
      }
    }

    /*!
     * \brief To support buffet stream, we need to check if the write is in the buffer.
     */
    void Visit(LinearWriteStream *lws) override {
      if (lws->be) {
        auto addr = lws->ls->poll(false);
        if (lws->be->SpaceAvailable()) {
          ok = true;
          DSA_LOG(SCHEDULE)
            << lws->be << " "
            << "Write to Buffet: " << lws->be->toString() << " Request: " << addr;
        } else {
          ok = false;
          DSA_LOG(SCHEDULE) << "No buffer space to write!";
        }
      }
    }

    bool ok{true};
};

std::vector<base_stream_t*> RoundRobin::Arbit(accel_t *accel) {
  BitstreamWrapper &bsw = accel->bsw;
  std::vector<base_stream_t*> res;
  for (int is_input = 0; is_input < 2; ++is_input) {
    for (int loc = 0; loc < LOC::TOTAL; ++loc) {
      auto &ports = bsw.ports[is_input];
      for (int j = 1; j <= (int) ports.size(); ++j) {
        int idx = (last_executed[is_input][loc] + j) % ports.size();
        int port = ports[idx].port;
        if (auto stream = accel->port(is_input, port)->stream) {
          if (!stream->stream_active()) {
            continue;
          }
          if (std::find(res.begin(), res.end(), stream) != res.end()) {
            continue;
          }
          if (stream->side(is_input) != loc) {
            continue;
          }
          BuffetChecker bc;
          stream->Accept(&bc);
          if (!bc.ok) {
            continue;
          }
          last_executed[is_input][loc] = idx;
          res.push_back(stream);
          DSA_LOG(SCHEDULE)
            << "Execute Stream (" << is_input << ", " << loc << "): " << stream->toString();
          break;
        }
      }
    }
  }
  return res;
}

}
}

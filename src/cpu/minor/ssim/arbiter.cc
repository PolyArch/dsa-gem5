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
  std::vector<std::vector<base_stream_t*>> stream_tables;
  stream_tables.resize(LOC::TOTAL * 2);
  for (int is_input = 0; is_input < 2; ++is_input) {
    const auto &ports = bsw.ports[is_input];
    for (int i = 0; i < (int) ports.size(); ++i) {
      if (auto *stream = accel->port(is_input, ports[i].port)->stream) {
        if (stream->stream_active()) {
          BuffetChecker bc;
          stream->Accept(&bc);
          if (bc.ok) {
            bool allow_rw = (bool) getenv("RW_BUS");
            int key = 0;
            if (allow_rw) {
              key = is_input * LOC::TOTAL + stream->side(is_input);
            } else {
              key = (stream->side(is_input) != LOC::DMA) * is_input * LOC::TOTAL + stream->side(is_input);
            }
            stream_tables[key].push_back(stream);
          }
        }
      }
    }
  }
  std::vector<base_stream_t*> res;
  for (int i = 0; i < stream_tables.size(); ++i) {
    if (!stream_tables[i].empty()) {
      res.push_back(stream_tables[i][last_executed[i] % stream_tables[i].size()]);
      last_executed[i]++;
    }
  }
  std::sort(res.begin(), res.end());
  auto new_end = std::unique(res.begin(), res.end());
  res.resize(new_end - res.begin());
  for (auto elem : res) {
    DSA_LOG(STREAM_SCHEDULE) << accel->now() << ": " << elem->toString();
  }
  return res;
}

}
}

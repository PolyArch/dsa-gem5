#include <vector>
#include <queue>
#include <iostream>

#include "dsa/arch/spad.h"

namespace dsa {
namespace sim {

struct ScratchMemory;
struct Entry;
struct RequestBuffer;
struct Response;

struct Bank {

  ScratchMemory *parent;
  int bankno;
  int fifo_size;
  std::vector<uint8_t> data;
  std::queue<Entry*> task_fifo;
  Entry *read{nullptr};
  Entry *compute{nullptr};
  Entry *write{nullptr};

  Bank(ScratchMemory *parent_, int bankno_, int fifo_size_, int bytes);

  template<typename T>
  struct Statistics {
    T task_idle, read_backlog;
    T read_idle, compute_backlog, atomic_backlog;
    T compute_idle, wb_backlog, decompose_backlog;
    T write_idle;

    Statistics(T a, T b, T c, T d, T e, T f, T g, T h, T i) :
      task_idle(a), read_backlog(b), read_idle(c), compute_backlog(d),
      atomic_backlog(e), compute_idle(f), wb_backlog(g), decompose_backlog(h),
      write_idle(i) {}

    Statistics<double> ToRatio(double total) {
      return {
        task_idle / total, read_backlog / total,
        read_idle / total, compute_backlog / total, atomic_backlog / total,
        compute_idle / total, wb_backlog / total, decompose_backlog / total,
        write_idle / total
      };
    }

    std::vector<T*> Elem() {
      return {
        &task_idle, &read_backlog, &read_idle, &compute_backlog, &atomic_backlog,
        &compute_idle, &wb_backlog, &decompose_backlog, &write_idle
      };
    }
  };
  Statistics<int> stat;

  /* \brief Return true if the task fifo is not full. */
  bool Available();
  /* \brief Forward the simulated pipeline. */
  void IssueRead();
  void AccessData();
  void InsituCompute();
  void WriteBack();
};

/* \brief The dynamic memory simulation. */
struct ScratchMemory : adg::ScratchMemory {
  /*
   * \brief The buffer for the pending requests.
   */
  RequestBuffer *rb;
  /* \brief The data structure for each bank. */
  std::vector<Bank> banks;

  ScratchMemory(int bank_width, int num_banks, int capacity, int fifo_size, RequestBuffer *rb);

  /* \brief Simulate the memory system. */
  Response Step();

};

template<typename T>
inline std::ostream &operator<<(std::ostream &os, const Bank::Statistics<T> &stat) {
  return os << stat.task_idle << ", " << stat.read_backlog
     << ", " << stat.read_idle << ", " << stat.compute_backlog
     << ", " << stat.atomic_backlog << ", " << stat.compute_idle
     << ", " << stat.wb_backlog << ", " << stat.decompose_backlog
     << ", " << stat.write_idle;
}

#define DEFINE_STAT_OVERLOAD(OP)                                            \
  template<typename T>                                                      \
  inline Bank::Statistics<T>                                                \
  operator OP(const Bank::Statistics<T> &a, const Bank::Statistics<T> &b) { \
    return Bank::Statistics<T>(                                             \
      a.task_idle OP b.task_idle,                                           \
      a.read_backlog OP b.read_backlog,                                     \
      a.read_idle OP b.read_idle,                                           \
      a.compute_backlog OP b.compute_backlog,                               \
      a.atomic_backlog OP b.atomic_backlog,                                 \
      a.compute_idle OP b.compute_idle,                                     \
      a.wb_backlog OP b.wb_backlog,                                         \
      a.decompose_backlog OP b.decompose_backlog,                           \
      a.write_idle OP b.write_idle                                          \
    );                                                                      \
  }

DEFINE_STAT_OVERLOAD(+)
DEFINE_STAT_OVERLOAD(*)

}
}
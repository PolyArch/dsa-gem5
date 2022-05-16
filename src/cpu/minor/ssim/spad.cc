#include "dsa/debug.h"
#include "dsa-ext/rf.h"
#include "./spad.h"
#include "./request.h"

namespace dsa {
namespace sim {

ScratchMemory::ScratchMemory(int line_size_, int num_banks_, int capacity_, int fifo_size_, RequestBuffer *rb_) :
  adg::ScratchMemory(line_size_, num_banks_, capacity_), rb(rb_) {
  DSA_CHECK(!rb->parent) << "Already assigned to a indirect memory?";
  rb->parent = this;
  DSA_CHECK(num_bytes % num_banks == 0) << "Memory size " << num_bytes << " is not divisible by #banks " << num_banks;
  DSA_CHECK((num_banks & -num_banks) == num_banks)
    << "#Banks is not a power of 2, " << num_banks << " " << (num_banks & -num_banks);
  banks.reserve(num_banks);
  for (int i = 0; i < num_banks; ++i) {
    banks.emplace_back(this, i, fifo_size_, num_bytes / num_banks);
  }
}

Entry::Entry(ScratchMemory *parent_, const Request &request_) :
  parent(parent_), request(request_), result(parent_->bank_width) {}

Bank::Bank(ScratchMemory *parent_, int bankno_, int fifo_size_, int bytes) :
  parent(parent_), bankno(bankno_), fifo_size(fifo_size_), data(bytes, 0),
  stat(0, 0, 0, 0, 0, 0, 0, 0, 0) {}

bool Bank::Available() {
  return ((int) task_fifo.size()) < fifo_size;
}

void Bank::IssueRead() {
  if (task_fifo.empty()) {
    DSA_LOG(ISSUE) << " [Issue] " << bankno << ": No tasks to issue!";
    ++stat.task_idle;
    return;
  }
  if (read) {
    DSA_LOG(ISSUE) << " [Issue] " << bankno << ": Read unit not available!";
    ++stat.read_backlog;
    return;
  }

  read = task_fifo.front();
  read->status = Entry::Status::Access;
  task_fifo.pop();
}

void Bank::AccessData() {
  if (!read) {
    DSA_LOG(READ) << " [Read] " << bankno << ": No indirect request!";
    ++stat.read_idle;
    return;
  }
  if (compute) {
    DSA_LOG(READ) << " [Read] " << bankno << ": Compute unit is busy!" << compute->request;
    ++stat.compute_backlog;
    return;
  }
  if (read->request.op != MemoryOperation::DMO_Read &&
      (write && write->request.addr == read->request.addr)) {
    DSA_LOG(READ) << " [Read] " << bankno << ": Pending write, block to guarantee the atomicity";
    ++stat.atomic_backlog;
    return;
  }
  DSA_LOG(READ) << " [Read] Execute " << read->request.addr;
  auto addr = read->cacheline();
  DSA_CHECK(addr < data.size())
    << "addr: " << read->request.addr << ", cacheline: "<< read->cacheline()
    << ", size: " << data.size();
  read->result = std::vector<uint8_t>(data.begin() + addr, data.begin() + addr + parent->bank_width);
  (compute = read)->status = Entry::Status::Compute;
  read = nullptr;
}

void Bank::InsituCompute() {

  if (!compute) {
    DSA_LOG(COMPUTE) << " [Compute] " << bankno << ": No compute instance";
    ++stat.compute_idle;
    return;
  }
  if (write) {
    DSA_LOG(COMPUTE) << " [Compute] " << bankno << ": write unit not available";
    ++stat.compute_idle;
    return;
  }

  auto l = data.begin() + compute->cacheline();
  // auto r = l + parent->bank_width;
  if (compute->request.op != MemoryOperation::DMO_Read) {
    auto f = [](int64_t a, int64_t b, MemoryOperation op) -> int64_t {
      switch(op) {
        case MemoryOperation::DMO_Add:
          return a + b;
        case MemoryOperation::DMO_Sub:
          return a - b;
        case MemoryOperation::DMO_Mul:
          return a * b;
        case MemoryOperation::DMO_Max:
          return std::max(a, b);
        case MemoryOperation::DMO_Min:
          return std::min(a, b);
        case MemoryOperation::DMO_Write:
          return b;
        default:
          DSA_CHECK(false) << (int) op << " is not a operation";
      }
      return -1;
    };
    for (int i = 0; i < parent->bank_width; i += compute->request.data_size) {
      for (int j = 1; j < compute->request.data_size; ++j) {
        DSA_CHECK(compute->request.mask[i] == compute->request.mask[i + j])
          << i << ", " << j
          << " Data operation should not be in sub-dtype granularity!";
      }
      std::vector<uint8_t> a(l + i, l + i + compute->request.data_size);
      std::vector<uint8_t> b(compute->request.operand.begin() + i,
                             compute->request.operand.begin() + i + compute->request.data_size);
      int64_t res = 0;
      switch (compute->request.data_size * 8) {
      #define BW_IMPL(bw)                                    \
        case bw:                                             \
          DSA_CHECK(a.size() == bw / 8);                         \
          DSA_CHECK(b.size() == bw / 8);                         \
          res = f(*reinterpret_cast<int##bw##_t*>(a.data()), \
                  *reinterpret_cast<int##bw##_t*>(b.data()), \
                  compute->request.op);                      \
          break;
        BW_IMPL(8)
        BW_IMPL(16)
        BW_IMPL(32)
        BW_IMPL(64)
      #undef BW_IMPL
        default:
          DSA_CHECK(false) << compute->request.data_size * 8 << " is not a power of 2!";
      }
      std::vector<uint8_t> c((uint8_t*)(&res), (uint8_t*)(&res) + 8);
      for (int j = 0; j < compute->request.data_size; ++j) {
        compute->result[i + j] = c[j];
      }
    }
  }
  DSA_CHECK(compute->result.size() == parent->bank_width)
    << compute->result.size() << " == " << parent->bank_width;
  write = compute;
  compute = nullptr;
}

void Bank::WriteBack() {
  if (write) {
    auto addr = write->cacheline();
    DSA_LOG(ADDR)
      << "bank: " << bankno << ", port: " << write->request.port
      << ", addr: " << write->request.addr
      << ", " << write->result.size() << "-byte:"
      << *reinterpret_cast<uint64_t*>(write->result.data());
    DSA_CHECK(addr < data.size()) << addr;
    if (write->request.op != MemoryOperation::DMO_Read) {
      for (int i = 0; i < parent->bank_width; ++i) {
        if (write->request.mask[i]) {
          data[addr + i] = write->result[i];
        }
      }
      DSA_LOG(COMMIT) << " [Commit] " << bankno << ": Commit " << write->request;
    }
    write->status = Entry::Status::Commit;
    write = nullptr;
    return;
  }
  ++stat.write_idle;
}

Response ScratchMemory::Step() {
  auto &memory = *this;
  memory.rb->PushRequests();
  for (int j = 0; j < (int) memory.banks.size(); ++j) {
    memory.banks[j].WriteBack();
  }
  for (int j = 0; j < (int) memory.banks.size(); ++j) {
    memory.banks[j].AccessData();
  }
  for (int j = 0; j < (int) memory.banks.size(); ++j) {
    memory.banks[j].InsituCompute();
  }
  for (int j = 0; j < (int) memory.banks.size(); ++j) {
    memory.banks[j].IssueRead();
  }
  return memory.rb->Commit();
}

}
}

#include "dsa/debug.h"
#include "./spad.h"
#include "./request.h"

namespace dsa {
namespace sim {

ScratchMemory::ScratchMemory(int line_size_, int num_banks_, int capacity_, int fifo_size_, RequestBuffer *rb_) :
  adg::ScratchMemory(line_size_, num_banks_, capacity_), rb(rb_) {
  CHECK(!rb->parent) << "Already assigned to a indirect memory?";
  rb->parent = this;
  CHECK(num_bytes % num_banks == 0) << "Memory size " << num_bytes << " is not divisible by #banks " << num_banks;
  CHECK((num_banks & -num_banks) == num_banks) << "#Banks is not a power of 2, " << num_banks;
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
    LOG(ISSUE) << " [Issue] " << bankno << ": No tasks to issue!";
    ++stat.task_idle;
    return;
  }
  if (read) {
    LOG(ISSUE) << " [Issue] " << bankno << ": Read unit not available!";
    ++stat.read_backlog;
    return;
  }

  read = task_fifo.front();
  read->status = Entry::Status::Access;
  task_fifo.pop();
}

void Bank::AccessData() {
  if (!read) {
    LOG(READ) << " [Read] " << bankno << ": No indirect request!";
    ++stat.read_idle;
    return;
  }
  if (compute) {
    LOG(READ) << " [Read] " << bankno << ": Compute unit is busy!" << compute->request;
    ++stat.compute_backlog;
    return;
  }
  if (read->request.op != Request::Operation::read &&
      (write && write->request.addr == read->request.addr)) {
    LOG(READ) << " [Read] " << bankno << ": Pending write, block to guarantee the atomicity";
    ++stat.atomic_backlog;
    return;
  }
  LOG(READ) << " [Read] Execute " << read->request.addr;
  auto addr = read->cacheline();
  CHECK(addr < data.size()) << read->cacheline();
  read->result = std::vector<uint8_t>(data.begin() + addr, data.begin() + addr + parent->bank_width);
  (compute = read)->status = Entry::Status::Compute;
  read = nullptr;
}

void Bank::InsituCompute() {
  if (!compute) {
    LOG(COMPUTE) << " [Compute] " << bankno << ": No compute instance";
    ++stat.compute_idle;
    return;
  }
  if (write) {
    LOG(COMPUTE) << " [Compute] " << bankno << ": write unit not available";
    ++stat.compute_idle;
    return;
  }

  if (compute->request.op == Request::Operation::read) {
    auto l = data.begin() + compute->cacheline();
    auto r = l + parent->bank_width;
    compute->result = std::vector<uint8_t>(l, r);
  } else if (compute->request.op == Request::Operation::write) {
    compute->result = compute->request.operand;
    CHECK(compute->result.size() == parent->bank_width);
  } else {
    CHECK(false) << "not supported yet!";
  }
  write = compute;
  compute = nullptr;
}

void Bank::WriteBack() {
  if (write) {
    auto addr = write->cacheline();
    LOG(ADDR)
      << "[" << bankno << "] (" << write->request.port << ")"
      << write->request.addr << " " << write->cacheline() << " + "
      << ": [" << write->result.size() << "]bytes";
    CHECK(addr < data.size()) << addr;
    if (write->request.op != Request::Operation::read) {
      for (int i = 0; i < parent->bank_width; ++i) {
        if (write->request.mask[i]) {
          data[addr + i] = write->result[i];
        }
      }
      LOG(COMMIT) << " [Commit] " << bankno << ": Commit " << write->request;
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
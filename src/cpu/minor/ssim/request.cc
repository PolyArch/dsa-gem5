#include <algorithm>
#include <functional>
#include <numeric>

#include "dsa/debug.h"
#include "dsa/arch/spad.h"
#include "./spad.h"
#include "./request.h"

namespace dsa {
namespace sim {

const char *STATUS[] = {
  "NotIssued",
  "InFIFO",
  "Access",
  "Compute",
  "Write",
  "Commit",
};

std::ostream &operator<<(std::ostream &os, const Request &req) {
  return os << "[Request] addr: " << req.addr << ", " << " dtype: " << req.data_size
            << ", operand: " << "[" << req.operand.size() << "]"
            << ", op: " << (int) (req.op);
}

void RequestBuffer::Decode(int port, Request::Operation op,
                           const stream::LinearStream::LineInfo &request,
                           const std::vector<uint8_t> &operands) {
  CHECK(Available()) << "No available slot in the reorder buffer!";
  for (int i = 0; i < parent->num_banks; ++i) {
    Request uop(port, request.linebase + i * parent->bank_width, parent->bank_width, op);
    int l = i * parent->bank_width;
    int r = l + parent->bank_width;
    if (!operands.empty()) {
      uop.operand = std::vector<uint8_t>(operands.begin() + l, operands.begin() + r);
    }
    uop.mask = std::vector<bool>(request.mask.begin() + l, request.mask.begin() + r);
    scoreboard[tail].emplace_back(parent, uop);
  }
  info[tail] = request;
  tail = (tail + 1) % scoreboard.size();
}

void RequestBuffer::Decode(const std::vector<Request> &requests) {
  const int num_bytes = parent->num_bytes;
  const int bank_width = parent->bank_width;
  const int num_banks = parent->num_banks;
  CHECK(Available()) << "No available slot in the reorder buffer!";
  for (const auto &elem : requests) {
    CHECK(elem.data_size < bank_width * num_banks)
      << "Request cannot be larger than the bandwidth";
    CHECK(elem.op == requests[0].op)
      << "All grouped requests should have the same operation!";
    CHECK(elem.addr + elem.data_size <= num_bytes)
      << "Requested address exceed the space " << elem.addr << " >= " << num_bytes;
    CHECK(elem.addr % elem.data_size == 0)
      << "Requested address should be aligned with the data size";
    if (elem.data_size >= bank_width) {
      // If a word we request straddle multiple banks,
      // we should check if the banks are aligned by this word.
      CHECK(elem.addr % num_banks % (elem.data_size / bank_width) == 0)
        << "Decomposability is straddled! ("
        << elem.addr << " % " << num_banks << ") % ("
        << elem.data_size << " / " << bank_width << ")";
    }

    LOG(XBAR) << "Process " << elem;
    // FIXME(@were): Confirm the details of coalescing requests with @Sihao
    // auto operand = elem.operand;
    // uint64_t master;
    // for (int j = 0; j < elem.data_size; j += bank_width) {
    //   Request duplicate = elem;
    //   duplicate.addr += j;
    //   duplicate.operand = operand & (~0ull >> (64 - bank_width * 8));
    //   operand >>= bank_width * 8;
    //   entries.emplace_back(parent, duplicate);
    //   LOG(XBAR)
    //     << "Micro code: [" << entries.back().bankno() << "] "
    //     << entries.back().cacheline() << " + " << entries.back().offset()
    //     << "  " << duplicate.operand;
    //   LOG(MASTER) << entries.back().id << " <-> " << entries[0].id;
    // }
  }
  /* Register the entry on the scoreboard, or maybe we should call it ROB. */
  tail = (tail + 1) % scoreboard.size();
}

void InputBuffer::PushRequests() {
  int num_banks = parent->num_banks;
  /* If two address go to the same bank, we only allow the first one. */
  std::vector<int> cnt(num_banks, 0);
  int issue_checked = 0;
  bool head_found = false;

  for (int i = 0, n = scoreboard.size(); i < n; ++i) {
    auto &refer = scoreboard[(i + front) % scoreboard.size()];
    for (int j = 0, m = refer.size(); j < m; ++j) {
      auto &elem = refer[j];
      int bankno = refer[j].bankno();
      if (elem.status == Entry::Status::NotIssued) {
        head_found = true;
        auto pred = std::accumulate(elem.request.mask.begin(), elem.request.mask.end(), 0,
                                    std::plus<int>());
        if (pred) {
          LOG(IDLE)
            << issue_checked << ": try to issue "
            << i << "," << j << " to " << bankno;
          if (cnt[bankno] >= provision) {
            LOG(IDLE) << bankno << " over provisioned!";
          } else {
            if (!parent->banks[bankno].Available()) {
              LOG(IDLE) << bankno << " task fifo overwhelmed!";
            } else {
              ++cnt[bankno];
              parent->banks[bankno].task_fifo.push(&refer[j]);
              refer[j].status = Entry::Status::InFIFO;
              LOG(XBAR) << "[" << i << "] Push micro code "
                          << refer[j].request << " to bank " << bankno;
            }
          }
        } else {
          elem.status = Entry::Status::Commit;
        }
      } else {
        LOG(XBAR) << "[" << i << "] Already issued: @"
                    << STATUS[(int) elem.status] << " " << elem.request;
      }
      if (head_found) {
        if (++issue_checked > issue_width) {
          return;
        }
      }
    }
  }
}

Response RequestBuffer::Commit() {
  std::vector<uint8_t> res;
  stream::LinearStream::LineInfo meta;
  int port = -1;
  Request::Operation op = Request::Operation::unknown;
  if (!scoreboard[front].empty()) {
    port = scoreboard[front][0].request.port;
    op = scoreboard[front][0].request.op;
    for (int i = 0; i < (int) scoreboard[front].size(); ++i) {
      if (scoreboard[front][i].status != Entry::Status::Commit) {
        LOG(RETIRE) << "[" << front << "] " << scoreboard[front][i].request
                      << " is not committed yet! "
                      << STATUS[(int) scoreboard[front][i].status];
        return Response();
      }
    }
    LOG(RETIRE) << "Row " << front << " is retiring";
    for (int i = 0; i < (int) scoreboard[front].size(); ++i) {
      auto &entry = scoreboard[front][i];
      res.insert(res.end(), entry.result.begin(), entry.result.end());
    }
    scoreboard[front].clear();
    scoreboard[front].shrink_to_fit();
    meta = info[front];
    info[front] = stream::LinearStream::LineInfo();
    ++front;
    front %= scoreboard.size();
  }
  return Response(port, op, res, meta);
}

RequestBuffer::RequestBuffer(int sb_size) : scoreboard(sb_size), info(sb_size) {}

InputBuffer::InputBuffer(int size, int issue_width_, int provision_) :
  RequestBuffer(size), issue_width(issue_width_), provision(provision_) {}

bool RequestBuffer::Active() {
  for (int i = 0, n = scoreboard.size(); i < n; ++i) {
    if (!scoreboard[i].empty()) {
      return true;
    }
  }
  return false;
}

bool RequestBuffer::Available() {
  return scoreboard[tail].empty();
}

void LinkBuffer::PushRequests() {
  for (int i = 0, n = scoreboard.size(); i < n; ++i) {
    auto iter = (i + front) % scoreboard.size();
    auto &refer = scoreboard[iter];
    for (int j = 0, m = refer.size(); j < m; ++j) {
      auto &elem = refer[j];
      int bankno = refer[j].bankno();
      if (elem.status == Entry::Status::NotIssued && grid[bankno][iter] == nullptr) {
        grid[bankno][iter] = &elem;
        LOG(XBAR) << "Buffer bankno" << bankno << ", request" << iter << ": "
                    << STATUS[(int) elem.status] << elem.request << " " << &elem;
      }
    }
  }

  for (int i = 0, n = grid.size(); i < n; ++i) {
    for (int j = 0, m = scoreboard.size(); j < m; ++j) {
      int iter = (j + front) % scoreboard.size();
      if (auto &to_issue = grid[i][iter]) {
        CHECK(to_issue->status == Entry::Status::NotIssued)
          << STATUS[(int) to_issue->status] << ": bank" << i
          << " request" << iter << to_issue->request << " " << to_issue;
        if (parent->banks[i].Available()) {
          to_issue->status = Entry::Status::InFIFO;
          parent->banks[i].task_fifo.push(to_issue);
          LOG(XBAR) << "Issue bankno" << i << ", request" << iter << ": "
                      << to_issue->request << " " << to_issue;
          to_issue = nullptr;
        }
        break;
      }
    }
  }

}

LinkBuffer::LinkBuffer(int sb_size, int bank_size)
   : RequestBuffer(sb_size), grid(bank_size, std::vector<Entry*>(sb_size, nullptr)) {}


}
}
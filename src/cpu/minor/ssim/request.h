#include <cstdint>
#include <climits>
#include <vector>

#include "dsa-ext/rf.h"

#include "./linear_stream.h"

namespace dsa {
namespace sim {

struct ScratchMemory;

// TODO(@were): A vector of request should have the same operation and port,
//              unify them in a higher packet level.
/*!
 * \brief The requests of sent to scratchpad.
 */
struct Request {
  /*!
   * \brief The port destination of this request.
   */
  int port;
  /*!
   * \brief The address requested.
   */
  uint64_t addr{0};
  /*!
   * \brief The data size requested.
   */
  int data_size;
  /*!
   * \brief The predication of accessing each addressable element in this bank line.
   */
  std::vector<bool> mask;
  /*!
   * \brief The operands of atomic operation.
   */
  std::vector<uint8_t> operand;
  /*!
   * \brief The operator of operation.
   */
  MemoryOperation op{MemoryOperation::DMO_Unkown};

  Request(int port_, uint64_t addr_, int data_size_, MemoryOperation op_) :
    port(port_), addr(addr_), data_size(data_size_), op(op_) {}
};

std::ostream &operator<<(std::ostream &os, const Request &req);

/*!
 * \brief The execution entry in the scratchpad pipeline.
 */
struct Entry {
  /*!
   * \brief The indirect memory this execution entry belongs to.
   */
  ScratchMemory *parent;

  /*!
   * \brief The underlying request.
   */
  Request request;

  /*!
   * \brief The result of atomic operation.
   */
  std::vector<uint8_t> result;

  enum class Status { NotIssued, InFIFO, Access, Compute, Write, Commit };
  /*!
   * \brief The status of this entry in the pipeline.
   */
  Status status{Status::NotIssued};

  /*!
   * \brief Bank number.
   */
  uint64_t bankno() {
    return (request.addr / parent->bank_width) % parent->num_banks;
  }

  /*!
   * \brief The cacheline number in a bank.
   */
  uint64_t cacheline() {
    return request.addr / parent->bank_width / parent->num_banks * parent->bank_width;
  }

  Entry(ScratchMemory *parent_, const Request &request_);
};

struct Response {
  uint64_t id;
  MemoryOperation op;
  std::vector<uint8_t> raw;
  stream::LinearStream::LineInfo info;

  Response(uint64_t id_ = -1, MemoryOperation op_ = MemoryOperation::DMO_Unkown,
           const std::vector<uint8_t> &raw_ = {},
           const stream::LinearStream::LineInfo &info_ = stream::LinearStream::LineInfo()) :
    id(id_), op(op_), raw(raw_), info(info_) {}
};

/* \brief The base class holds the pending requests */
struct RequestBuffer {

  ScratchMemory *parent{nullptr};
  /* \brief A repetitive queue for filling the requests */
  int front{0}, tail{0};
  std::vector<std::vector<Entry>> scoreboard;
  std::vector<stream::LinearStream::LineInfo> info;

  RequestBuffer(int sb_size);

  /* \brief The strategy of pushing requests */
  virtual void PushRequests() = 0;

  /*!
   * \brief If this request buffer is available to process requests.
   */
  bool Available();

  /* \brief Breaks all the indirect scalar requests to bank micro codes. */
  void Decode(const std::vector<Request> &requests, const stream::LinearStream::LineInfo &meta);

  /* \brief Breaks a cacheline request to bank micro codes. */
  void Decode(int port, MemoryOperation op,
              const stream::LinearStream::LineInfo &request,
              const std::vector<uint8_t> &operands);

  /* \brief If there is ongoing requests */
  bool Active();

  /* \brief Commit the complete requests */
  Response Commit();
};

/* \brief The InputBuffer strategy */
struct InputBuffer : RequestBuffer {
  int issue_width;
  int provision{1};

  InputBuffer(int size, int issue_width = INT_MAX, int provision = 1);

  /* \brief The strategy of pushing requests */
  void PushRequests() override;
};

/* \brief The LinkBuffer strategy */
struct LinkBuffer : RequestBuffer {

  std::vector<std::vector<Entry*>> grid;
  std::vector<int> idx;

  LinkBuffer(int sb_size, int bank_size);

  LinkBuffer(int buffer);

  /* \brief The strategy of pushing requests */
  void PushRequests() override;
};

}
}
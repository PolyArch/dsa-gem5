#pragma once

#include <cstdint>
#include <cassert>
#include <vector>
#include <sstream>
#include <iostream>

#include "dsa/debug.h"
#include "dsa-ext/spec.h"
#include "dsa-ext/rf.h"

#include "./loc.hh"

struct BuffetEntry;

namespace dsa {
namespace sim {
namespace stream {

/*!
 * \brief The status of the state machine of an affine stream.
 */
struct AffineStatus {
  /*!
   * \brief The outer-most dimension of "first". Say, if we yield a 2d stream
   *        [[1,2,3,4,5],[6,7,8,9,10]]. When yielding 1, this value should be 2-d.
   *        Though it is also the first of a 1-d stream, 1-d is NOT the outer-most.
   *        However, when yielding 6, this value should be 1-d, since it is just the beginning
   *        of the second 1-d stream.
   */
  int dim_1st{0};
  /*!
   * \brief The outer-most dimension of "last". Explaination similar to what we have above.
   */
  int dim_last{0};
  /*!
   * \brief If this is the first response of this stream.
   */
  bool stream_1st{false};
  /*!
   * \brief If this is the last response of this stream.
   */
  bool stream_last{false};
  /*!
   * \brief The trip count of the inner most dimension.
   */
  int64_t n{-1};
  /*!
   * \brief The padding policy of this stream.
   */
  Padding padding{Padding::DP_NoPadding};
  /*!
   * \brief The bitmask of the tag.
   */
  uint32_t mask{~0u};
  /*!
   * \brief Pack the data to stream tag bitmask.
   * \param If this tag is for the 1st element of this response packet.
   * \param If this tag is for the last element of this response packet.
   */
  uint8_t toTag(bool packet_1st, bool packet_last) const;
  /*!
   * \brief Dump the textformat of the affine status for debugging.
   */
  std::string toString() const;

  AffineStatus() {}
};

/*! \brief The abstract class of a linear stream. */
struct LinearStream {

  LinearStream(int64_t volume_, bool is_mem_) : volume(volume_), is_mem(is_mem_) {}

  /*!
   * \brief The infomation of the a cacheline response.
   */
  struct LineInfo {
    /*!
     * \brief The address aligned to the cacheline, giving the starting address.
     */
    int64_t linebase{0};
    /*!
     * \brief The original starting address.
     */
    int64_t start{0};
    /*!
     * \brief The bitmask predicate of this cacheline operation.
     */
    std::vector<bool> mask;
    /*!
     * \brief If we want to shrink buffet after using this response.
     */
    int64_t shrink{0};
    /*!
     * \brief The status of this affined stream when generating this cacheline request.
     */
    AffineStatus as;

    LineInfo(int64_t linebase_, int64_t start_, const std::vector<bool> &mask_,
             int64_t shrink_, const AffineStatus &as_) :
      linebase(linebase_), start(start_), mask(mask_), shrink(shrink_), as(as_) {}

    LineInfo(int64_t linebase_, int64_t start_, const std::vector<bool> &mask_,
             int64_t shrink_) :
      linebase(linebase_), start(start_), mask(mask_), shrink(shrink_) {}

    /*!
     * \brief The valid bytes read from memory in this response.
     */
    int bytes_read() const;
  };

  /*! \brief If this linear stream has next element. */
  virtual bool hasNext() = 0;

  /*!
   * \brief Extract the front element from this stream.
   * \param next This element is popped.
   */
  virtual int64_t poll(bool next=true) = 0;

  /*!
   * \brief Extract several elements from this stream.
   * \param bandwidth The memory bandwidth.
   * \param available When it comes to a write stream, it is the number of elements
   *                  avaiable in the port FIFO.
   * \param be Buffet entry associated with this request.
   * \param mo Memory operation to be applied.
   * \param unit Is it either memory or spad.
   */
  virtual LineInfo cacheline(int bandwidth, int available, BuffetEntry *be,
                             MemoryOperation mo, LOC unit) = 0;

  /*!
   * \brief The text format of this stream for the purpose of debugging.
   */
  virtual std::string toString() = 0;

  /*!
   * \brief The data size of each "word" in this stream.
   */
  virtual int word_bytes() = 0;

  /*!
   * \brief The number of dimensions.
   */
  virtual int dimension() = 0;

  /*!
   * \brief The total bytes of data read by this stream.
   */
  int64_t volume;
  /*!
   * \brief If this stream is for memory.
   */
  bool is_mem;
  /*!
   * \brief The progress of this stream.
   */
  int64_t i{0};

  void updateStatus(AffineStatus &as, bool isLast) {
    as.stream_1st = as.stream_1st && i == 0;
    as.stream_last = !hasNext();
    as.dim_1st = i == 0 ? dimension() : as.dim_1st;
    as.dim_last = !hasNext() ? dimension() : as.dim_last;
  }
};

/*! \brief 1-d linear stream. */
struct Linear1D : LinearStream {

  /*!
   * \brief Construct a new 1d linear stream object.
   * \param word_ The coefficient multiplier of the stride and stretch.
   * \param start_ The starting value of generation.
   * \param length_ The trip count of the accumulation.
   * \param is_mem If this is a memory stream.
   */
  Linear1D(int64_t word_, int64_t start_, int64_t stride_, int64_t length_, bool is_mem) :
    LinearStream(word_ * length_, is_mem), word(word_), start(start_), stride(stride_), length(length_) {
    if (is_mem) {
      CHECK(word) << "word should not be zero!";
      CHECK(start % word == 0) << start << " " << std::dec << word;
    }
  }

  /*!
   * \brief The number of dimensions.
   */
  int dimension() override {
    return 1;
  }

  /*!  \brief If we have a next value to retrieve. */
  bool hasNext() override {
    return i < length;
  }

  /*!
   * \brief Retrieve a scalar value from the stream, used by const stream.
   * \param next If we want to pop the value.
   */
  int64_t poll(bool next=true) override {
    assert(hasNext());
    auto res = i * stride * word + start;
    if (next) {
      ++i;
    }
    return res;
  }

  /*!
   * \brief Retrive a cacheline starting address and the bitmask.
   * \param bandwidth The width of the cacheline in bytes.
   * \param at_most The the number of bits at most in the bitmask.
   */
  LineInfo cacheline(int bandwidth, int at_most, BuffetEntry *be, MemoryOperation mo, LOC unit) override;

  /*!
   * \brief The text format for the purpose of debugging.
   */
  std::string toString() override {
    auto &l1d = *this;
    std::ostringstream oss;
    oss << "start:" << l1d.start
        << " stride1d:" << l1d.stride
        << " word:" << l1d.word
        << " cnt:" << l1d.i << "/" << l1d.length;
    if (hasNext()) {
      oss << " next: " << l1d.poll(false);
    } else {
      oss << " [end]";
    }
    return oss.str();
  }

  int word_bytes() override {
    return word;
  }

  int64_t word;
  int64_t start;
  int64_t stride;
  int64_t length;
};


struct Linear2D : LinearStream {
  Linear2D(int64_t word_, int64_t start_, int64_t stride1d, int64_t lin,
           int64_t stretch_,  int64_t stride2d, int64_t lout,
           bool is_mem_) :
    LinearStream((lin + (stretch_ * (lout - 1) + lin)) * lout / 2 * word_, is_mem_),
    stretch(stretch_), stride(stride2d), length(lout),
    exec(word_, start_, stride1d, lin, is_mem),
    init(word_, start_, stride1d, lin, is_mem) {}

  bool hasNext() override {
    if (i < length) {
      if (exec.hasNext()) {
        return true;
      }
      if (++i == length) {
        return false;
      }
      exec = Linear1D(init.word,
                      init.start + stride * init.word,
                      init.stride,
                      init.length + stretch,
                      is_mem);
      init = exec;
      return exec.hasNext();
    }
    return false;
  }

  /*!
   * \brief The number of dimensions.
   */
  int dimension() override {
    return 2;
  }

  int64_t poll(bool next=true) override {
    assert(hasNext());
    return exec.poll(next);
  }

  LineInfo cacheline(int bandwidth, int available, BuffetEntry *be, MemoryOperation mo, LOC unit) override {
    auto res = exec.cacheline(bandwidth, available, be, mo, unit);
    res.as.stream_1st = res.as.stream_1st && i == 0;
    res.as.stream_last = !hasNext();
    res.as.dim_1st = i == 0 ? 2 : res.as.dim_1st;
    res.as.dim_last = i == length ? 2 : res.as.dim_last;
    if (hasNext()) {
      res.shrink = std::min(res.shrink, init.start);
      DSA_LOG(SHRINK) << "shrink: " << res.shrink << " " << init.start;
    }
    return res;
  }

  std::string toString() override {
    if (hasNext()) {
      auto &l2d = *this;
      std::ostringstream oss;
      oss << "stride:" << l2d.stride
          << " stretch:" << l2d.stretch
          << " cnt:" << l2d.i << "/" << l2d.length
          << " inner: [" << l2d.exec.toString() << "]";
      return oss.str();
    } else {
      return "[2-d stream to retire]";
    }
  }

  int word_bytes() override {
    return init.word_bytes();
  }

  int64_t stretch;
  int64_t stride;
  int64_t length;
  Linear1D exec;
  Linear1D init;
};

struct Linear3D : LinearStream {
  Linear3D(int64_t word_, int64_t start_, int64_t stride1d_,
           int64_t l1d, int64_t stretch_2d_, int64_t stride_2d_, int64_t l2d,
           int64_t delta_stretch_2d_, int64_t delta_stride_2d_,
           int64_t delta_length_1d_, int64_t delta_length_2d_,
           int64_t stride_3d_, int64_t l3d, bool is_mem_) :
    LinearStream(0, is_mem_),
    delta_stretch_2d(delta_stretch_2d_),
    delta_stride_2d(delta_stride_2d_),
    delta_length_1d(delta_length_1d_),
    delta_length_2d(delta_length_2d_),
    stride_3d(stride_3d_),
    length(l3d),
    init(word_, start_, stride1d_, l1d, stretch_2d_, stride_2d_, l2d, is_mem),
    exec(word_, start_, stride1d_, l1d, stretch_2d_, stride_2d_, l2d, is_mem) {
      volume += init.volume;
    }
  
  bool hasNext() override {
    if (i < length) {
      if (exec.hasNext()) {
        return true;
      }
      if (++i == length) {
        return false;
      }
      exec = Linear2D(
        init.init.word,
        init.init.start + stride_3d * word_bytes(),
        init.init.stride, init.init.length + delta_length_1d,
        init.stretch + delta_stretch_2d, init.stride + delta_stride_2d,
        init.length + delta_length_2d, is_mem);
      init = exec;
      volume += init.volume;
      return exec.hasNext();
    }
    return false;
  }

  int64_t poll(bool next = true) override {
    assert(hasNext());
    return exec.poll(next);
  }

  LineInfo cacheline(int bandwidth, int available, BuffetEntry *be, MemoryOperation mo, LOC unit) override {
    auto res = exec.cacheline(bandwidth, available, be, mo, unit);
    res.as.stream_1st = res.as.stream_1st && i == 0;
    res.as.stream_last = !hasNext();
    res.as.dim_1st = i == 0 ? 3 : res.as.dim_1st;
    res.as.dim_last = i == length ? 3 : res.as.dim_last;
    if (hasNext()) {
      res.shrink = std::min(res.shrink, init.init.start);
      DSA_LOG(SHRINK) << "shrink: " << res.shrink << " " << init.init.start;
    }
    return res;
  }

  int64_t delta_stretch_2d;
  int64_t delta_stride_2d;
  int64_t delta_length_1d;
  int64_t delta_length_2d;
  int64_t stride_3d;
  int64_t length;
  Linear2D init;
  Linear2D exec;

  std::string toString() override {
    std::ostringstream oss;
    if (hasNext()) {
      oss << "delta 2d-stretch: " << delta_stretch_2d
          << " delta 2d-stride: " << delta_stride_2d
          << " delta 1d-length: " << delta_length_1d
          << " delta 2d-length: " << delta_length_2d
          << " 3d-stride: " << stride_3d
          << " " << i << "/" << length
          << " [" << exec.toString() << "]";
    } else {
      return "[3-d stream to retire]";
    }
    return oss.str();
  }

  int word_bytes() override {
    return init.word_bytes();
  }

  /*!
   * \brief The number of dimensions.
   */
  int dimension() override {
    return 3;
  }
};

}
}
}

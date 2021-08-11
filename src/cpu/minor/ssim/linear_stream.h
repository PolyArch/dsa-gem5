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

/*! \brief The abstract class of a linear stream. */
struct LinearStream {

  LinearStream(int64_t volume_, bool is_mem_) : volume(volume_), is_mem(is_mem_) {}

  /*! \brief The infomation of the retrieved cacheline. */
  struct LineInfo {
    int64_t linebase;
    int64_t start;
    std::vector<bool> mask;
    int64_t shrink;
    bool stride_first;
    bool stride_last;
    bool stream_last;
    int padding;
    explicit LineInfo(int64_t linebase_ = 0, int64_t start_ = 0,
                      const std::vector<bool> &mask_ = {},
                      int64_t shrink_ = 0, bool stride_first_ = false,
                      bool stride_last_ = false, bool stream_last_ = false,
                      int padding_ = 0) :
      linebase(linebase_), start(start_), mask(mask_), shrink(shrink_),
      stride_first(stride_first_), stride_last(stride_last_),
      stream_last(stream_last_), padding(padding_) {}

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

  int64_t volume;
  bool is_mem;
  int64_t i{0};
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

  int64_t poll(bool next=true) override {
    assert(hasNext());
    return exec.poll(next);
  }

  LineInfo cacheline(int bandwidth, int available, BuffetEntry *be, MemoryOperation mo, LOC unit) override {
    auto res = exec.cacheline(bandwidth, available, be, mo, unit);
    res.stream_last = false;
    res.stream_last = !hasNext();
    if (hasNext()) {
      res.shrink = std::min(res.shrink, init.start);
      LOG(SHRINK) << "shrink: " << res.shrink << " " << init.start;
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
    res.stream_last = false;
    res.stream_last = !hasNext();
    if (hasNext()) {
      res.shrink = std::min(res.shrink, init.init.start);
      LOG(SHRINK) << "shrink: " << res.shrink << " " << init.init.start;
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
};

}
}
}

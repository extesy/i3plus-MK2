#ifndef __CIRCULARQUEUE_H__
#define __CIRCULARQUEUE_H__
#include <Arduino.h>
template<typename T, uint8_t N>
class CircularQueue {
  private:
    struct buffer_t {
      uint8_t head;
      uint8_t tail;
      uint8_t count;
      uint8_t size;
      T queue[N];
    } buffer;
  public:
    CircularQueue<T, N>() {
      this->buffer.size = N;
      this->buffer.count = this->buffer.head = this->buffer.tail = 0;
    }
    T dequeue() {
      if (this->isEmpty()) return T();
      uint8_t index = this->buffer.head;
      --this->buffer.count;
      if (++this->buffer.head == this->buffer.size)
        this->buffer.head = 0;
      return this->buffer.queue[index];
    }
    bool enqueue(T const &item) {
      if (this->isFull()) return false;
      this->buffer.queue[this->buffer.tail] = item;
      ++this->buffer.count;
      if (++this->buffer.tail == this->buffer.size)
        this->buffer.tail = 0;
      return true;
    }
    bool isEmpty() {
      return this->buffer.count == 0;
    }
    bool isFull() {
      return this->buffer.count == this->buffer.size;
    }
    uint8_t size() {
      return this->buffer.size;
    }
    T peek() {
      return this->buffer.queue[this->buffer.head];
    }
    uint8_t count() {
      return this->buffer.count;
    }
};
#endif

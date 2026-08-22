#ifndef POLL_LOG_H
#define POLL_LOG_H

#include <stdio.h>
#include <stdint.h>

struct TH_Poll {
  // these are from 0 to 2^16, and must be converted as in Poll_TH()
  // if 0, do not accept as data
  uint16_t temp;
  uint16_t hum;

  //uint16_t minSec; // stores minutes and seconds as:
  // 0000 MMMM MMSS SSSS
  //uint8_t hour; // 00 - 23
  //uint8_t day;
  uint32_t timestamp;
};

struct Poll_Node {
  struct Poll_Node* prev;
  struct Poll_Node* next;
  struct TH_Poll polls[10];
  uint8_t written; // where to write next
};

void append_poll_node(struct Poll_Node* node);
void add_log(struct Poll_Node* node, struct TH_Poll* log);
void clear_log(struct Poll_Node* node);

#endif
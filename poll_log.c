#include <stdio.h>
#include <stdlib.h>
#include "poll_log.h"

void append_poll_node(struct Poll_Node* node) {
  node->next = malloc(sizeof(struct Poll_Node));
  if (node->next == NULL) return;

  struct Poll_Node* newNode = node->next;

  // init th_polls to 0
  for (int i = 0; i < 10; i++) {
    newNode->polls[i].hum = 0;
    newNode->polls[i].temp = 0;
  }

  newNode->prev = node;
  newNode->next = NULL;
  newNode->written = 0;
}

void add_log(struct Poll_Node* node, struct TH_Poll* log) {
  struct Poll_Node* n = node;

  // find latest poll
  while (n->next != NULL) {
    n = n->next;
  }

  if (n->written < 10) {
    n->polls[n->written].hum = log->hum;
    n->polls[n->written].temp = log->temp;
    n->polls[n->written].timestamp = log->timestamp;
    /*
    n->polls[n->written].minSec = log->minSec;
    n->polls[n->written].hour = log->hour;
    n->polls[n->written].day = log->day;
    */
    n->written++;
    return;
  }

  append_poll_node(n);

  n->next->polls[0].hum = log->hum;
  n->next->polls[0].temp = log->temp;
  n->next->polls[0].timestamp = log->timestamp;
  /*
  n->next->polls[0].minSec = log->minSec;
  n->next->polls[0].hour = log->hour;
  n->next->polls[0].day = log->day;
  */
  n->next->written = 1;
}

void clear_log(struct Poll_Node* node) {
  struct Poll_Node* n = node->next;
  struct Poll_Node* nx = NULL;

  while (n != NULL) {
    nx = n->next;
    free(n);
    n = nx;
  }

  // do not deallocate base (first) node, instead reset
  node->next = NULL;
  node->written = 0;
  for (int i = 0; i < 10; i++) {
    /*
    node->polls[i].day = 1;
    node->polls[i].hour = 0;
    node->polls[i].minSec = 0;
    */
    node->polls[i].timestamp = 0;
    node->polls[i].hum = 0;
    node->polls[i].temp = 0;
  }
}

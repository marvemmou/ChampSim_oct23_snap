/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "trace_instruction.h"

// branch types
enum branch_type {
  NOT_BRANCH = 0,
  BRANCH_DIRECT_JUMP = 1,
  BRANCH_INDIRECT = 2,
  BRANCH_CONDITIONAL = 3,
  BRANCH_DIRECT_CALL = 4,
  BRANCH_INDIRECT_CALL = 5,
  BRANCH_RETURN = 6,
  BRANCH_OTHER = 7
};

struct ooo_model_instr {
  uint64_t instr_id = 0;
  uint64_t ip = 0;
  uint64_t event_cycle = 0;

  bool is_branch = 0;
  bool branch_taken = 0;
  bool branch_prediction = 0;
  bool branch_mispredicted = 0; // A branch can be mispredicted even if the direction prediction is correct when the predicted target is not correct

  std::array<uint8_t, 2> asid = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};

  uint8_t branch_type = NOT_BRANCH;
  uint64_t branch_target = 0;

  uint8_t dib_checked = 0;
  uint8_t fetched = 0;
  uint8_t decoded = 0;
  uint8_t scheduled = 0;
  uint8_t executed = 0;

  unsigned completed_mem_ops = 0;
  int num_reg_dependent = 0;

  std::vector<uint8_t> destination_registers = {}; // output registers
  std::vector<uint8_t> source_registers = {};      // input registers

  std::vector<uint64_t> destination_memory = {};
  std::vector<uint64_t> source_memory = {};

  // these are indices of instructions in the ROB that depend on me
  std::vector<std::reference_wrapper<ooo_model_instr>> registers_instrs_depend_on_me;

  uint32_t trace_id = 0;

  uint64_t rob_head_cycle = 0;

  // did this instr trigger a switch?
  bool did_trigger_switch = 0;
  // should this instr be able to trigger a switch? default is 0
  bool cant_trigger_switch = 0;
  
  bool went_offchip = 0, went_offchip_pred = 0;
  bool llc_miss_pred = 0, critical_pred = 0;
  uint64_t rob_stall_cycles_pred = 0;

  bool is_cxl = false;
  bool is_warmup = false;
  // whether we should replay the instr or not
  bool is_in_runahead = false;
  // whether the instr should be retired immediately or not
  bool useful = false;
  // whether the instr is part of a valid execution chain or not
  bool invalid = false;
  // whether this is the replayed instance or not
  bool from_replay = false;


  mlp_instr mlp_trace_info = {};

private:
  template <typename T>
  ooo_model_instr(T instr, std::array<uint8_t, 2> local_asid, uint32_t tid) : ip(instr.ip), is_branch(instr.is_branch), branch_taken(instr.branch_taken), asid(local_asid), trace_id(tid)
  {
    std::remove_copy(std::begin(instr.destination_registers), std::end(instr.destination_registers), std::back_inserter(this->destination_registers), 0);
    std::remove_copy(std::begin(instr.source_registers), std::end(instr.source_registers), std::back_inserter(this->source_registers), 0);
    std::remove_copy(std::begin(instr.destination_memory), std::end(instr.destination_memory), std::back_inserter(this->destination_memory), 0);
    std::remove_copy(std::begin(instr.source_memory), std::end(instr.source_memory), std::back_inserter(this->source_memory), 0);

    bool writes_sp = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_STACK_POINTER);
    bool writes_ip = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_INSTRUCTION_POINTER);
    bool reads_sp = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_STACK_POINTER);
    bool reads_flags = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_FLAGS);
    bool reads_ip = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_INSTRUCTION_POINTER);
    bool reads_other = std::count_if(std::begin(source_registers), std::end(source_registers), [](uint8_t r) {
      return r != champsim::REG_STACK_POINTER && r != champsim::REG_FLAGS && r != champsim::REG_INSTRUCTION_POINTER;
    });

    for (uint32_t i = 0; i < destination_memory.size(); i++) {
      destination_memory[i] = (((destination_memory[i] << 5) >> 5) | (((uint64_t)trace_id) << 59));
    }

    for (int i = 0; i < source_memory.size(); i++) {
      source_memory[i] = (((source_memory[i] << 5) >> 5) | (((uint64_t)trace_id) << 59));
    }

    ip = (((ip << 5) >> 5) | (((uint64_t)trace_id) << 59));
    
    // determine what kind of branch this is, if any
    if (!reads_sp && !reads_flags && writes_ip && !reads_other) {
      // direct jump
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_DIRECT_JUMP;
    } else if (!reads_sp && !reads_flags && writes_ip && reads_other) {
      // indirect branch
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_INDIRECT;
    } else if (!reads_sp && reads_ip && !writes_sp && writes_ip && reads_flags && !reads_other) {
      // conditional branch
      is_branch = true;
      branch_taken = instr.branch_taken; // don't change this
      branch_type = BRANCH_CONDITIONAL;
    } else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && !reads_other) {
      // direct call
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_DIRECT_CALL;
    } else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && reads_other) {
      // indirect call
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_INDIRECT_CALL;
    } else if (reads_sp && !reads_ip && writes_sp && writes_ip) {
      // return
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_RETURN;
    } else if (writes_ip) {
      // some other branch type that doesn't fit the above categories
      is_branch = true;
      branch_taken = instr.branch_taken; // don't change this
      branch_type = BRANCH_OTHER;
    } else {
      branch_taken = false;
    }
  }

public:
  ooo_model_instr(uint8_t cpu, input_instr instr, uint32_t tid) : ooo_model_instr(instr, {cpu, cpu},tid) {}
  ooo_model_instr(uint8_t, cloudsuite_instr instr, uint32_t tid) : ooo_model_instr(instr, {instr.asid[0], instr.asid[1]},tid) {}

  std::size_t num_mem_ops() const { return std::size(destination_memory) + std::size(source_memory); }

  static bool program_order(const ooo_model_instr& lhs, const ooo_model_instr& rhs) { return lhs.instr_id < rhs.instr_id; }
};

#endif

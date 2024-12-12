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

#ifndef TRACE_INSTRUCTION_H
#define TRACE_INSTRUCTION_H

#include <limits>

// special registers that help us identify branches
namespace champsim
{
constexpr char REG_STACK_POINTER = 6;
constexpr char REG_FLAGS = 25;
constexpr char REG_INSTRUCTION_POINTER = 26;
} // namespace champsim

// instruction format
constexpr std::size_t NUM_INSTR_DESTINATIONS_SPARC = 4;
constexpr std::size_t NUM_INSTR_DESTINATIONS = 2;
constexpr std::size_t NUM_INSTR_SOURCES = 4;

struct input_instr {
  // instruction pointer or PC (Program Counter)
  unsigned long long ip;

  // branch info
  unsigned char is_branch;
  unsigned char branch_taken;

  unsigned char destination_registers[NUM_INSTR_DESTINATIONS]; // output registers
  unsigned char source_registers[NUM_INSTR_SOURCES];           // input registers

  unsigned long long destination_memory[NUM_INSTR_DESTINATIONS]; // output memory
  unsigned long long source_memory[NUM_INSTR_SOURCES];           // input memory
};

struct cloudsuite_instr {
  // instruction pointer or PC (Program Counter)
  unsigned long long ip;

  // branch info
  unsigned char is_branch;
  unsigned char branch_taken;

  unsigned char destination_registers[NUM_INSTR_DESTINATIONS_SPARC]; // output registers
  unsigned char source_registers[NUM_INSTR_SOURCES];                 // input registers

  unsigned long long destination_memory[NUM_INSTR_DESTINATIONS_SPARC]; // output memory
  unsigned long long source_memory[NUM_INSTR_SOURCES];                 // input memory

  unsigned char asid[2];
};

struct mlp_instr {
  // instruction pointer or PC (Program Counter)
  // set for instrs with source memory
  unsigned long long ip = 0;

  // virtual address of the load
  // set for instrs with source memory
  unsigned long long v_address[4] = {};
  unsigned long long p_address[4] = {};
  
  // was it an llc miss
  bool went_offchip[4] = {};

  // cycles: scheduled, completed, head_of_rob, retired
  // set for: loads, loads, all instrs, is_memory instrs
  unsigned long long add_lq_c[4] = {}, schedule_c[4] = {}, issue_c[4] = {}, completed_c[4] = {}, rob_head_c = 0, retired_c = 0;
  unsigned long long fetch_c = 0, start_fetch_c = 0, decode_c = 0, dispatch_c = 0, rob_add_c = 0;
  // mlp array with the # of llc misses at retire
  // set for is_memory
  unsigned int mlp[512] = {};

  unsigned int trace_id = 0;
  unsigned long long fetch = 0, flush = 0;
};

#endif

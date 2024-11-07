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

#include <algorithm>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>
#include <iostream>

#include "champsim.h"
#include "champsim_constants.h"
#include "core_inst.inc"
#include "phase_info.h"
#include "stats_printer.h"
#include "tracereader.h"
#include "vmem.h"
#include <CLI/CLI.hpp>
#include <fmt/core.h>

namespace champsim
{
std::vector<phase_stats> main(environment& env, std::vector<phase_info>& phases, std::vector<tracereader>& traces);
}

int main(int argc, char** argv)
{
  champsim::configured::generated_environment gen_environment{};

  CLI::App app{"A microarchitecture simulator for research and education"};

  bool knob_cloudsuite{false};
  uint64_t warmup_instructions = 0;
  uint64_t simulation_instructions = std::numeric_limits<uint64_t>::max();
  std::string json_file_name;
  std::vector<std::string> trace_names;

  auto set_heartbeat_callback = [&](auto) {
    for (O3_CPU& cpu : gen_environment.cpu_view())
      cpu.show_heartbeat = false;
  };

  app.add_flag("-c,--cloudsuite", knob_cloudsuite, "Read all traces using the cloudsuite format");
  app.add_flag("--hide-heartbeat", set_heartbeat_callback, "Hide the heartbeat output");
  auto warmup_instr_option = app.add_option("-w,--warmup-instructions", warmup_instructions, "The number of instructions in the warmup phase");
  auto deprec_warmup_instr_option =
      app.add_option("--warmup_instructions", warmup_instructions, "[deprecated] use --warmup-instructions instead")->excludes(warmup_instr_option);
  auto sim_instr_option = app.add_option("-i,--simulation-instructions", simulation_instructions,
                                         "The number of instructions in the detailed phase. If not specified, run to the end of the trace.");
  auto deprec_sim_instr_option =
      app.add_option("--simulation_instructions", simulation_instructions, "[deprecated] use --simulation-instructions instead")->excludes(sim_instr_option);

  auto json_option =
      app.add_option("--json", json_file_name, "The name of the file to receive JSON output. If no name is specified, stdout will be used")->expected(0, 1);

  app.add_option("--traces", trace_names, "The paths to the traces")->required()->check(CLI::ExistingFile);

  bool f_allowed = 0;
  uint32_t switch_point = 0, switch_policy = 0;
  bool is_cgmt = 0;
  bool record = 0;

  auto switch_policy_option = app.add_option("-p", switch_policy, "Context switch policy");
  
  app.add_flag("-f", f_allowed, "Flush allowed");
  
  auto switch_point_option = app.add_option("-s", switch_point, "Context switch point");
  
  app.add_flag("-m", is_cgmt, "Coarse-grain MT");

  app.add_flag("-r", record, "Create MLP record");


  CLI11_PARSE(app, argc, argv);

  const bool warmup_given = (warmup_instr_option->count() > 0) || (deprec_warmup_instr_option->count() > 0);
  const bool simulation_given = (sim_instr_option->count() > 0) || (deprec_sim_instr_option->count() > 0);

  if (deprec_warmup_instr_option->count() > 0)
    fmt::print("WARNING: option --warmup_instructions is deprecated. Use --warmup-instructions instead.\n");

  if (deprec_sim_instr_option->count() > 0)
    fmt::print("WARNING: option --simulation_instructions is deprecated. Use --simulation-instructions instead.\n");

  if (simulation_given && !warmup_given)
    warmup_instructions = simulation_instructions * 2 / 10;

  std::vector<champsim::tracereader> traces;
  std::transform(
      std::begin(trace_names), std::end(trace_names), std::back_inserter(traces),
      [knob_cloudsuite, repeat = simulation_given, i = uint8_t(0)](auto name) mutable { return get_tracereader(name, i++, knob_cloudsuite, repeat); });

  for (int i=0; i<traces.size(); i++) {
    traces[i].trace_id = i;
  }

  std::vector<champsim::phase_info> phases{
      {champsim::phase_info{"Warmup", true, warmup_instructions * traces.size(), std::vector<std::size_t>(std::size(trace_names), 0), trace_names},
       champsim::phase_info{"Simulation", false, simulation_instructions * traces.size(), std::vector<std::size_t>(std::size(trace_names), 0), trace_names}}};

  for (auto& p : phases)
    std::iota(std::begin(p.trace_index), std::end(p.trace_index), 0);


  std::string first_trace = trace_names[0];
  std::size_t lastSlashPos = first_trace.find_last_of('/');
  std::size_t lastPeriodPos = first_trace.find_last_of('.');
  first_trace = first_trace.substr(lastSlashPos + 1, lastPeriodPos - lastSlashPos - 1);
  first_trace += "_mlp_trace.txt";
  first_trace = "mlp_traces_1Mistructions/" + first_trace;
  std::cout << "opened " << first_trace << std::endl;
  

  for (O3_CPU& cpu : gen_environment.cpu_view()) {
    cpu.flush_allowed = f_allowed;
    cpu.switch_point = switch_point;
    cpu.switch_policy = switch_policy;
    cpu.is_cgmt = is_cgmt;
    cpu.record = record;
    cpu.num_traces = traces.size();
    cpu.caches = gen_environment.cache_view();

    //cpu.outfile.open(first_trace, std::ios_base::binary);
   
    for(auto i=0; i<traces.size(); i++) {
      //cpu->trace_pos[i] = &traces[i]->start_pos;
      if (record) {
        cpu.outfile.open(first_trace, std::ios_base::binary | std::ios_base::trunc);
        if (cpu.outfile.is_open()) {
          std::cout << "File opened successfully!" << std::endl;
          // You can proceed with file operations here
        } else {
          std::cout << "Failed to open the file." << std::endl;
          assert(0);
        }
      }
      else {
        cpu.infile[i].open(first_trace, std::ios_base::binary); // | std::ios_base::trunc);
        if (cpu.infile[i].is_open()) {
          std::cout << "File opened successfully!" << std::endl;
          // You can proceed with file operations here
        } else {
          std::cout << "Failed to open the file." << std::endl;
          //assert(0);
        }
      }
    }
  }

  fmt::print("\n*** ChampSim Multicore Out-of-Order Simulator ***\nWarmup Instructions: {}\nSimulation Instructions: {}\nNumber of CPUs: {}\nPage size: {}\n\n",
             phases.at(0).length, phases.at(1).length, std::size(gen_environment.cpu_view()), PAGE_SIZE);

  fmt::print("\nSwitch policy: {}\nSwitch point: {}\n\n",switch_policy,switch_point);

  auto phase_stats = champsim::main(gen_environment, phases, traces);

  fmt::print("\nChampSim completed all CPUs\n\n");

  for (O3_CPU& cpu : gen_environment.cpu_view()) {
      cpu.timestamp_file.close(); 
      if (record)
        cpu.outfile.close();
      else {
      for(auto i=0; i<traces.size(); i++) 
          cpu.infile[i].close();
      }
  }

  champsim::plain_printer{std::cout}.print(phase_stats);

  for (CACHE& cache : gen_environment.cache_view())
    cache.impl_prefetcher_final_stats();

  for (CACHE& cache : gen_environment.cache_view())
    cache.impl_replacement_final_stats();

  if (json_option->count() > 0) {
    if (json_file_name.empty()) {
      champsim::json_printer{std::cout}.print(phase_stats);
    } else {
      std::ofstream json_file{json_file_name};
      champsim::json_printer{json_file}.print(phase_stats);
    }
  }

  return 0;
}

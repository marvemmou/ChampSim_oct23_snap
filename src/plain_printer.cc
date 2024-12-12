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

#include <numeric>
#include <sstream>
#include <utility>
#include <vector>

#include "stats_printer.h"
#include <fmt/core.h>
#include <fmt/ostream.h>
#include "ooo_cpu.h"

void champsim::plain_printer::print(O3_CPU::stats_type stats)
{
  constexpr std::array<std::pair<std::string_view, std::size_t>, 6> types{
      {std::pair{"BRANCH_DIRECT_JUMP", BRANCH_DIRECT_JUMP}, std::pair{"BRANCH_INDIRECT", BRANCH_INDIRECT}, std::pair{"BRANCH_CONDITIONAL", BRANCH_CONDITIONAL},
       std::pair{"BRANCH_DIRECT_CALL", BRANCH_DIRECT_CALL}, std::pair{"BRANCH_INDIRECT_CALL", BRANCH_INDIRECT_CALL},
       std::pair{"BRANCH_RETURN", BRANCH_RETURN}}};

  auto total_branch = std::ceil(
      std::accumulate(std::begin(types), std::end(types), 0ll, [tbt = stats.total_branch_types](auto acc, auto next) { return acc + tbt[next.second]; }));
  auto total_mispredictions = std::ceil(
      std::accumulate(std::begin(types), std::end(types), 0ll, [btm = stats.branch_type_misses](auto acc, auto next) { return acc + btm[next.second]; }));

  fmt::print(stream, "\n{} cumulative IPC: {:.4g} instructions: {} cycles: {}\n", stats.name, std::ceil(stats.instrs()) / std::ceil(stats.cycles()),
             stats.instrs(), stats.cycles());
  fmt::print(stream, "{} Branch Prediction Accuracy: {:.4g}% MPKI: {:.4g} Average ROB Occupancy at Mispredict: {:.4g}\n", stats.name,
             (100.0 * std::ceil(total_branch - total_mispredictions)) / total_branch, (1000.0 * total_mispredictions) / std::ceil(stats.instrs()),
             std::ceil(stats.total_rob_occupancy_at_branch_mispredict) / total_mispredictions);

  fmt::print(stream, "{} ROB stalls due to an llc miss: {}\n", stats.name, stats.rob_stalls);
  fmt::print(stream, "{} Context Switches: {}\n", stats.name, stats.num_context_switches);
  fmt::print(stream, "{} Instructions retired during runahead without executing: {}\n", stats.name, stats.num_of_useless_instructions);
  fmt::print(stream, "{} Instructions retired during runahead after executing: {}\n", stats.name, stats.num_of_useful_instructions);
  fmt::print(stream, "{} Instructions retired in runahead: {}\n", stats.name, stats.num_of_runahead_instructions);

  std::vector<double> mpkis;
  std::transform(std::begin(stats.branch_type_misses), std::end(stats.branch_type_misses), std::back_inserter(mpkis),
                 [instrs = stats.instrs()](auto x) { return 1000.0 * std::ceil(x) / std::ceil(instrs); });

  fmt::print(stream, "Branch type MPKI\n");
  for (auto [str, idx] : types)
    fmt::print(stream, "{}: {:.3}\n", str, mpkis[idx]);

  // MISS Pred Stats
  fmt::print(stream, "\nSH: ------------ MISS PRED STATS -----------------\n");
  fmt::print(stream, "{} MISS_PRED_ACC: {:.4}\n", stats.name, stats.MP_stats.correct_offchip/stats.MP_stats.MP_total_miss_preds);
  fmt::print(stream, "{} MISS_PRED_COV: {:.4}\n", stats.name, stats.MP_stats.correct_offchip/stats.MP_stats.total_offchip);
  fmt::print(stream, "\n");

  fmt::print(stream, "\nSH: ------------ MLP PRED STATS -----------------\n");
  fmt::print(stream, "{} MLP_TP: {:.4}\n", stats.name, stats.MLP_stats.mlp_TP/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_TN: {:.4}\n", stats.name, stats.MLP_stats.mlp_TN/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_FP: {:.4}\n", stats.name, stats.MLP_stats.mlp_FP/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_FN: {:.4}\n\n", stats.name, stats.MLP_stats.mlp_FN/stats.MLP_stats.mlp_total);
  
  fmt::print(stream, "{} MLP_DIST_ACC: {:.4}\n", stats.name, stats.MLP_stats.mlp_dist_acc/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_ERROR: {:.4}\n", stats.name, stats.MLP_stats.mlp_error/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_PERC_OVERLAP: {:.4}\n\n", stats.name, stats.MLP_stats.mlp_perc_overlapped/stats.MLP_stats.mlp_total);
  
  fmt::print(stream, "{} MLP_MIN_DIST: {:.4}\n", stats.name, stats.MLP_stats.mlp_min_dist/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_MAX_DIST: {:.4}\n", stats.name, stats.MLP_stats.mlp_max_dist/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_DENSITY: {:.4}\n", stats.name, stats.MLP_stats.mlp_density/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_MAX_DIST_CYCLES: {:.4}\n", stats.name, stats.MLP_stats.mlp_max_dist_cycles/stats.MLP_stats.mlp_total);
  fmt::print(stream, "{} MLP_DENSITY_CYCLES: {:.4}\n\n", stats.name, stats.MLP_stats.mlp_density_cycles/stats.MLP_stats.mlp_total);

  fmt::print(stream, "\nSH: ------------ CRIT PRED STATS -----------------\n");
  
  fmt::print(stream, "\nSH: Top 10 IPs: \n");
  // Unpack CritFrequencies
  // Convert map to vector of pairs
  std::vector<std::pair<uint64_t, per_load_stats>> vec(stats.per_load_stat.begin(), stats.per_load_stat.end());

  // Sort vector in descending order of value
  std::sort(vec.begin(), vec.end(),
      [](const std::pair<uint64_t, per_load_stats>& a, const std::pair<uint64_t, per_load_stats>& b) {
          return a.second.cnt[CRIT]  > b.second.cnt[CRIT];
      });

  fmt::print(stream, "\n");
  for (int i = 0; i < 10 && i < vec.size(); ++i) {
    fmt::print("IP: {:d}, Freq: {:.4} %, Crit Freq: {:.4}, Noncrit Freq: {:.4}, Crit Hit Freq: {:.4}, Crit Cycles (all avg): {:.4}, Crit Cycles (Crit avg): {:.4}\n", vec[i].first, vec[i].second.cnt[CRIT]*100/stats.crit_stats[CRIT].cnt, vec[i].second.cnt[CRIT], vec[i].second.cnt[NON_CRIT], vec[i].second.cnt[CRIT_HIT], vec[i].second.crit_cycles/vec[i].second.occ, vec[i].second.crit_cycles/vec[i].second.cnt[CRIT]);
  }

  fmt::print("\n");

  fmt::print(stream, "{} ACC: FREQ: {:.4}\n", stats.name, stats.CR_stats[FREQ].correct_preds_num/stats.CR_stats[FREQ].true_preds_num);
  fmt::print(stream, "{} COV: FREQ: {:.4}\n", stats.name, stats.CR_stats[FREQ].correct_preds_num/stats.crit_stats[CRIT].cnt);

  fmt::print(stream, "\n");

  fmt::print(stream, "\nSH: -------------- SIM STATS -----------------\n");
  double total_retired = 0;
  for (int i = 0; i < 9; i++) {
    total_retired += stats.retire_cnt_distr[i];
  }
  fmt::print(stream, "{} Retire Cnt: 0:{:.4}, 1:{:.4}, 2:{:.4}, 3:{:.4}, 4:{:.4}, 5:{:.4}, 6:{:.4}, 7:{:.4}, 8:{:.4},\n", stats.name, stats.retire_cnt_distr[0]/total_retired, stats.retire_cnt_distr[1]/total_retired, stats.retire_cnt_distr[2]/total_retired, stats.retire_cnt_distr[3]/total_retired, stats.retire_cnt_distr[4]/total_retired, stats.retire_cnt_distr[5]/total_retired, stats.retire_cnt_distr[6]/total_retired, stats.retire_cnt_distr[7]/total_retired, stats.retire_cnt_distr[8]/total_retired);
  fmt::print(stream, "{} Crit_type- L1: {:.4}, L2: {:.4}, LLC: {:.4}, DRAM: {:.4}, St: {:.4}, Br: {:.4}, Ex: {:.4} \n", stats.name, stats.crit_type[0][L1]/stats.crit_cycles_thr[0], stats.crit_type[0][L2]/stats.crit_cycles_thr[0], stats.crit_type[0][LLC]/stats.crit_cycles_thr[0], stats.crit_type[0][DRAM]/stats.crit_cycles_thr[0], stats.crit_type[0][ST]/stats.crit_cycles_thr[0], stats.crit_type[0][BR]/stats.crit_cycles_thr[0], stats.crit_type[0][EX]/stats.crit_cycles_thr[0]);
  fmt::print(stream, "{} Crit_type1- L1: {:.4}, L2: {:.4}, LLC: {:.4}, DRAM: {:.4}, St: {:.4}, Br: {:.4}, Ex: {:.4} \n", stats.name, stats.crit_type[1][L1]/stats.crit_cycles_thr[1], stats.crit_type[1][L2]/stats.crit_cycles_thr[1], stats.crit_type[1][LLC]/stats.crit_cycles_thr[1], stats.crit_type[1][DRAM]/stats.crit_cycles_thr[1], stats.crit_type[1][ST]/stats.crit_cycles_thr[1], stats.crit_type[1][BR]/stats.crit_cycles_thr[1], stats.crit_type[1][EX]/stats.crit_cycles_thr[1]);
  fmt::print("Crit Cycles per thread: 0: {}, 1: {}\n", stats.crit_cycles_thr[0], stats.crit_cycles_thr[1]);
  fmt::print("Addr Stats: Crit- L1: {:.4}, L2: {:.4}, LLC: {:.4}, DRAM: {:.4}, Pg_fault: {:.4}\n", stats.crit_stats[CRIT].addr_status[L1]/stats.crit_stats[CRIT].cnt, stats.crit_stats[CRIT].addr_status[L2]/stats.crit_stats[CRIT].cnt, stats.crit_stats[CRIT].addr_status[L3]/stats.crit_stats[CRIT].cnt, stats.crit_stats[CRIT].addr_status[DRAM_]/stats.crit_stats[CRIT].cnt, stats.crit_stats[CRIT].addr_status[PG_FAULT]/stats.crit_stats[CRIT].cnt);
  fmt::print("Addr Stats: NonCrit- L1: {:.4}, L2: {:.4}, LLC: {:.4}, DRAM: {:.4}, Pg_fault: {:.4}\n", stats.crit_stats[NON_CRIT].addr_status[L1]/stats.crit_stats[NON_CRIT].cnt, stats.crit_stats[NON_CRIT].addr_status[L2]/stats.crit_stats[NON_CRIT].cnt, stats.crit_stats[NON_CRIT].addr_status[L3]/stats.crit_stats[NON_CRIT].cnt, stats.crit_stats[NON_CRIT].addr_status[DRAM_]/stats.crit_stats[NON_CRIT].cnt, stats.crit_stats[NON_CRIT].addr_status[PG_FAULT]/stats.crit_stats[NON_CRIT].cnt);
  fmt::print(stream, "\n");

  fmt::print(stream, "\nSH: -------------- MT STATS -----------------\n");
  fmt::print("Sq stalls frac: {:.4}, tot: {}\n", stats.sq_stalls/stats.sq_stalls_den, stats.sq_stalls_den);
  fmt::print("Fetch stalls frac: {:.4}, tot: {}\n", stats.fetch_stalls_num/stats.fetch_stalls_den, stats.fetch_stalls_den);
  fmt::print("Num RA: {:.4}\n", stats.num_ra_exec/stats.num_ra);
  fmt::print("Total energy: {:.4} J\n", stats.total_energy);
  fmt::print("EDP: {:.4} J-s\n", stats.total_energy*stats.total_cycles/(FREQUENCY*1e9));
  fmt::print("Total Cycles: {}\n", stats.total_cycles);

  fmt::print(stream, "\n");
}

void champsim::plain_printer::print(CACHE::stats_type stats)
{
  constexpr std::array<std::pair<std::string_view, std::size_t>, 5> types{
      {std::pair{"LOAD", champsim::to_underlying(access_type::LOAD)}, std::pair{"RFO", champsim::to_underlying(access_type::RFO)},
       std::pair{"PREFETCH", champsim::to_underlying(access_type::PREFETCH)}, std::pair{"WRITE", champsim::to_underlying(access_type::WRITE)},
       std::pair{"TRANSLATION", champsim::to_underlying(access_type::TRANSLATION)}}};

  for (std::size_t cpu = 0; cpu < NUM_CPUS; ++cpu) {
    uint64_t TOTAL_HIT = 0, TOTAL_MISS = 0;
    for (const auto& type : types) {
      TOTAL_HIT += stats.hits.at(type.second).at(cpu);
      TOTAL_MISS += stats.misses.at(type.second).at(cpu);
    }

    fmt::print(stream, "{} TOTAL        ACCESS: {:10d} HIT: {:10d} MISS: {:10d}\n", stats.name, TOTAL_HIT + TOTAL_MISS, TOTAL_HIT, TOTAL_MISS);
    for (const auto& type : types) {
      fmt::print(stream, "{} {:<12s} ACCESS: {:10d} HIT: {:10d} MISS: {:10d}\n", stats.name, type.first,
                 stats.hits[type.second][cpu] + stats.misses[type.second][cpu], stats.hits[type.second][cpu], stats.misses[type.second][cpu]);
    }

    fmt::print(stream, "{} PREFETCH REQUESTED: {:10} ISSUED: {:10} USEFUL: {:10} USELESS: {:10}\n", stats.name, stats.pf_requested, stats.pf_issued,
               stats.pf_useful, stats.pf_useless);

    fmt::print(stream, "{} AVERAGE MISS LATENCY: {:.4g} cycles\n", stats.name, stats.avg_miss_latency);
  }
}

void champsim::plain_printer::print(DRAM_CHANNEL::stats_type stats)
{
  fmt::print(stream, "\n{} RQ ROW_BUFFER_HIT: {:10}\n  ROW_BUFFER_MISS: {:10}\n", stats.name, stats.RQ_ROW_BUFFER_HIT, stats.RQ_ROW_BUFFER_MISS);
  if (stats.dbus_count_congested > 0)
    fmt::print(stream, " AVG DBUS CONGESTED CYCLE: {:.4g}\n", std::ceil(stats.dbus_cycle_congested) / std::ceil(stats.dbus_count_congested));
  else
    fmt::print(stream, " AVG DBUS CONGESTED CYCLE: -\n");
  fmt::print(stream, "WQ ROW_BUFFER_HIT: {:10}\n  ROW_BUFFER_MISS: {:10}\n  FULL: {:10}\n", stats.name, stats.WQ_ROW_BUFFER_HIT, stats.WQ_ROW_BUFFER_MISS,
             stats.WQ_FULL);
}

void champsim::plain_printer::print(champsim::phase_stats& stats)
{
  fmt::print(stream, "=== {} ===\n", stats.name);

  int i = 0;
  for (auto tn : stats.trace_names)
    fmt::print(stream, "CPU {} runs {}", i++, tn);

  if (NUM_CPUS > 1) {
    fmt::print(stream, "\nTotal Simulation Statistics (not including warmup)\n");

    for (const auto& stat : stats.sim_cpu_stats)
      print(stat);

    for (const auto& stat : stats.sim_cache_stats)
      print(stat);
  }

  fmt::print(stream, "\nRegion of Interest Statistics\n");

  for (const auto& stat : stats.roi_cpu_stats)
    print(stat);

  for (const auto& stat : stats.roi_cache_stats)
    print(stat);

  fmt::print(stream, "\nDRAM Statistics\n");
  for (const auto& stat : stats.roi_dram_stats)
    print(stat);
}

void champsim::plain_printer::print(std::vector<phase_stats>& stats)
{
  for (auto p : stats)
    print(p);
}

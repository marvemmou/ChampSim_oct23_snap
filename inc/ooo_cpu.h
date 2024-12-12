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

#ifdef CHAMPSIM_MODULE
#define SET_ASIDE_CHAMPSIM_MODULE
#undef CHAMPSIM_MODULE
#endif

#ifndef OOO_CPU_H
#define OOO_CPU_H

#include <array>
#include <bitset>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <vector>

#include "predictors.h"
#include "champsim.h"
#include "champsim_constants.h"
#include "channel.h"
#include "instruction.h"
#include "module_impl.h"
#include "operable.h"
#include "util/lru_table.h"
#include <type_traits>
#include "ptw.h"
#include "cache.h"
#include "vmem.h"

#include <fstream>
#include <atomic>
#define FREQUENCY 2   // GHz
#define VOLTAGE 0.9   // V
#define Csw 2   // nf

enum STATUS { INFLIGHT = 1, COMPLETED = 2 };
enum INST_TYPE { L1 = 0, L2 = 1, LLC = 2, DRAM = 3, ST = 4, BR = 5, EX = 6, INST_TYPE_CNT = 7 };

class CACHE;
class CacheBus
{
  using channel_type = champsim::channel;
  using request_type = typename channel_type::request_type;
  using response_type = typename channel_type::response_type;

  channel_type* lower_level;
  uint32_t cpu;

  friend class O3_CPU;

public:
  CacheBus(uint32_t cpu_idx, champsim::channel* ll) : lower_level(ll), cpu(cpu_idx) {}
  bool issue_read(request_type packet);
  bool issue_write(request_type packet);
};

struct cpu_stats {
  std::string name;
  uint64_t begin_instrs = 0, begin_cycles = 0;
  uint64_t end_instrs = 0, end_cycles = 0;
  uint64_t total_rob_occupancy_at_branch_mispredict = 0;

  std::array<long long, 8> total_branch_types = {};
  std::array<long long, 8> branch_type_misses = {};

  uint64_t instrs() const { return end_instrs - begin_instrs; }
  uint64_t cycles() const { return end_cycles - begin_cycles; }

  MISS_PRED_STATS MP_stats;
  double total_loads = 0;

  CR_STATS CR_stats[CRIT_METRIC_COUNT];   // Crit predictor stats
  CRITICALITY_STATS crit_stats[CRIT_TYPE_COUNT] = {};   // Global crit stats
  std::map<uint64_t, per_load_stats> per_load_stat;
  double retire_cnt_distr[9] = {0};
  double crit_cycles_thr0[16] = {0};
  double crit_cycles_thr[16] = {0};
  double crit_type[16][9] = {0};  // ld, st, br, ex
  double sq_stalls = 0;
  double sq_stalls_den = 0;
  double fetch_stalls_num = 0;
  double fetch_stalls_den = 0;
  double num_ra = 0;
  double num_ra_exec = 0;
  
  // Energy Calcs
  double window_cycles = 0;
  double retired_insts_window = 0;
  double total_energy = 0;
  double total_cycles = 0;

  MLP_STATS MLP_stats;
  uint64_t rob_stalls = 0;
  uint64_t num_context_switches = 0;
  uint64_t num_of_useless_instructions = 0;
  uint64_t num_of_useful_instructions = 0;
  uint64_t num_of_runahead_instructions = 0;
};

struct LSQ_ENTRY {
  uint64_t instr_id = 0;
  uint64_t virtual_address = 0;
  uint64_t ip = 0;
  uint64_t event_cycle = std::numeric_limits<uint64_t>::max();
  std::string hit_level = "none";
  uint64_t exec_cycle = 0;
  uint64_t ret_cycle = 0;
  bool is_in_runahead = false;

  std::array<uint8_t, 2> asid = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};
  bool fetch_issued = false;

  uint64_t producer_id = std::numeric_limits<uint64_t>::max();
  std::vector<std::reference_wrapper<std::optional<LSQ_ENTRY>>> lq_depend_on_me{};

  LSQ_ENTRY(uint64_t id, uint64_t addr, uint64_t ip, std::array<uint8_t, 2> asid, bool in_runahead);
  void finish(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end) const;

  uint8_t merged = 0;
  uint8_t data_index = 0;
  uint32_t trace_id = 0;
  bool is_prefetch = 0, has_lq_entry = 0;
  bool is_cxl = 0;

  ooo_model_instr* my_rob_it = NULL;

};

// cpu
class O3_CPU : public champsim::operable
{
public:

  double crit_cnt[CRIT_TYPE_COUNT] = {0};
  static const uint64_t table_size = 512;
  MISS_PRED_ENTRY miss_pred_table[table_size];
  std::map<uint64_t, crit_pred_entry> crit_pred_table;
  uint64_t mlp_pred_table[table_size];
  std::deque<llsr_entry> llsr;
  uint32_t cpu = 0;
  std::vector<std::reference_wrapper<CACHE>> cache_view;
  std::vector<std::reference_wrapper<PageTableWalker>> ptw_view;
  uint64_t cooldown_instrs = 0;
  uint64_t unique_instr_id = 0;

  // cycle
  uint64_t begin_phase_cycle = 0;
  uint64_t begin_phase_instr = 0;
  uint64_t finish_phase_cycle = 0;
  uint64_t finish_phase_instr = 0;
  uint64_t last_heartbeat_cycle = 0;
  uint64_t last_heartbeat_instr = 0;
  uint64_t next_print_instruction = STAT_PRINTING_PERIOD;

  // instruction
  uint64_t num_retired = 0;

  bool show_heartbeat = true;

  using stats_type = cpu_stats;

  stats_type roi_stats{}, sim_stats{};
  
  uint32_t flush_smt;
  bool flush_allowed = 0;
  /* per-thread; set to 1 when a switch condition is met, set to 0 when the instigator is retired
        checked in operate to decide whether to call pipeline_flush*/
  bool do_pipeline_flush[16] = {};
  bool do_context_switch[16] = {};
  uint64_t instr_until_cs = 100;

  std::vector<std::reference_wrapper<CACHE>>  caches;

  bool in_runahead[16] = {};
  bool tmp_in_runahead[16] = {};
  uint64_t runahead_trace_id = 0;
  std::atomic<uint64_t> instrs_in_runahead[16] = {};
  int print_after_crit = 20;

  // per-smt-core; set to the rob_it that caused the switch
  std::deque<ooo_model_instr>::iterator flush_it;

  uint32_t switch_point = 0;
  uint32_t counter = 0;

  typedef enum{
  ON_DETECT_MISS = 0,
  ON_BLOCK_ROB = 1,
  ON_DISPATCH = 2,
  TRACE_MAX_MLP = 3,
  TRACE_GREEDY_MLP = 4,
  ON_FETCH = 5
  } switch_points;

  uint64_t switch_policy = 0;

  typedef enum{
    RR = 0,
    ICOUNT = 1,
    ON_FOUND_MISS = 2,
    ON_PREDICT_MISS = 3,
  } policies;

  uint32_t num_traces;
  uint64_t retire_bubbles = 0, miss_retire_bubbles = 0;
  uint64_t rob_all_blocked_on_ld = 0, rob_min_one_blocked_on_llc = 0, rob_all_blocked_on_llc = 0, rob_blocked_on_st = 0;
  uint64_t rob_blocked_on_other = 0, rob_blocked_on_non_memory = 0, rob_blocked_on_memory = 0, rob_empty = 0, rob_blocked_on_schedule = 0;
  uint64_t num_context_switches[16] = {}, num_flushes[16] = {};
  //VectorCounter flushed_instructions, flush_intervals, empty_rob_intervals;
  uint64_t total_flushed = 0, flushed_on_fetch = 0, flushed_on_decode = 0, flushed_on_dispatch = 0, flushed_on_rob = 0;
  uint64_t flushed_return_loads = 0;
  uint64_t flushed_loads_hits[16] = {}, flushed_loads_misses[16] = {};

  uint64_t total_fetched_instrs = 0;
  uint64_t cycles_no_fetch = 0, cycles_no_retire = 0, cycles_no_fetch_full = 0, cycles_lq_full = 0, cycles_sq_full = 0;
  uint64_t prev_fetch_cycle = 0;

  uint32_t cur_trace_id = 0, next_trace_id = 0;
  uint32_t cur_fetch_id = 0;

  std::unordered_map<uint64_t, mlp_instr> timestamp_map;

  std::deque<ooo_model_instr> instr_to_file_pos, replay[16], trace_replay[16];
  bool context_switch = 0;
  uint64_t flush_fetch_resume = 0;
  // per-trace; set to 1 when a switch condition is met (eg llc miss), set to 0 when the instigator is retired
  bool switch_condition_met[16] = {};
  bool switch_condition_done[16] = {};
  bool flush_completed[16] = {};
  uint64_t last_flush_cycle = 0, last_retire_cycle = 0;

  bool is_cgmt = 0;
  bool record = 0;

  std::ofstream outfile;
  std::ifstream infile[16];
  std::ofstream timestamp_file;

  uint64_t instrs_per_trace[16] = {};

  mlp_instr instigator;
  uint64_t issue_mlp = 0;


  // instruction buffer
  struct dib_shift {
    std::size_t shamt;
    auto operator()(uint64_t val) const { return val >> shamt; }
  };
  using dib_type = champsim::lru_table<uint64_t, dib_shift, dib_shift>;
  dib_type DIB;

  // reorder buffer, load/store queue, register file
  std::deque<ooo_model_instr> IFETCH_BUFFER;
  std::deque<ooo_model_instr> DISPATCH_BUFFER;
  std::deque<ooo_model_instr> DECODE_BUFFER;
  std::deque<ooo_model_instr> ROB;

  std::vector<std::optional<LSQ_ENTRY>> LQ;
  std::deque<LSQ_ENTRY> SQ;

  std::array<std::vector<std::reference_wrapper<ooo_model_instr>>, std::numeric_limits<uint8_t>::max() + 1> reg_producers[16];

  // Constants
  const std::size_t IFETCH_BUFFER_SIZE, DISPATCH_BUFFER_SIZE, DECODE_BUFFER_SIZE, ROB_SIZE, SQ_SIZE;
  const long int FETCH_WIDTH, DECODE_WIDTH, DISPATCH_WIDTH, SCHEDULER_SIZE, EXEC_WIDTH;
  const long int LQ_WIDTH, SQ_WIDTH;
  const long int RETIRE_WIDTH;
  const unsigned BRANCH_MISPREDICT_PENALTY, DISPATCH_LATENCY, DECODE_LATENCY, SCHEDULING_LATENCY, EXEC_LATENCY;
  const long int L1I_BANDWIDTH, L1D_BANDWIDTH;
  const unsigned FLUSH_PENALTY;

  // branch
  uint64_t fetch_resume_cycle = 0;

  const long IN_QUEUE_SIZE = 2 * FETCH_WIDTH;

  // 1 input queue per thread on a core
  std::deque<ooo_model_instr> input_queue[16];

  std::deque<ooo_model_instr> oracle_queue[16];
  std::deque<ooo_model_instr> to_core_queue[16];

  CacheBus L1I_bus, L1D_bus;
  CACHE* l1i;

  void initialize() override final;
  long operate() override final;
  void begin_phase() override final;
  void end_phase(unsigned cpu) override final;

  void initialize_instruction();
  long check_dib();
  long fetch_instruction();
  long promote_to_decode();
  long decode_instruction();
  long dispatch_instruction();
  long schedule_instruction();
  long execute_instruction();
  long operate_lsq();
  long complete_inflight_instruction();
  long handle_memory_return();
  long retire_rob();

  bool do_init_instruction(ooo_model_instr& instr);
  bool do_predict_branch(ooo_model_instr& instr);
  void do_check_dib(ooo_model_instr& instr);
  bool do_fetch_instruction(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end);
  void do_dib_update(const ooo_model_instr& instr);
  void do_scheduling(ooo_model_instr& instr);
  void do_execution(ooo_model_instr& rob_it);
  void do_memory_scheduling(ooo_model_instr& instr);
  void do_complete_execution(ooo_model_instr& instr);
  void do_sq_forward_to_lq(LSQ_ENTRY& sq_entry, LSQ_ENTRY& lq_entry);

  void do_finish_store(const LSQ_ENTRY& sq_entry);
  bool do_complete_store(const LSQ_ENTRY& sq_entry);
  bool execute_load(const LSQ_ENTRY& lq_entry);
  ADDRESS_STATUS get_address_status(uint32_t cpu, uint64_t v_address, uint64_t instr_id=0);
  void pipeline_flush();
  void mark_register_dependencies(ooo_model_instr& instr);

  uint64_t roi_instr() const { return roi_stats.instrs(); }
  uint64_t roi_cycle() const { return roi_stats.cycles(); }
  uint64_t sim_instr() const { return num_retired - begin_phase_instr; }
  uint64_t sim_cycle() const { return current_cycle - sim_stats.begin_cycles; }

  void print_deadlock() override final;

#include "ooo_cpu_module_decl.inc"

  struct module_concept {
    virtual ~module_concept() = default;

    virtual void impl_initialize_branch_predictor() = 0;
    virtual void impl_last_branch_result(uint64_t ip, uint64_t target, uint8_t taken, uint8_t branch_type) = 0;
    virtual uint8_t impl_predict_branch(uint64_t ip) = 0;

    virtual void impl_initialize_btb() = 0;
    virtual void impl_update_btb(uint64_t ip, uint64_t predicted_target, uint8_t taken, uint8_t branch_type) = 0;
    virtual std::pair<uint64_t, uint8_t> impl_btb_prediction(uint64_t ip) = 0;
  };

  template <unsigned long long B_FLAG, unsigned long long T_FLAG>
  struct module_model final : module_concept {
    O3_CPU* intern_;
    explicit module_model(O3_CPU* core) : intern_(core) {}

    void impl_initialize_branch_predictor();
    void impl_last_branch_result(uint64_t ip, uint64_t target, uint8_t taken, uint8_t branch_type);
    uint8_t impl_predict_branch(uint64_t ip);

    void impl_initialize_btb();
    void impl_update_btb(uint64_t ip, uint64_t predicted_target, uint8_t taken, uint8_t branch_type);
    std::pair<uint64_t, uint8_t> impl_btb_prediction(uint64_t ip);
  };

  std::unique_ptr<module_concept> module_pimpl;

  void impl_initialize_branch_predictor() { module_pimpl->impl_initialize_branch_predictor(); }
  void impl_last_branch_result(uint64_t ip, uint64_t target, uint8_t taken, uint8_t branch_type)
  {
    module_pimpl->impl_last_branch_result(ip, target, taken, branch_type);
  }
  uint8_t impl_predict_branch(uint64_t ip) { return module_pimpl->impl_predict_branch(ip); }

  void impl_initialize_btb() { module_pimpl->impl_initialize_btb(); }
  void impl_update_btb(uint64_t ip, uint64_t predicted_target, uint8_t taken, uint8_t branch_type)
  {
    module_pimpl->impl_update_btb(ip, predicted_target, taken, branch_type);
  }
  std::pair<uint64_t, uint8_t> impl_btb_prediction(uint64_t ip) { return module_pimpl->impl_btb_prediction(ip); }

  class builder_conversion_tag
  {
  };
  template <unsigned long long B_FLAG = 0, unsigned long long T_FLAG = 0>
  class Builder
  {
    using self_type = Builder<B_FLAG, T_FLAG>;

    uint32_t m_cpu{};
    double m_freq_scale{};
    std::size_t m_dib_set{};
    std::size_t m_dib_way{};
    std::size_t m_dib_window{};
    std::size_t m_ifetch_buffer_size{};
    std::size_t m_decode_buffer_size{};
    std::size_t m_dispatch_buffer_size{};
    std::size_t m_rob_size{};
    std::size_t m_lq_size{};
    std::size_t m_sq_size{};
    unsigned m_fetch_width{};
    unsigned m_decode_width{};
    unsigned m_dispatch_width{};
    unsigned m_schedule_width{};
    unsigned m_execute_width{};
    unsigned m_lq_width{};
    unsigned m_sq_width{};
    unsigned m_retire_width{};
    unsigned m_mispredict_penalty{};
    unsigned m_flush_penalty{};
    unsigned m_decode_latency{};
    unsigned m_dispatch_latency{};
    unsigned m_schedule_latency{};
    unsigned m_execute_latency{};

    CACHE* m_l1i{};
    long int m_l1i_bw{};
    long int m_l1d_bw{};
    champsim::channel* m_fetch_queues{};
    champsim::channel* m_data_queues{};

    friend class O3_CPU;

    template <unsigned long long OTHER_B, unsigned long long OTHER_T>
    Builder(builder_conversion_tag, const Builder<OTHER_B, OTHER_T>& other)
        : m_cpu(other.m_cpu), m_freq_scale(other.m_freq_scale), m_dib_set(other.m_dib_set), m_dib_way(other.m_dib_way), m_dib_window(other.m_dib_window),
          m_ifetch_buffer_size(other.m_ifetch_buffer_size), m_decode_buffer_size(other.m_decode_buffer_size),
          m_dispatch_buffer_size(other.m_dispatch_buffer_size), m_rob_size(other.m_rob_size), m_lq_size(other.m_lq_size), m_sq_size(other.m_sq_size),
          m_fetch_width(other.m_fetch_width), m_decode_width(other.m_decode_width), m_dispatch_width(other.m_dispatch_width),
          m_schedule_width(other.m_schedule_width), m_execute_width(other.m_execute_width), m_lq_width(other.m_lq_width), m_sq_width(other.m_sq_width),
          m_retire_width(other.m_retire_width), m_mispredict_penalty(other.m_mispredict_penalty), m_flush_penalty(other.m_flush_penalty), m_decode_latency(other.m_decode_latency),
          m_dispatch_latency(other.m_dispatch_latency), m_schedule_latency(other.m_schedule_latency), m_execute_latency(other.m_execute_latency),
          m_l1i(other.m_l1i), m_l1i_bw(other.m_l1i_bw), m_l1d_bw(other.m_l1d_bw), m_fetch_queues(other.m_fetch_queues), m_data_queues(other.m_data_queues)
    {
    }

  public:
    Builder() = default;

    self_type& index(uint32_t cpu_)
    {
      m_cpu = cpu_;
      return *this;
    }
    self_type& frequency(double freq_scale_)
    {
      m_freq_scale = freq_scale_;
      return *this;
    }
    self_type& dib_set(std::size_t dib_set_)
    {
      m_dib_set = dib_set_;
      return *this;
    }
    self_type& dib_way(std::size_t dib_way_)
    {
      m_dib_way = dib_way_;
      return *this;
    }
    self_type& dib_window(std::size_t dib_window_)
    {
      m_dib_window = dib_window_;
      return *this;
    }
    self_type& ifetch_buffer_size(std::size_t ifetch_buffer_size_)
    {
      m_ifetch_buffer_size = ifetch_buffer_size_;
      return *this;
    }
    self_type& decode_buffer_size(std::size_t decode_buffer_size_)
    {
      m_decode_buffer_size = decode_buffer_size_;
      return *this;
    }
    self_type& dispatch_buffer_size(std::size_t dispatch_buffer_size_)
    {
      m_dispatch_buffer_size = dispatch_buffer_size_;
      return *this;
    }
    self_type& rob_size(std::size_t rob_size_)
    {
      m_rob_size = rob_size_;
      return *this;
    }
    self_type& lq_size(std::size_t lq_size_)
    {
      m_lq_size = lq_size_;
      return *this;
    }
    self_type& sq_size(std::size_t sq_size_)
    {
      m_sq_size = sq_size_;
      return *this;
    }
    self_type& fetch_width(unsigned fetch_width_)
    {
      m_fetch_width = fetch_width_;
      return *this;
    }
    self_type& decode_width(unsigned decode_width_)
    {
      m_decode_width = decode_width_;
      return *this;
    }
    self_type& dispatch_width(unsigned dispatch_width_)
    {
      m_dispatch_width = dispatch_width_;
      return *this;
    }
    self_type& schedule_width(unsigned schedule_width_)
    {
      m_schedule_width = schedule_width_;
      return *this;
    }
    self_type& execute_width(unsigned execute_width_)
    {
      m_execute_width = execute_width_;
      return *this;
    }
    self_type& lq_width(unsigned lq_width_)
    {
      m_lq_width = lq_width_;
      return *this;
    }
    self_type& sq_width(unsigned sq_width_)
    {
      m_sq_width = sq_width_;
      return *this;
    }
    self_type& retire_width(unsigned retire_width_)
    {
      m_retire_width = retire_width_;
      return *this;
    }
    self_type& mispredict_penalty(unsigned mispredict_penalty_)
    {
      m_mispredict_penalty = mispredict_penalty_;
      return *this;
    }
    self_type& flush_penalty(unsigned flush_penalty_)
    {
      m_flush_penalty = flush_penalty_;
      return *this;
    }
    self_type& decode_latency(unsigned decode_latency_)
    {
      m_decode_latency = decode_latency_;
      return *this;
    }
    self_type& dispatch_latency(unsigned dispatch_latency_)
    {
      m_dispatch_latency = dispatch_latency_;
      return *this;
    }
    self_type& schedule_latency(unsigned schedule_latency_)
    {
      m_schedule_latency = schedule_latency_;
      return *this;
    }
    self_type& execute_latency(unsigned execute_latency_)
    {
      m_execute_latency = execute_latency_;
      return *this;
    }
    self_type& l1i(CACHE* l1i_)
    {
      m_l1i = l1i_;
      return *this;
    }
    self_type& l1i_bandwidth(long int l1i_bw_)
    {
      m_l1i_bw = l1i_bw_;
      return *this;
    }
    self_type& l1d_bandwidth(long int l1d_bw_)
    {
      m_l1d_bw = l1d_bw_;
      return *this;
    }
    self_type& fetch_queues(champsim::channel* fetch_queues_)
    {
      m_fetch_queues = fetch_queues_;
      return *this;
    }
    self_type& data_queues(champsim::channel* data_queues_)
    {
      m_data_queues = data_queues_;
      return *this;
    }

    template <unsigned long long B>
    Builder<B, T_FLAG> branch_predictor()
    {
      return Builder<B, T_FLAG>{builder_conversion_tag{}, *this};
    }
    template <unsigned long long T>
    Builder<B_FLAG, T> btb()
    {
      return Builder<B_FLAG, T>{builder_conversion_tag{}, *this};
    }
  };

  template <unsigned long long B_FLAG, unsigned long long T_FLAG>
  explicit O3_CPU(Builder<B_FLAG, T_FLAG> b)
      : champsim::operable(b.m_freq_scale), cpu(b.m_cpu), DIB(b.m_dib_set, b.m_dib_way, {champsim::lg2(b.m_dib_window)}, {champsim::lg2(b.m_dib_window)}),
        LQ(b.m_lq_size), IFETCH_BUFFER_SIZE(b.m_ifetch_buffer_size), DISPATCH_BUFFER_SIZE(b.m_dispatch_buffer_size), DECODE_BUFFER_SIZE(b.m_decode_buffer_size),
        ROB_SIZE(b.m_rob_size), SQ_SIZE(b.m_sq_size), FETCH_WIDTH(b.m_fetch_width), DECODE_WIDTH(b.m_decode_width), DISPATCH_WIDTH(b.m_dispatch_width),
        SCHEDULER_SIZE(b.m_schedule_width), EXEC_WIDTH(b.m_execute_width), LQ_WIDTH(b.m_lq_width), SQ_WIDTH(b.m_sq_width), RETIRE_WIDTH(b.m_retire_width),
        BRANCH_MISPREDICT_PENALTY(b.m_mispredict_penalty), FLUSH_PENALTY(b.m_flush_penalty), DISPATCH_LATENCY(b.m_dispatch_latency), DECODE_LATENCY(b.m_decode_latency),
        SCHEDULING_LATENCY(b.m_schedule_latency), EXEC_LATENCY(b.m_execute_latency), L1I_BANDWIDTH(b.m_l1i_bw), L1D_BANDWIDTH(b.m_l1d_bw),
        L1I_bus(b.m_cpu, b.m_fetch_queues), L1D_bus(b.m_cpu, b.m_data_queues), l1i(b.m_l1i), module_pimpl(std::make_unique<module_model<B_FLAG, T_FLAG>>(this))
  {
  }
};

#include "ooo_cpu_module_def.inc"

#endif

#ifdef SET_ASIDE_CHAMPSIM_MODULE
#undef SET_ASIDE_CHAMPSIM_MODULE
#define CHAMPSIM_MODULE
#endif

# Stats Description

The meaning of each new statistic is described in the order it has been printed. Stats whose names are self explanatory are omitted. 

## MLP Pred stats

- MLP_{T/F, P/N}: (As per legacy MLP paper) Based on binary outcome. Predicted non-zero MLP when there is actual MLP is marked correct.
- MLP_DIST_ACC: (As per legacy MLP paper) Prediction is considered correct if predicted MLP distance is more than actual MLP distance. Note: This metric is only relevant when using maximum MLP distance (as per MLP paper) and not for MLP density or true MLP
- MLP Error: Average absolute difference between the predicted MLP distance and actual max MLP distance.
- MLP_PERC_OVERLAP: Fraction of misses at the tail of LLSR having cycle overlap MLP with the head of LLSR
- MLP_MIN_DIST: Min. distance to find MLP
- MLP_MAX_DIST: (As per legacy MLP paper) Maximum MLP distance
- MLP_DENSITY: Number of non-zeros in the LLSR
- MLP_MAX_DIST_CYCLES: Max distance with cycle overlap
- MLP_DENSITY_CYCLES: Density with cycle overlap

## Crit Pred stats

- Top 10 IPs: Top 10 most frequent critical load IPs. Per IP stats are as follows
- Freq: Percentage of all critical occurences
- (Non) Crit Freq: Number of (non) crit occurences
- Crit hit freq: Number of cache hits of an IP that became critical
- Crit Cycles (all avg): Critical cycles/total occurences
- Crit Cycles (Crit avg): Critical cycles/critical occurences
- ACC/COV: FREQ: Testing purposes

## Distributions

- Retire Cnt (n): Fraction of total cycles we committed n instructions
- Crit_type: When no instruction was committed, what type of instruction was blocking head of the ROB
- Crit_type1: Same as above but for thread 1
- Addr Stats: Crit: For instructions that end up being critical, which level of cache hierarchy was their address located during fetch.
- Addr Stats: NonCrit: Same as above but for non-critical loads
- CO: Cycle outcome. What did each commit cycle do?
  - Blocked on CS: Spent in context switching cost (Note: this will only show up if accounting for cost at retire)
  - Blocked on RA: Spent in committing runahead instruction
  - Blocked on some type of instrution
  - Committed n instructions

## MT Stats

- Sq stalls frac: Fraction of cycles fetch was stalled because SQ was full
- Fetch stalls frac: Deprecated
- Num RA: Fraction of runahead instructions which have completed execution when they are pseudo-committed
- EDP: Energy delay product
- Frac parallel exec: Fraction of cycles execution took place from different threads in the same cycle
- RA_window: Average size of runahead window determined by the predicted MLP distance
- Non_RA_window: Average size of normal instructions
- RA_Loads_issued: Fraction of runahead loads that were inflight/completed when runahead instruction is pseudo-committed.
- RA_DRAM_Loads_issued: Of the runahead loads which had their address in DRAM at fetch, how many of these were inflight/completed before getting pseudo-committed.

# Run configurables

- The policy to throw away all runahead instructions at retire is used, regardless of whether they had executed.
- The context switching cost can be incurred in two ways-
  - Flush penalty: Delay flush by these many cycles whenever we context switch. Configured using FLUSH_PENALTY_ macro in ooo_cpu.cc
  - Retire cycle cost: Whenever retire switches threads, we stall the ROB for CS_COST number of cycles. Again macro in ooo_cpu.cc
- Number of runahead instructions: Set using instr_until_cs variable
- Cooldown instructions: Set using cooldown_instrs variable



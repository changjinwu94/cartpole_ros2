# Recordings and metrics

Single-trial recordings are written to `logs/position_energy_v4_test_<timestamp>.json`. A batch also creates `logs/final_push_pairs.XXXXXX/` containing `results.csv`, a terminal transcript per attempt and copies of motion recordings. The repository ignores generated logs and the machine-specific .env.

| Field | Interpretation |
|---|---|
| `fraction` in results.csv | Requested final-push setting for this run |
| `outcome` | How the trial ended, including upright entry, return without entry, or a bounded timeout |
| `closest_upright_deg` | Smallest absolute angle measured while the sensor monitor was active, including samples during stopping |
| `entry_rate_rad_s` | Estimated pole angular velocity at first entry into the upright window. Empty when no entry was recorded. |
| `stop_confirmed` | Whether the runner confirmed stopping after motion |
| `recording` | Name of the motion JSON copied into the batch directory |

For an A/B comparison, locate the `energy_decision` event with reason `reduced_final_push`. Record its energy fraction, current counts, target counts and move fraction. Some trials reach upright before this branch fires. Treat those as full-push trials when assessing the effect of tapering. The summary's legacy `final_move_cap` field belongs to the energy-v4 policy. The decision event gives the fraction applied by final-push.

With the configured geometry, convert a command displacement to metres using
`(target_counts - current_counts) * 0.04 / (3000 * 9.7)`. Use motor feedback for actual displacement. Upright entry, crossing vertical and remaining within ±5° for five seconds are distinct outcomes. This runner records swing-up and stops at entry.

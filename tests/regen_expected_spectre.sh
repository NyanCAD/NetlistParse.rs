#!/usr/bin/env bash
# Regenerate Spectre CST dumps from the *Rust* parser (dump_cst ... spectre).
#
# Historically the Spectre ground truth came from the Julia parser via
# NyanSpectreNetlistParser.jl/tools/dump_spectre_cst.jl, and the differential
# suite byte-matches it. Since the Rust port now leads for the PDK gaps, fixes
# that *change* a construct the corpus already covers (e.g. the structural-`if`
# body) must be re-dumped Rust-first. Do NOT run this over the whole corpus:
# that would replace the remaining Julia ground truth. Pass the specific files
# (paths relative to the repo root or absolute).
#
#   ./tests/regen_expected_spectre.sh tests/corpus/spectre/conditional/multi_statement.scs ...
#
# `.scs` is parsed in Spectre mode; `.cir` starts in SPICE; either may switch
# dialects via `simulator lang=`.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <spectre-corpus-file.scs|.cir> [...]" >&2
  exit 1
fi

for f in "$@"; do
  rel="${f#tests/corpus/spectre/}"
  out="tests/expected/spectre/${rel%.*}.txt"
  cargo run -q -p netlist-syntax --bin dump_cst -- "$f" spectre > "$out"
  echo "regenerated $out"
done

# NetlistParse.rs

A fast, standalone Rust parser for **SPICE** and **Spectre** circuit netlists. It
produces a lossless concrete syntax tree (CST) and exposes it to C, C++, and
Python — so tools can parse netlists without a Julia (or any simulator) runtime.

It is a port of the netlist parser in
[Cadnip.jl](https://github.com/NyanCAD/Cadnip.jl) (`NyanLexers.jl` +
`NyanSpectreNetlistParser.jl`), validated to reproduce that parser's tree
**byte-for-byte** by a differential test suite.

## Highlights

- **Two dialects, one parser** — SPICE and Spectre, including the mid-file
  `simulator lang=` switch that hands off between them.
- **Lossless CST** — the tree tiles the source exactly (`tree.text() == source`),
  preserving trivia (whitespace, comments, line continuations). Built on
  [`rowan`](https://github.com/rust-analyzer/rowan).
- **Graceful error recovery** — malformed lines become `Error`/`Incomplete`
  nodes while the rest of the tree stays intact and lossless.
- **Language bindings** — C ABI (cbindgen), C++ (cxx), and Python (PyO3).

## Coverage

**SPICE** — R/C/L, V/I sources (DC/AC and transient functions), diodes, MOSFETs,
BJTs, JFET/MESFET, behavioral (`B`) and controlled (`E`/`F`/`G`/`H`) sources,
switches, mutual inductors (`K`), transmission lines, subcircuit calls (`X`), and
OSDI devices; dot-commands `.model .param .subckt/.ends .dc .ac .tran .op .ic
.nodeset .global .temp .width .options .include .hdl .lib .data .if .measure
.print`, plus Xyce extensions (`.step .func .global_param`, `Y` devices).

**Spectre** — instances, `model`, `parameters`, `subckt`/`ends` (incl. `inline`
+ binning), analyses, `save`/`ic`/`nodeset`, `global`, the control statements
(`alter`/`altergroup`/`check`/`checklimit`/`info`/`options`/`set`/`shell`/
`paramtest`), `if`/`else if`/`else`, `real` function declarations,
`include`/`ahdl_include`, and arrays.

Both dialects share one precedence-climbing expression parser (ternary, unary,
function calls, `{...}`/`'...'`/`(...)` groups) and context-sensitive lexing.

## Layout

```
crates/
  netlist-syntax/   core (rowan): SPICE + Spectre lexers/parsers, shared
                    SyntaxKind + canonical CST dumper
  netlist-cabi/     C ABI (cbindgen -> include/netlist_parser.h), cdylib + staticlib
  netlist-cxx/      C++ binding (cxx) + an owned POD projection of the Spectre AST
  netlist-py/       Python extension (PyO3 / maturin)
tests/
  corpus/           SPICE (.sp) and Spectre (.scs) input netlists
  expected/         canonical CST dumps captured from the Julia parser (ground truth)
docs/               design & integration plans
```

## Quick start

```rust
use netlist_syntax::{parse_spectre, parse_spectre_with, StartLang};

let tree = parse_spectre(src);                          // Spectre (.scs)
let tree = parse_spectre_with(src, StartLang::Spice);   // SPICE (.cir)
```

`.scs` files open in Spectre and `.cir` in SPICE; either may switch mid-file with
`simulator lang=`.

```bash
cargo test                                             # unit + differential suites
cargo run -p netlist-syntax --bin dump_cst <file.sp>   # print the canonical CST dump
./crates/netlist-cabi/tests/run_c_smoke.sh             # C ABI smoke test

# Python (PyO3, in a venv with maturin):
cd crates/netlist-py && maturin develop && python -m pytest tests/
```

## How it works

A textbook lossless-CST design: a hand-written recursive-descent parser over a
context-sensitive lexer, with a precedence-climbing expression parser — the model
[`rowan`](https://github.com/rust-analyzer/rowan) implements. Each grammar
production is a small closure mirroring its Julia `parse_*` counterpart 1:1, and
every node/token dump label equals its Julia form-struct name — which is what
makes the differential test byte-exact.

| Julia | Rust |
|---|---|
| `EXPR{F}` / `Node{T}` green/red tree | `rowan` green/red tree |
| `Tokens.Kind` `@enum` | `TokenKind` (same order) |
| ~148 `forms.jl` structs | one `SyntaxKind` enum; dump label = form name |
| `Tries.jl` keyword trie | `KeywordTrie` (same unique-prefix completion) |
| `NyanLexers` lexer | `Lexer` over a char cursor (state preserved verbatim) |
| `@trynext` / `Incomplete{T}` recovery | `wrapped()` closes each production ok / `Incomplete` |
| `prec(::Kind)` + `parse_binop` | `prec()` + `parse_binop` (precedence climbing) |

Trivia (whitespace, comments, non-significant newlines, `+`/`\` continuations)
are emitted as `rowan` tokens so the tree tiles the source, but carry trivia
`SyntaxKind`s and are omitted from the canonical dump — matching Julia, where
trivia is absorbed into node offsets and never becomes a node.

### Differential testing

The validation gate: the Rust CST dump must byte-match the Julia parser's dump.
Both emit a canonical preorder form — `<kind> <start>-<end>` per node, content
spans only (trivia excluded), 0-based half-open byte offsets. `tests/expected/`
holds dumps captured from the Julia parser, and `cargo test` checks the Rust
output against them — **no Julia needed at test time**. To refresh ground truth
after a Julia-side change, run `tests/regen_expected.sh` against a Julia checkout
of Cadnip (see the script header).

Coverage (via [`cargo-llvm-cov`](https://github.com/taiki-e/cargo-llvm-cov)) is
~90% regions / ~94% lines of `netlist-syntax`. The remainder is intentionally
unreachable from valid input: Verilog-A/Spectre operators the SPICE `prec()`
rejects, non-ngspice dialect paths, and dead `dump_label` arms.

## Roadmap

- **Semantic layer** — number/unit evaluation, name/scope resolution, `.include`,
  and subckt/model/param binding, producing a *resolved* netlist (what most
  C++/Python consumers want; the CST alone already serves editors, linters, and
  formatters).
- **VACASK integration** — feeding parsed netlists into the VACASK simulator
  through the `netlist-cxx` projection. See [`docs/`](docs/).

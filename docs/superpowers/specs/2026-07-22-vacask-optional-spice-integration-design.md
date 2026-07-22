# Optional SPICE/Spectre support in the `vacask` binary

**Date:** 2026-07-22
**Status:** Design approved, ready for implementation plan
**Repos touched:** `VACASK` (build + wiring, most of the work), `NetlistParse.rs` (release tagging so it is fetchable)

## Problem

The Rust parser → `ParserTables` adapter (`VACASK/lib/netlistrs.cpp`:
`buildParserTablesFromFile` / `mergeNetlist`) is wired only into the demo
(`VACASK/demo/api/demo_netlistrs.cpp`). The production `vacask` binary
(`VACASK/simulator/main.cpp`) parses exclusively through the native bison/flex
grammar (`Parser::parseNetlistFile`). Consequences:

- The binary **cannot ingest SPICE `.cir` or Spectre `.scs`** today. Every
  SPICE capability already built and tested (passives/sources/D/M/Q, AGAUSS,
  `.lib` sections, BSIM binning) is reachable only through the demo.
- The Rust toolchain is **mandatory** to build VACASK at all: the top-level
  `CMakeLists.txt` `FATAL_ERROR`s if `NETLIST_RS_DIR` (a hardcoded, now-stale
  relative path `../Cadnip.jl/netlist-parser-rs`) is missing — yet the object
  code (`netlistrs.o`) is only linked into the demo. The main binary links
  today purely because the linker drops `netlistrs.o` as unreferenced.

We want two things: (1) make the parser an **optional** dependency toggled by a
single, trivially-flippable CMake switch, and (2) have a normal build **fetch
the dependency automatically** — no manual `-DNETLIST_RS_DIR=` path, no env
flags — when the switch is on.

## Approach chosen: foreign-as-include (Approach B), auto-fetched, source-built

Top-level input stays native VACASK. The `include` directive learns to
format-dispatch: an included SPICE/Spectre file routes through the Rust adapter
and merges its models/subckts/devices into the live `ParserTables`; commands in
the foreign file are ignored (with a warning). The user writes `analysis`,
`options`, `save`, `load` in native VACASK. This matches the guiding principle:
**translate the PDK, not the testbench.**

Rejected alternatives (recorded for context):
- *SPICE as top-level format + dot-command translation* — high-cost,
  low-payoff; the buggy dialect-specific surface is exactly the part we avoid.
- *Publish to crates.io + consume from registry* — crates.io is a **source**
  registry; consuming it still requires cargo to compile, and it makes the
  cxx-bridge codegen (which needs `lib.rs` on disk) *more* awkward than a git
  clone, not less. No win for the "just works" goal.
- *Prebuilt C-ABI binaries (via `netlist-cabi` + cbindgen)* — the only path
  that lets a consumer build VACASK with **no** Rust toolchain, but it puts us
  in the business of producing/hosting per-platform binaries. VACASK is already
  a heavy from-source build (bison/flex/openvaf/boost/suitesparse) that already
  requires cargo, so source-built is the pragmatic fit.

## Design

### 1. Build: optional dependency via auto-fetch

A single option gates everything (top-level `VACASK/CMakeLists.txt`):

```cmake
option(VACASK_WITH_SPICE
    "Build with SPICE/Spectre netlist support (requires a Rust toolchain)" ON)
```

Default `ON` (opt-out). This is one line and is deliberately trivial for a
maintainer to flip; the hard requirement is only that the feature is *optional*.

When `ON`:
- `find_program(CARGO cargo)`; if not found, `FATAL_ERROR` with the hint:
  *"A Rust toolchain (cargo) is required for SPICE/Spectre support. Install
  Rust, or pass `-DVACASK_WITH_SPICE=OFF` to build without it."*
- Fetch the parser source automatically:
  ```cmake
  FetchContent_Declare(NetlistParseRs
      GIT_REPOSITORY https://github.com/NyanCAD/NetlistParse.rs.git
      GIT_TAG <pinned-tag>)          # version bump = edit this one line
  FetchContent_MakeAvailable(NetlistParseRs)
  ```
  then point Corrosion at the fetched tree:
  ```cmake
  corrosion_import_crate(MANIFEST_PATH
      "${netlistparsers_SOURCE_DIR}/crates/netlist-cxx/Cargo.toml")
  corrosion_add_cxxbridge(netlist_cxx_bridge CRATE netlist_cxx FILES lib.rs)
  ```
- **Remove** the stale `NETLIST_RS_DIR` default. Keep it as an optional local
  override: if `-DNETLIST_RS_DIR=<path>` is passed, use that path instead of
  fetching (preserves the maintainer's edit-parser-and-rebuild dev loop).

When `OFF`: the whole `FetchContent` + Corrosion + `corrosion_add_cxxbridge`
block is skipped. No cargo required; the `Corrosion` FetchContent for the CMake
module itself is also skipped.

### 2. Compile & link guards

- `VACASK/lib/CMakeLists.txt`: `netlistrs.cpp` (and `include/netlistrs.h`) are
  added to the `simlib` source list **only when `VACASK_WITH_SPICE` is ON**.
  When on:
  ```cmake
  target_compile_definitions(simlib PUBLIC VACASK_WITH_SPICE)
  target_link_libraries(simlib PUBLIC netlist_cxx_bridge)
  ```
  `PUBLIC` propagation means both `vacask` (`simulator/`) and the demo inherit
  the bridge automatically — no per-target link edits needed.
- The include-dispatch hook in `dfllexer.l` (§3) is wrapped in
  `#ifdef VACASK_WITH_SPICE`.
- When `OFF`, `netlistrs.o` is not built, so the `netlist::` cxx symbols are
  never referenced and the binary links with no Rust artifacts at all.

### 3. Main-binary wiring — the include-dispatch seam

Interception point: the two include-resolution end-states in
`VACASK/lib/dfllexer.l`:
- `<INCEND>\n` (~line 303): `include "file"` without a section.
- `<LIBEND>\n` (~line 371): `include "file" section=name`.

Both currently resolve a canonical path via `tables.fileStack().addFile(...)`
then `yypush_buffer_state(...)` to lex the included file natively. New behavior,
after `fname` is resolved and before the native buffer push:

- If `fname`'s (lowercased) extension is in the **foreign set** → call a new
  helper `mergeForeignFile(fname, section, tables, p, s)` that parses the file
  through the Rust adapter and `mergeNetlist`s its models/subckts/devices into
  the **existing** live table, then **continue scanning the parent file** (no
  buffer push, no new lexer state). On adapter error, emit `error(*loc, ...)`
  and return `token::YYerror`.
- Otherwise (`.sim`, or unknown) → existing native buffer push, unchanged.
- Under `#else` (SPICE compiled out): a foreign extension →
  `error(*loc, "include of a SPICE/Spectre file requires a build with "
  "-DVACASK_WITH_SPICE=ON")` and return `token::YYerror`.

**Foreign extension set:**
- SPICE start language: `.cir`, `.sp`, `.spice`, `.mod`, `.lib`
- Spectre start language: `.scs`, `.spectre`

This reuses the extension→language logic already present in
`buildParserTablesFromFile`.

**Why a new `mergeForeignFile` helper instead of calling
`buildParserTablesFromFile`:** the latter is a *top-level* entry — it calls
`tab.defaultGround()` and `tab.setDefaultSubDef(...)`, i.e. it (re)initializes a
fresh table. An include must merge into the *existing* default subcircuit
definition. `mergeForeignFile` factors out just the parse + `mergeNetlist`
into the live table's current top. Nested includes *inside* the foreign file
continue to be handled by `mergeNetlist`'s existing recursion (it already
routes `.lib` sections via `parse_netlist_lib` and other files via
`parse_netlist`).

**Parser handle:** `mergeNetlist` needs a `Parser&` for expression parsing.
The scanner has `tables` (the `ParserTables&`) but not necessarily a `Parser`.
`mergeForeignFile` constructs a transient `Parser p(tables)` for the merge.
(Implementation detail to verify during TDD — confirm a transient Parser is
sufficient and does not disturb the in-flight native parse.)

**Commands inside foreign includes:** ignored, but **warn** — surface a stray
`.tran`/`.dc`/`.op`/etc. on stderr (do not silently swallow). The warning names
the file and the ignored directive.

### 4. OSDI auto-load

Today the demo **hardcodes** the `load "...osdi"` list
(`demo_netlistrs.cpp:34-43`). For the real binary, the adapter auto-emits a
de-duplicated set of top-level `load` cards (`PTLoad` into `tab`) for exactly
the OSDI masters it references while merging. The adapter already computes
`model type+level → master` via `spiceModelMaster`; add a `master → osdi-file`
table and, whenever a master is first referenced, emit its `load` once.

Seed the `master → osdi-file` table from the demo's current list:

| master        | osdi file             |
|---------------|-----------------------|
| `resistor`    | `resistor.osdi`       |
| `sp_resistor` | `spice/resistor.osdi` |
| `capacitor`   | `capacitor.osdi`      |
| `inductor`    | `inductor.osdi`       |
| `diode`       | `diode.osdi`          |
| `sp_diode`    | `spice/diode.osdi`    |
| `sp_bsim4v8`  | `spice/bsim4v8.osdi`  |
| `bsim3`       | `bsim3v3.osdi`        |
| `bsim4`       | `bsim4v8.osdi`        |
| `vbic13`      | `vbic_1p3.osdi`       |

(Extend as further masters gain SPICE coverage: `bsimbulk`→`bsimbulk106.osdi`,
`psp103va`→`psp103v4.osdi`, etc.) De-dup so including a PDK that uses a master
many times emits a single `load`. Builtin masters (`vsource`, `isource`) emit
nothing. Result: including a PDK auto-pulls exactly the OSDI it needs, and the
demo's hardcoded block is deleted.

Open sub-item for the plan: confirm `load` is toplevel-only and that emitting it
mid-merge (from within an `include`) reaches the toplevel table correctly, per
`docs/cir-loading.md` / `dflparser.y` `PTLoad`.

### 5. Testing

- **E2E through the real `vacask` binary** (not the demo): a native `.sim`
  testbench that `include`s a SPICE PDK file, run through `vacask`, asserting
  op/tran results. Mirror the existing demo SPICE cases (RC, diode, subckt,
  binned MOS, VCVS, CCCS) but driven end-to-end by the binary. These prove the
  include seam + OSDI auto-load + solve path.
- **`section=` case:** `include "corners.lib" section=tt` through the binary.
- **Optionality build test:** configure with `-DVACASK_WITH_SPICE=OFF` in an
  environment with **no cargo**; assert it configures, builds, and links; and
  that a foreign `include` produces the clean "rebuild with
  `-DVACASK_WITH_SPICE=ON`" error rather than a crash or a native syntax error.
- **Command-warning case:** a foreign include containing a `.tran` emits the
  warning and is otherwise ignored.
- CI: the `ON` path needs a Rust toolchain (already effectively present); add an
  `OFF`-path job that runs without cargo to guard the optional build.

## Out of scope

- Command translation (Approach A) — not doing it.
- Prebuilt/binary distribution of the parser (C-ABI route) — source-built only.
- Spectre-specific feature expansion beyond what the parser already supports
  (Spectre include dispatch comes for free under this design; no new Spectre
  parsing work is implied).

## Work targeting / git

Per the established workflow, VACASK-side changes go on the single
`rust-parser-integration` branch on the `fork` remote
(`ssh://codeberg.org/pepijndevos/VACASK.git`, PR #87) — not a new per-feature
branch. `NetlistParse.rs` needs only a release **tag** for `FetchContent` to
pin (`GIT_TAG`); no source changes to the crate are required by this design.
```

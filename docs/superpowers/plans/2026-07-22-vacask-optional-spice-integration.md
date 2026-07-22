# Optional SPICE/Spectre support in the `vacask` binary — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Rust SPICE/Spectre parser an optional, auto-fetched dependency and wire it into the production `vacask` binary via format-dispatching `include`.

**Architecture:** A single CMake option (`VACASK_WITH_SPICE`, default ON) gates a `FetchContent` clone of NetlistParse.rs consumed through Corrosion. When on, the flex scanner's `include` handler routes foreign-format files (`.cir/.sp/.spice/.mod/.lib/.scs/.spectre`) through the Rust→`ParserTables` adapter, merging models/subckts/devices into the live table and auto-emitting the OSDI `load`s those devices need; commands in the foreign file are ignored with a warning. When off, no Rust toolchain is required and a foreign include yields a clean error.

**Tech Stack:** CMake 3.18+, Corrosion 0.5.1, cxx bridge, flex/bison, C++20, Rust (cargo), CTest.

## Global Constraints

- VACASK-side changes go on branch `rust-parser-integration` on remote `fork` (`ssh://codeberg.org/pepijndevos/VACASK.git`), feeding PR #87. No new per-feature branches.
- `NetlistParse.rs` requires **no source change** — only a git **tag** for `FetchContent` to pin.
- Default `VACASK_WITH_SPICE=ON`; the default line must be a single trivially-editable `option(...)`.
- Foreign extension set: SPICE = `.cir .sp .spice .mod .lib`; Spectre = `.scs .spectre`. Native = `.sim` (and unknown → native).
- Adapter comment invariant today: "PTLoads are NOT added here." This plan intentionally changes that; update the comments.
- Verification uses the existing configured build: `cmake --build build` (Ninja) and `ctest --test-dir build`.

---

### Task 1: Release tag on NetlistParse.rs for FetchContent to pin

**Files:**
- No file edits. Create an annotated git tag in `/home/pepijn/code/nyanodide/NetlistParse.rs`.

**Interfaces:**
- Produces: a tag name (e.g. `v0.1.0`) used as `GIT_TAG` in Task 2.

- [ ] **Step 1:** Confirm the crate builds clean: `cargo build -p netlist-cxx` in `NetlistParse.rs`. Expected: success.
- [ ] **Step 2:** Create the tag on the current parser HEAD:
  `git -C /home/pepijn/code/nyanodide/NetlistParse.rs tag -a v0.1.0 -m "netlist parser v0.1.0 (vacask FetchContent pin)"`
- [ ] **Step 3:** Push the tag to origin (github NyanCAD/NetlistParse.rs):
  `git -C .../NetlistParse.rs push origin v0.1.0`
  Expected: tag appears on the remote so a fresh VACASK checkout can fetch it.

> NOTE: until the tag is pushed and reachable, Task 2 uses `-DNETLIST_RS_DIR=` (local override) for verification; the `GIT_TAG` path is exercised only from a clean checkout with network access.

---

### Task 2: Adapter — OSDI auto-load + `mergeForeignFile` helper

**Files:**
- Modify: `VACASK/lib/netlistrs.cpp`
- Modify: `VACASK/include/netlistrs.h`
- Modify: `VACASK/demo/api/demo_netlistrs.cpp` (drop the hardcoded PTLoad block — becomes the regression test)
- Test vehicle: existing demo CTests (`demo_netlistrs_spice_*`).

**Interfaces:**
- Produces:
  - `void emitOsdiLoads(ParserTables& tab, const std::set<std::string>& masters);` — maps each master to its `.osdi` file and adds a `PTLoad` for it, skipping builtins and de-duplicating against loads already present in `tab`.
  - `bool mergeForeignFile(const std::string& path, const std::string& section, PTSubcircuitDefinition& top, ParserTables& tab, Parser& p, Status& s);` — parse `path` (SPICE/Spectre by extension), `mergeNetlist` its models/subckts/devices into the caller-provided `top` (the grammar's in-progress toplevel def, NOT `tab.defaultSubDef()`), **suppress** analysis/command projection (warn if any present), then `emitOsdiLoads` for referenced masters into `tab`. `section` empty = whole file; non-empty = `.lib`-style section. Does NOT call `defaultGround()`/`setDefaultSubDef()`.
  - `mergeNetlist(...)` gains a trailing out-param `std::set<std::string>& masters` (populated with every OSDI master it references) and a bool `projectAnalyses` (default `true`).

- [ ] **Step 1 (failing test):** Edit `demo_netlistrs.cpp` — delete the hardcoded `tab.add(PTLoad(...))...` block (lines 34-43). Leave the `buildParserTablesFromFile` call. This makes the demo depend on auto-emit.
- [ ] **Step 2 (verify fail):** `cmake --build build && ctest --test-dir build -R demo_netlistrs_spice_rc -V`. Expected: FAIL (no OSDI loaded → elaboration/verify error), proving auto-load is required.
- [ ] **Step 3 (master→osdi table + emitOsdiLoads):** In `netlistrs.cpp`, add near the top:

```cpp
// SPICE-flavored master name -> OSDI module file. Keep in sync with the
// devices shipped under the module path. Builtins (vsource/isource) omitted.
static const std::map<std::string, std::string>& osdiFileForMaster() {
    static const std::map<std::string, std::string> t = {
        {"resistor",   "resistor.osdi"},       {"sp_resistor", "spice/resistor.osdi"},
        {"capacitor",  "capacitor.osdi"},      {"inductor",    "inductor.osdi"},
        {"diode",      "diode.osdi"},          {"sp_diode",    "spice/diode.osdi"},
        {"sp_bsim4v8", "spice/bsim4v8.osdi"},  {"bsim3",       "bsim3v3.osdi"},
        {"bsim4",      "bsim4v8.osdi"},        {"vbic13",      "vbic_1p3.osdi"},
        {"bsimbulk",   "bsimbulk106.osdi"},    {"psp103va",    "psp103v4.osdi"},
    };
    return t;
}

void emitOsdiLoads(ParserTables& tab, const std::set<std::string>& masters) {
    std::set<std::string> emitted; // dedup within this call
    for (const auto& m : masters) {
        auto it = osdiFileForMaster().find(m);
        if (it == osdiFileForMaster().end()) continue; // builtin or unknown
        if (!emitted.insert(it->second).second) continue;
        if (tab.hasLoad(it->second)) continue; // dedup vs already present
        tab.add(PTLoad(it->second));
    }
}
```

> If `ParserTables` has no `hasLoad`, add a small predicate there, or track emitted loads in a `static`-free set threaded from the caller. Verify the exact `PTLoad`/`ParserTables::add` API during implementation (see `parseroutput.h`).

- [ ] **Step 4 (thread masters through mergeNetlist):** Add `std::set<std::string>& masters` and `bool projectAnalyses = true` params to both `mergeNetlist` declarations/definitions. Wherever a device/model master is determined (the `spiceModelMaster(...)` results and passive/semiconductor master assignments), `masters.insert(master)`. Guard the analyses loop (line ~892): `if (projectAnalyses) { ...existing... } else if (!nl.analyses.empty()) { Simulator::err() << "warning: ignoring " << nl.analyses.size() << " analysis command(s) from included file '" << <file> << "'\n"; }`. Thread the new args through the existing recursive calls (lines ~855, ~955) and the top-level callers (`buildParserTables`, `buildParserTablesFromFile`).
- [ ] **Step 5 (emit in top-level callers):** At the end of `buildParserTables` and `buildParserTablesFromFile`, after a successful merge, call `emitOsdiLoads(tab, masters)` with the accumulated set (both create a local `std::set<std::string> masters;` and pass it in).
- [ ] **Step 6 (mergeForeignFile):** Add the new function:

```cpp
bool mergeForeignFile(const std::string& path, const std::string& section,
                      PTSubcircuitDefinition& top, ParserTables& tab,
                      Parser& p, Status& s) {
    namespace fs = std::filesystem;
    fs::path fp(path);
    std::string ext = fp.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    bool spice = (ext != ".scs" && ext != ".spectre");
    std::ifstream in(path);
    if (!in) { s.set(Status::NotFound, "cannot open: " + path); return false; }
    std::stringstream ss; ss << in.rdbuf();
    std::string source = ss.str();
    netlist::Netlist nl = section.empty()
        ? netlist::parse_netlist(rust::Str(source), spice)
        : netlist::parse_netlist_lib(rust::Str(source), rust::Str(section));
    if (!nl.errors.empty()) {
        std::ostringstream os;
        os << "netlist parse error(s) in '" << path << "': " << nl.errors.size();
        s.set(Status::Syntax, os.str()); return false;
    }
    fs::path absPath; try { absPath = fs::canonical(fp); } catch (...) { absPath = fs::absolute(fp); }
    std::set<fs::path> visited{ absPath };
    std::set<std::string> addedModels, masters;
    // Merge into the caller's toplevel def (the grammar's $2.def), NOT
    // tab.defaultSubDef() — that is overwritten by the parser at end-of-parse.
    if (!mergeNetlist(nl, top, tab, p, absPath.parent_path(), visited, addedModels, masters,
                      /*projectAnalyses=*/false, s))
        return false;
    emitOsdiLoads(tab, masters);
    return true;
}
```

> `ParserTables::defaultSubDef()` returns a mutable ref (`parseroutput.h:747`) but is NOT the merge target — see Task 4. `top` is supplied by the grammar drain action.

- [ ] **Step 7 (header):** In `netlistrs.h`, declare `emitOsdiLoads` and `mergeForeignFile`; update the stale "PTLoads are NOT added here" comments to note auto-emission.
- [ ] **Step 8 (verify pass):** `cmake --build build && ctest --test-dir build -R demo_netlistrs_spice -V`. Expected: all `demo_netlistrs_spice_*` PASS with the hardcoded loads gone (auto-emit replaced them).
- [ ] **Step 9 (commit):** stage `lib/netlistrs.cpp include/netlistrs.h demo/api/demo_netlistrs.cpp` → `feat(netlist): auto-emit OSDI loads and add mergeForeignFile for include dispatch`.

---

### Task 3: CMake — `VACASK_WITH_SPICE` option, FetchContent, guards

**Files:**
- Modify: `VACASK/CMakeLists.txt` (lines 16-35 region)
- Modify: `VACASK/lib/CMakeLists.txt` (simlib source list + link)

**Interfaces:**
- Produces: compile definition `VACASK_WITH_SPICE` on `simlib` (PUBLIC) and the `netlist_cxx_bridge` target, both only when the option is ON.

- [ ] **Step 1:** Replace the current Corrosion/NETLIST_RS block (CMakeLists.txt:16-35) with:

```cmake
# --- Rust netlist parser (optional; netlist-cxx via Corrosion) --------------
option(VACASK_WITH_SPICE "Build with SPICE/Spectre netlist support (needs Rust/cargo)" ON)
if(VACASK_WITH_SPICE)
    find_program(CARGO_EXECUTABLE cargo)
    if(NOT CARGO_EXECUTABLE)
        message(FATAL_ERROR "VACASK_WITH_SPICE=ON needs a Rust toolchain (cargo) on PATH. "
                            "Install Rust or configure with -DVACASK_WITH_SPICE=OFF.")
    endif()
    include(FetchContent)
    FetchContent_Declare(Corrosion
        GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git GIT_TAG v0.5.1)
    FetchContent_MakeAvailable(Corrosion)

    # Local override for parser development; otherwise fetch a pinned release.
    if(NETLIST_RS_DIR)
        set(_netlistrs_src "${NETLIST_RS_DIR}")
    else()
        FetchContent_Declare(NetlistParseRs
            GIT_REPOSITORY https://github.com/NyanCAD/NetlistParse.rs.git
            GIT_TAG v0.1.0)
        FetchContent_MakeAvailable(NetlistParseRs)
        set(_netlistrs_src "${netlistparsers_SOURCE_DIR}")
    endif()
    if(NOT EXISTS "${_netlistrs_src}/crates/netlist-cxx/Cargo.toml")
        message(FATAL_ERROR "netlist-cxx not found under ${_netlistrs_src}")
    endif()
    corrosion_import_crate(MANIFEST_PATH "${_netlistrs_src}/crates/netlist-cxx/Cargo.toml")
    corrosion_add_cxxbridge(netlist_cxx_bridge CRATE netlist_cxx FILES lib.rs)
endif()
```

- [ ] **Step 2:** In `lib/CMakeLists.txt`, make `netlistrs.cpp` + its header conditional. Find where `netlistrs.cpp` is added to `simlib` sources (grep `netlistrs.cpp`); wrap it and add link/def after the `add_library(simlib ...)`:

```cmake
if(VACASK_WITH_SPICE)
    target_sources(simlib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/netlistrs.cpp)
    target_compile_definitions(simlib PUBLIC VACASK_WITH_SPICE)
    target_link_libraries(simlib PUBLIC netlist_cxx_bridge)
endif()
```
Remove `netlistrs.cpp` (and, if desired, `../include/netlistrs.h`) from the unconditional source list so the OFF build omits it.

- [ ] **Step 3 (verify ON):** `cmake -S . -B build -DNETLIST_RS_DIR=/home/pepijn/code/nyanodide/NetlistParse.rs && cmake --build build`. Expected: configures + builds; `build/simulator/vacask` present.
- [ ] **Step 4 (verify OFF):** `cmake -S . -B build-nospice -DVACASK_WITH_SPICE=OFF -DOPENVAF_DIR=/home/pepijn/code/nyanodide/OpenVAF/target/release/ && cmake --build build-nospice`. Expected: configures with NO cargo invocation and builds `vacask` with no Rust artifacts. (Confirm via `grep -i cargo build-nospice/CMakeCache.txt` → no NetlistParseRs/corrosion entries.)
- [ ] **Step 5 (commit):** `feat(build): optional VACASK_WITH_SPICE with FetchContent auto-fetch of the parser`.

---

### Task 4: Scanner stash + grammar drain of foreign includes

**Files:**
- Modify: `VACASK/include/parseroutput.h` (add pending-foreign staging to `ParserTables`)
- Modify: `VACASK/lib/dfllexer.l` (`<INCEND>\n` ~303, `<LIBEND>\n` ~371 — stash, skip native push)
- Modify: `VACASK/lib/dflparser.y` (`output : INNETLIST subckt_build END` ~258 — drain)

**Interfaces:**
- Consumes: `sim::mergeForeignFile` (Task 2), `VACASK_WITH_SPICE` define (Task 3).
- Produces on `ParserTables`:
  - `struct PendingForeign { std::string path; std::string section; };`
  - `ParserTables& addPendingForeign(std::string path, std::string section) &;`
  - `std::vector<PendingForeign>& pendingForeign();`

- [ ] **Step 1 (staging on ParserTables):** In `parseroutput.h`, inside `class ParserTables`, add (near `loads_`):

```cpp
    struct PendingForeign { std::string path; std::string section; };
    ParserTables& addPendingForeign(std::string path, std::string section) & {
        pendingForeign_.push_back({std::move(path), std::move(section)}); return *this; }
    std::vector<PendingForeign>& pendingForeign() { return pendingForeign_; }
    // ... in the private members section:
    std::vector<PendingForeign> pendingForeign_;
```

- [ ] **Step 2 (foreign-ext predicate + guarded include in lexer prologue):** In the `%{ ... %}` C prologue of `dfllexer.l`, ensure `<algorithm>`/`<string>` are included, then add under the guard:

```cpp
#ifdef VACASK_WITH_SPICE
static bool isForeignNetlistExt(const std::string& path) {
    auto dot = path.find_last_of('.');
    if (dot == std::string::npos) return false;
    std::string e = path.substr(dot);
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e==".cir"||e==".sp"||e==".spice"||e==".mod"||e==".lib"
         ||e==".scs"||e==".spectre";
}
#endif
```

- [ ] **Step 3 (INCEND — no section):** In `<INCEND>\n`, BEFORE `fileStack().addFile(...)`/`pushStream`/`yypush_buffer_state`, restructure so foreign files stash and skip the native push. The filename is `sbuf` (resolution to canonical happens via `addFile`; for the stash we resolve relative to the current file dir / include path — reuse `Simulator::includePath()` semantics or store the raw `sbuf` and let `mergeForeignFile` resolve). Simplest correct form: still call `addFile` to get the canonical path, then branch:

```cpp
                    auto& fname = tables.fileStack().canonicalName(fileStackPosition);
#ifdef VACASK_WITH_SPICE
                    if (isForeignNetlistExt(fname)) {
                        tables.addPendingForeign(fname, "");
                        // do NOT push a buffer; keep scanning the parent file
                    } else
#else
                    if (isForeignNetlistExt(fname)) {
                        error(*loc, std::string("include of SPICE/Spectre file '")+fname+
                              "' requires a build with -DVACASK_WITH_SPICE=ON");
                        return(token::YYerror);
                    } else
#endif
                    {
                        auto streamPtr = pushStream(fname, *loc);
                        if (!streamPtr) { error(*loc, std::string("Failed to open include file '")+fname+"'."); return(token::YYerror); }
                        yypush_buffer_state(yy_create_buffer(*streamPtr, YY_BUF_SIZE));
                        loc->end.initialize(nullptr, 1, 1, 0);
                        loc->end.setFileStack(tables.fileStack(), fileStackPosition);
                        loc->begin = loc->end;
                    }
```

> Preserve the exact existing native-push body (lines ~321-334) inside the `else {}`. The stash branch must still leave the lexer in a sane state to continue the parent (the `yy_pop_state()` calls that precede this action already returned to INC/LINESTART).

- [ ] **Step 4 (LIBEND — with section):** Same restructure in `<LIBEND>\n` (~376), stashing the captured `section`: `tables.addPendingForeign(fname, section);` in the foreign branch; keep the native library-file push (including `setSection`) in the `else {}`.
- [ ] **Step 5 (grammar drain):** In `dflparser.y`, `output : INNETLIST subckt_build END` action (~258), after `$2.def.add(std::move($2.parameters));` and BEFORE `tables.setDefaultSubDef(...)`:

```cpp
#ifdef VACASK_WITH_SPICE
    {
        sim::Parser mp(tables);
        for (auto& fi : tables.pendingForeign()) {
            if (!sim::mergeForeignFile(fi.path, fi.section, $2.def, tables, mp, status)) {
                YYERROR;
            }
        }
        tables.pendingForeign().clear();
    }
#endif
    tables.setDefaultSubDef(std::move($2.def));
```

Add `#include "netlistrs.h"` to the grammar's C prologue under `#ifdef VACASK_WITH_SPICE` (guarded so the OFF build, which omits `netlistrs.h`/its symbols, still compiles).

- [ ] **Step 6 (build):** `cmake --build build`. Expected: flex+bison regenerate, compile+link clean (bridge linked via simlib PUBLIC).
- [ ] **Step 7 (commit):** `feat(parser): stash foreign-format includes and drain them into the toplevel def`.

---

### Task 5: End-to-end tests through the `vacask` binary

**Files:**
- Create: `VACASK/test/spice/rc_top.sim` (native testbench that includes a SPICE file)
- Create: `VACASK/test/spice/rc_models.cir` (SPICE models/devices)
- Modify: `VACASK/test/CMakeLists.txt` (register CTest cases, guarded by `VACASK_WITH_SPICE`)

**Interfaces:**
- Consumes: the `vacask` binary + include dispatch (Tasks 2-4).

- [ ] **Step 1 (native testbench):** `rc_top.sim` — a minimal native VACASK deck: `include "rc_models.cir"`, instantiate the RC from the included models, add native `analysis`/`op` or `tran`, `save`, and (if not auto-loaded) nothing else. Mirror an existing `test/*.sim` for exact native syntax.
- [ ] **Step 2 (SPICE include):** `rc_models.cir` — a SPICE deck with an R and C (and a `.model` if useful) matching the demo's `spice_rc.cir`.
- [ ] **Step 3 (register test):**

```cmake
if(VACASK_WITH_SPICE)
  add_test(NAME vacask_include_spice_rc
           COMMAND vacask "${CMAKE_CURRENT_SOURCE_DIR}/spice/rc_top.sim")
  set_tests_properties(vacask_include_spice_rc PROPERTIES
      PASS_REGULAR_EXPRESSION "<expected node voltage / analysis marker>")
endif()
```

- [ ] **Step 4 (section case):** Add a `.lib` file with a `section=tt` and a `rc_top_section.sim` that does `include "corners.lib" section=tt`; register `vacask_include_spice_section`.
- [ ] **Step 5 (command-warning case):** A `.cir` containing a stray `.tran`; assert the run warns and still succeeds (`PASS_REGULAR_EXPRESSION "ignoring .* command"`).
- [ ] **Step 6 (run):** `ctest --test-dir build -R vacask_include -V`. Expected: all PASS.
- [ ] **Step 7 (commit):** `test(parser): e2e include of SPICE/Spectre through the vacask binary`.

---

### Task 6: OFF-build regression gate + docs

**Files:**
- Modify: `VACASK/docs/input-include.md` (document foreign-format includes + the flag)
- Optionally: CI config for an `-DVACASK_WITH_SPICE=OFF` job.

- [ ] **Step 1:** Document in `docs/input-include.md`: foreign extensions dispatch to the SPICE/Spectre parser; commands in them are ignored (warned); requires `VACASK_WITH_SPICE=ON`.
- [ ] **Step 2:** Verify the OFF build one more time (Task 3 Step 4) and that a foreign include errors cleanly there (add a small negative test or a manual check).
- [ ] **Step 3 (commit):** `docs(include): document foreign-format include dispatch and VACASK_WITH_SPICE`.

---

## Self-Review Notes

- **Spec coverage:** build optionality (Task 3), auto-fetch (Task 3 + Task 1 tag), main-binary wiring/include seam (Task 4 + `mergeForeignFile` Task 2), OSDI auto-load (Task 2), command-ignore-with-warning (Task 2 Step 4 + Task 5 Step 5), extension set (Global Constraints + Task 4 Step 1), testing incl. OFF build (Tasks 5-6). All covered.
- **API risks flagged inline** (must verify against `parseroutput.h` during implementation): `ParserTables::hasLoad`, `ParserTables::defaultSubDef()` mutable access, exact `PTLoad`/`add` chaining, and the flex action control-flow for skipping the native buffer push. These are the highest-risk unknowns; resolve them first when executing Task 2 and Task 4.
- **Type consistency:** `mergeNetlist` gains `std::set<std::string>& masters, bool projectAnalyses` in decl, defs, recursive calls, and all callers — keep the signature identical everywhere.

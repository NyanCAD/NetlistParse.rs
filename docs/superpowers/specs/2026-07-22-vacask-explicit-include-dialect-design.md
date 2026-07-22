# Explicit per-include dialect for foreign netlist includes

**Date:** 2026-07-22
**Status:** Design approved, ready for implementation plan
**Repos touched:** `VACASK` (grammar + bridge, most of the work), `NetlistParse.rs` (FFI signature + a `dialect` parameter on the `netlist-syntax` mixed parser)

## Problem

VACASK's `include` directive currently decides *how* to parse an included file
from its **filename extension**. There are three extension-driven inference
points:

1. `isForeignNetlistExt` (`VACASK/lib/dfllexer.l:28`) — native VACASK vs.
   foreign. `.cir/.sp/.spice/.mod/.lib/.scs/.spectre` route to the Rust bridge;
   everything else is lexed natively.
2. `mergeForeignFile` / `mergeNetlist` / `spiceBlockToTables`
   (`VACASK/lib/netlistrs.cpp:1143,1035,927`) — SPICE vs. Spectre, via
   `ext == ".scs" || ext == ".spectre"`.
3. The FFI entry point `parse_netlist(src, start_spice: bool)`
   (`NetlistParse.rs/crates/netlist-cxx/src/lib.rs:199`) — a bool, hardcoded to
   the **ngspice** dialect via `parse_spice`.

Extension cannot carry the meaning the parser needs:

- **Semantics are simulator-specific.** `level=49` (and MOS level numbering,
  statistical functions, temperature params, …) mean different things in
  ngspice vs. hspice vs. pspice. An extension does not say which.
- **The mapping is wrong in practice.** The Cadnip-generated
  `sky130_fd_pr/vacask/combined_models/sky130.lib.spice` is *VACASK-native
  Spectre-family syntax* (`library`/`section`/`@if`/`model`/`subckt`) but has a
  `.spice` extension. Loading it today yields **735 parse errors** because the
  `.spice` extension forces it through the Rust bridge as ngspice SPICE. Loaded
  natively (no bridge) the same content parses cleanly and honors `section=`
  selection and `@if` (verified empirically).

We want the caller to state the dialect **explicitly, per include**, and to stop
inferring it from the extension.

## Scope

**In scope — plumbing only.** Choose the correct parser/dialect based on an
explicit keyword, and thread that choice through the bridge and FFI.

**Out of scope — runtime translation.** This change does *not* add any
ngspice→VACASK semantic translation (target-model parameter filtering, device
dispatch, etc.). Loading raw ngspice models directly will still fail on
untranslated constructs — e.g. `include "sky130.lib.spice" lang=ngspice
section=tt` still errors on `Parameter 'scalm' not found`. Producing loadable
VACASK models from raw ngspice remains Cadnip's offline job
(`spak-convert … --input-simulator ngspice --output-simulator vacask`). See the
Appendix for why.

## Approach

### Routing rule (keyword-driven, replaces extension inference)

- **No `lang=`** → VACASK's **native** parser. It already fully handles its own
  Spectre-family syntax, `section=` selection, and `@if`/`@elseif`/`@end`
  (verified). The Cadnip-converted tree is intended to load here.
- **`lang=<dialect>`** → deferred to the **Rust bridge** with that explicit
  dialect. Accepted values: `ngspice | hspice | pspice | xyce | spectre`.
  - `ngspice/hspice/pspice/xyce` map to the parser's existing
    `Dialect` enum (`netlist-syntax/src/lexer.rs:26`), threaded into the mixed
    parser via a new `dialect` parameter on `parse_spectre_with`.
  - `spectre` routes through the bridge's Spectre parser, *not* the native path.

`isForeignNetlistExt` and the extension-based SPICE/Spectre split in
`netlistrs.cpp` are removed. Nothing selects a parser from the extension
anymore.

### Syntax

```
include "path"                          // native VACASK parse
include "path" section=tt               // native, section tt
include "path" lang=ngspice             // foreign, ngspice dialect
include "path" lang=ngspice section=tt  // foreign ngspice, section tt
include "path" lang=spectre             // foreign Cadence Spectre via bridge
```

`lang=` precedes the optional `section=`.

### Nested includes inside a foreign file

Plain `.include`/`.lib` directives *inside* a foreign file (e.g. the `tt`
section's `.include "corners/tt.spice"`) carry no `lang=` keyword. They inherit
the **parent foreign include's dialect** — the bridge threads the dialect
through its recursion instead of re-inferring from each nested file's
extension.

## Components and changes

### VACASK lexer (`lib/dfllexer.l`)
- Add a `lang=` keyword to the include directive, scanned in the same
  `INCEND`/`LIBEND` region that currently handles `section=`
  (`dfllexer.l:308-436`). A new lexer state captures the dialect identifier
  (mirrors the `LIBSECTION` state used for `section=`).
- Delete `isForeignNetlistExt` (`:28`) and its two callsites (`:330,413`). The
  native-vs-foreign decision becomes: `lang=` present ⇒ foreign (defer via
  `addPendingForeign`); absent ⇒ native (`setSection`/`pushStream`).

### VACASK parser tables (`include/parseroutput.h`)
- `struct PendingForeign` (`:780`) gains `std::string language`.
- `addPendingForeign` (`:781`) gains a `language` parameter.

### VACASK grammar (`lib/dflparser.y`)
- The end-of-parse drain (`:275`) passes `fi.language` to `mergeForeignFile`.

### VACASK bridge (`lib/netlistrs.cpp`, `include/netlistrs.h`)
- `mergeForeignFile` (`netlistrs.cpp:1136` / `netlistrs.h:49`) gains a
  `language` parameter; it selects the FFI entry (or `parse_netlist_lib` when a
  section is given) using the explicit dialect rather than the extension.
- `mergeNetlist` / `spiceBlockToTables` thread the dialect through nested-include
  recursion; the `ext == ".scs"` / `ext != ".scs"` branches
  (`:927,1035,1143`) are removed.

### Rust parser + FFI (`crates/netlist-syntax`, `crates/netlist-cxx/src/lib.rs`)
- **`netlist-syntax`:** give the mixed parser a `dialect` — add `parse_spectre_with(src, start_lang, dialect)` and thread it to `handoff_to_spice`, removing the hardcoded `Dialect::Ngspice` (`spectre_parser.rs:154`). This keeps the current `SpectreNetlistSource` CST shape and `simulator lang=` switching.
- **`netlist-cxx` FFI:** replace `parse_netlist(src, start_spice: bool)` with
  `parse_netlist(src, language: &str)` — a validated dialect string mapped to
  `Dialect` (SPICE dialects) or the Spectre start (`lang=spectre`), calling the
  dialect-aware mixed parser so projection is unchanged from today.
- `parse_netlist_lib(src, section, language)` gains the dialect and uses
  `parse_spice_dialect` for section extraction. `parse_spectre_netlist` is
  removed (subsumed by `lang=spectre`).
- **Spectre-with-section is unsupported** — `parse_netlist_lib` matches SPICE
  `LibStatement` only. `lang=spectre section=…` is rejected with a clear error.
  (VACASK-native Spectre sections go through the native path, which handles them.)

## Acceptance criteria

1. `include "x" lang=ngspice` parses as ngspice SPICE regardless of extension;
   `lang=spectre` routes to the Spectre parser through the bridge.
2. A Spectre-syntax file with a `.spice` name loads correctly via the native
   path (no keyword) — the extension mismatch is gone.
3. `VACASK/test/test_sky130_nfet_include.sim.in` is updated to put
   `lang=ngspice` on its `include` lines and still passes. This is the
   regression test for the plumbing.
4. Omitting `lang=` never routes to the bridge by extension; an unknown
   `lang=` value is a clear parse-time error.
5. A nested plain `.include` inside a `lang=ngspice` file is parsed as ngspice
   (dialect inherited), not re-inferred.

## Appendix: why raw ngspice still won't load (out of scope, for context)

Cadnip's converter is a semantic re-target, not a transliteration; that is the
work this change deliberately does *not* replicate:

- **Device dispatch to OSDI masters.** ngspice `nmos level=54/14` →
  `sp_bsim4v8` (+ injected `type=`); ngspice `d` → an `sp_diode` instance
  wrapped in a subckt (`Cadnip.jl/SpiceArmyKnife.jl/src/codegen.jl:205-367`).
- **Parameter filtering against the target model's parameter set.** Generic
  drops (`level`, `version`, `lmin/lmax/wmin/wmax`, `tref→tnom`) live in
  `parameter_mapping(VACASK)` (`simulator_traits.jl:226-240`); model-specific
  params the target OSDI lacks (e.g. `scalm` on a diode) are dropped because
  Cadnip knows `sp_diode`'s parameters. The bridge has no target-model
  knowledge and forwards params verbatim.
- **Model/subckt namespace de-collision.** Cadnip emits every model as
  `m_<name>` (`cg_spectre.jl` `:model_prefix => "m_"`), so a SPICE `.model foo`
  and `.subckt foo` (separate SPICE namespaces) do not collide in VACASK's
  unified model/subckt namespace (`circuit.cpp:581`).
- **Binning → `@if/@elseif`**, statistical funcs collapsed to nominal,
  `.lib/.endl` → `section/endsection`.

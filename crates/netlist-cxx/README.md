# netlist-cxx

A clean **C++ binding** for the Spectre netlist parser, built with
[cxx](https://cxx.rs).

It exposes an *eager, owned projection* of the Spectre typed AST
(`netlist_syntax::spectre_ast`) as plain-old-data value structs (`netlist::Netlist`
with `rust::Vec<netlist::Instance>`, `rust::String` fields). A C++ consumer sees
ordinary value types and range-for — no opaque handles, no FFI lifetime
management:

```cpp
#include "netlist_cxx/netlist.h"

netlist::Netlist nl = netlist::parse_netlist(rust::Str(source), rust::Str("spectre"));
for (const auto &inst : nl.instances)
    std::cout << std::string(inst.name) << " -> " << std::string(inst.master) << "\n";
```

## Why eager POD (not lazy handles)?

rowan already materialises the whole tree at parse time, so a lazy view saves no
parsing. What it *adds* is a heap allocation + an FFI crossing per node touched —
and any real consumer (building simulator parser tables, lowering to a flat
device list) walks essentially the whole netlist. The eager projection walks the
tree once in Rust and hands the data across in one transfer. It is also the
natural shape of the neutral structural IR downstream tools want.

Parameter/expression values are carried as **verbatim source text**; the consumer
re-parses them with its own expression engine. Dialect normalizations are applied
first, using the parser's own scope knowledge so the consumer needs no post-parse
scope tracking:

- the ambient-temperature builtin is canonicalized to `$temp` (Spectre free
  `temp`, SPICE free `temper`),
- each **Spectre** definition's *unconditional* parameter list is emitted in
  dependency order (a true cycle among unconditional parameters is reported
  through `errors`).

Dependency ordering is **Spectre-only**. SPICE parameter lists are left in source
order: the consumer's former sorter ordered Spectre definitions alone, and
reordering a SPICE list changes name resolution (a SPICE subckt `a=b` used to
read the enclosing `b`; a moved local definition would win instead). A genuine
unconditional SPICE cycle is still reported through `errors`. Parameters that
live inside conditional clauses (either dialect) are left in source order and
never report a cycle: the consumer resolves clause-gated namespaces itself and
evaluates only the active clauses, so a cycle between names defined in an
unselected clause is not an error — and the FFI `errors` channel is a bare span
that cannot say "not really an error".

The rewrite and the dependency scan both read verbatim text, so both consume the
dialect's tokens: a complete numeric literal is taken whole (the `k` in `a=1k`
and the `e` in `x=1e-3` are scale/exponent markers, not parameter references), a
single quote is the dialects' expression wrapper (`PRIME`) rather than a string
literal — only a double quote opens a string — and a lone ternary `:` is not
member qualification (`.` and `::` are). Case handling matches each dialect:
Spectre identifiers are case-sensitive, SPICE identifiers are case-folded in both
the builtin match and the dependency keys, and the emitted spelling is never
changed. `ic` values go through the same normalization as every other projected
expression. The FFI struct shapes are unchanged — the order inside each `Vec` and
the rewritten text are the only signals.

Shadowing by a parameter declared in an **included** file is supported without
the parser reading any file: the consumer resolves the include closure and hands
the parser an include **graph** — for each file, the parameter names it declares
(separating definition-level from clause-gated declarations) and its own nested
includes — through `parse_netlist_with_includes` (or, per section,
`parse_netlist_lib_with_includes`). The parser walks its OWN `include`
statements and applies each included file's declarations in the scope where that
statement sits: a definition-level include makes the file's unconditional names
visible in the enclosing scope, following nested definition-level includes
through the graph, while an include inside a conditional clause contributes only
clause-gated names. A reference in an *included* file — which carries no
`include` statement for its own includer — sees the merging scope's names through
the graph's `inherited` list. A free `temp`/`temper` that matches a
definition-level name is left unrewritten. Names follow the same case rule as
every other declaration — case-sensitive for Spectre, case-folded for SPICE. The
original `parse_netlist` / `parse_netlist_lib` entry points remain and default
the graph to empty, so single-file callers are unaffected.

## Known limitations of the builtin normalization

The projection covers the temperature-scope shapes the consumer needs — a free
`temp`/`temper` shadowed by a declaration in the same definition, by an
unconditional declaration visible through an include in that scope, or (bound by
the *active* clause of) a clause-gated declaration — with no consumer-side scope
re-derivation and no `temp=$temp` style alias injection. In particular, the two
corners the earlier flat inherited-name list could not express are now closed:

1. **Per-subcircuit namespaces.** The include graph is applied in the scope where
   each `include` sits, so a subcircuit that includes nothing never sees a
   sibling subcircuit's include declarations.
2. **Inactive-clause fallback.** A clause-gated `temp` declaration is
   canonicalized to `$temp` (the declared NAME is emitted as `$temp`, and free
   `temp` references in that namespace are rewritten to `$temp`) instead of
   suppressing the rewrite, so a reference in an inactive clause falls back to
   the ambient `$temp` and an active clause's `$temp` shadows it. No consumer
   alias is needed.

Two things remain out of scope, and a consumer that reaches them must still
resolve the name itself:

- **The rewrite is name-level, not an evaluator.** The parser never evaluates a
  condition, so it cannot know which clause is active at run time; the fallback
  relies on the consumer's contract that a parameter literally named `$temp`
  shadows its ambient option. A construct whose `temp` visibility depends on
  run-time state the graph does not describe is outside this model.
- **The includer's scope is still passed as a flat list.** For a reference in an
  *included* file, the names visible where that file is merged are supplied as
  `IncludeGraph.inherited` — inherently flat, because the file has no `include`
  statement for its own includer. The graph scopes the file's OWN includes; it
  cannot derive the includer context. This is narrower than the old whole-file
  list (it is one scope, not the whole closure) but it is not per-file scoped on
  its own.

The canonicalization is **Spectre-only** (`temp`); SPICE `temper` handling and
clause semantics are unchanged, and ordering stays Spectre-only.

## FFI boundary

Rust has no stable ABI and cannot emit the C++ ABI, so the linkage boundary is
`extern "C"` under the hood — cxx generates ergonomic C++ types on top of it. This
crate is the *exporting* side (`extern "Rust"` only), so it is pure Rust and needs
no `build.rs`: the C++ shim + header are produced by the `cxxbridge` CLI at the
consumer's build time.

## Building a C++ consumer

Prerequisites on `PATH`: `cargo`, and `cxxbridge` (`cargo install cxxbridge-cmd`).

The recipe (see `demo/CMakeLists.txt` for a working CMake version):

```sh
# 1. static library
cargo build -p netlist-cxx                       # -> target/debug/libnetlist_cxx.a

# 2. codegen: C++ header + shim from the bridge
cxxbridge crates/netlist-cxx/src/lib.rs --header -o gen/netlist_cxx/netlist.h
cxxbridge crates/netlist-cxx/src/lib.rs          -o gen/netlist_cxx/shim.cc

# 3. compile + link
g++ -std=c++20 -Igen consumer.cpp gen/netlist_cxx/shim.cc \
    target/debug/libnetlist_cxx.a -lpthread -ldl -lm -o consumer
```

## Demo

```sh
cmake -S crates/netlist-cxx/demo -B build
cmake --build build
./build/smoke crates/netlist-cxx/demo/example.scs
ctest --test-dir build --output-on-failure
```

## SPICE scopes and conditionals

SPICE regions are returned in `Netlist.spice_blocks`. Both `SpiceBlock` and
`SpiceSubckt` carry `includes` and `conditionals` alongside their existing
device, model, parameter, and subcircuit collections.

Each `SpiceConditional` contains ordered `SpiceCondClause` values:

- `condition` is the expression text for `.if` or `.elseif`, without the
  directive's surrounding parentheses. An empty string denotes `.else`.
- `body` is a `SpiceBlock` containing that clause's statements, including
  further conditionals, local model/parameter definitions, and includes.

For example, devices inside `.if (rfmode == 0)` stay in that clause's body;
they do not appear in the enclosing subcircuit's unconditional `devices`.
Conditions are not evaluated during parsing, even when constant. The consumer
selects branches using the eventual instance parameters.

Subcircuit-local `.include` and `.lib` directives retain their path and optional
section in that subcircuit's `includes`. The parser does not read included files
or deduplicate references across scopes. The consumer must resolve each include
relative to its source file and in its defining scope. For Spectre, the consumer
can hand the resolved closure to `parse_netlist_with_includes` /
`parse_netlist_lib_with_includes` as an include graph, so a `temp` a file's
include declares shadows (or, clause-gated, is canonicalized to `$temp`)
according to the include's own scope. As with
the existing projection, statements are grouped by kind; order is preserved
within each collection, not across different statement kinds.

Consumers must process these collections recursively, or reject constructs they
cannot support; ignoring them can turn a valid device wrapper into an empty
subcircuit. Preserving OSDI instances in the bridge does not implement their
electrical behavior in a simulator.

This extends the shared C++ struct layouts. Regenerate the header/shim and
rebuild consumers together with the Rust library; do not mix old generated
headers with a new static library. The `spice_scopes` CTest exercises the new
recursive data through the generated C++ interface.

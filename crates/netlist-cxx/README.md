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
the parser reading any file: the consumer resolves includes, collects the
parameter names they declare, and passes them to
`parse_netlist_with_inherited_names` (or, per section,
`parse_netlist_lib_with_inherited_names`); a free `temp`/`temper` that matches an
inherited name is left unrewritten. Inherited names follow the same case rule as
every other declaration — case-sensitive for Spectre, case-folded for SPICE. The
original `parse_netlist` / `parse_netlist_lib` entry points remain and default
the inherited set to empty, so single-file callers are unaffected.

## Known limitations of the builtin normalization

The projection covers the common temperature-scope shapes — a free `temp`/`temper`
shadowed by a declaration in the same definition, in one of its conditional
clauses, or in the enclosing include chain (the last passed in explicitly, above)
— and within those it needs no consumer-side scope re-derivation. It is **not** a
complete equivalent of the consumer's old scope machinery. Two corners are
deliberately out of scope, and a consumer that reaches them must resolve the name
itself:

1. **The inherited set is whole-file, so it cannot express per-subcircuit
   namespaces.** The inherited-names argument is one flat list applied to the
   whole projection. If an `include` local to subcircuit `s` declares `temp`,
   that name should shadow the builtin only inside `s`. A single whole-file list
   cannot express that: passing `temp` would wrongly shadow it in sibling
   subcircuits, and omitting it would wrongly expose the builtin inside `s`.
   Neither choice is correct for both, so scoped-to-one-subcircuit include
   shadowing is not representable through `parse_netlist_with_inherited_names` /
   `parse_netlist_lib_with_inherited_names`.

2. **A declaration in an *inactive* conditional clause still shadows.** A
   clause-gated `temp` declaration joins the definition's namespace, so a free
   `temp` elsewhere in the same definition is left spelled `temp` instead of being
   rewritten to `$temp`. When that clause is not selected at run time, the
   consumer's fixed-point resolver — which activates only the selected clauses —
   never binds `temp`, and the reference is left unbound. The projection cannot
   know which clauses are active, so it cannot avoid this.

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
relative to its source file and in its defining scope, and may feed the parameter
names those files declare back into the projection via
`parse_netlist_with_inherited_names` / `parse_netlist_lib_with_inherited_names`
so a declared `temp`/`temper` shadows the ambient-temperature builtin. As with
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

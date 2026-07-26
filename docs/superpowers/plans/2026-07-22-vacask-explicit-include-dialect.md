# Explicit per-include dialect (`lang=`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a VACASK `include` directive state its foreign dialect explicitly (`include "f" lang=ngspice section=tt`) and stop inferring the parser/dialect from the file extension.

**Architecture:** The VACASK lexer captures an optional `lang=<dialect>` on an include; presence of `lang=` routes the file through the Rust bridge with that explicit dialect, absence routes it to VACASK's native parser. The dialect threads: lexer → `PendingForeign` → grammar drain → `mergeForeignFile` → dialect-aware Rust FFI, and inherits through nested includes. On the Rust side, `netlist-syntax`'s mixed parser (`parse_spectre_with`) gains a `dialect` parameter — removing the hardcoded `Dialect::Ngspice` in `handoff_to_spice` — and the `netlist-cxx` FFI passes the caller's dialect straight through, preserving the current CST shape (`SpectreNetlistSource` + `collect_scope`) and mid-file `simulator lang=` switching.

**Tech Stack:** Rust (`cxx` FFI, `netlist-cxx`/`netlist-syntax`), C++17, flex (`dfllexer.l`), bison (`dflparser.y`), CMake/CTest.

## Global Constraints

- Scope is **plumbing only**: no runtime ngspice→VACASK parameter/device translation. Raw ngspice loads will still fail on untranslated params (e.g. `scalm`); that is expected and out of scope.
- Accepted `lang=` values (case-insensitive): `ngspice`, `hspice`, `pspice`, `xyce`, `spectre`. Map `ngspice/hspice/pspice/xyce` → `netlist_syntax::Dialect::{Ngspice,Hspice,Pspice,Xyce}` (parsed via `parse_spectre_with(src, StartLang::Spice, dialect)`); `spectre` → `parse_spectre_with(src, StartLang::Spectre, _)`.
- `lang=` precedes the optional `section=` in the directive.
- `lang=spectre section=…` is a hard error (Spectre sectioning is unsupported by `parse_netlist_lib`).
- Unknown `lang=` value is a clear parse-time error naming the value and include path.
- VACASK C++ changes are guarded by the existing `VACASK_WITH_SPICE` macro where they touch bridge code.
- Build VACASK in the existing `build/` tree (`cmake --build build --target sim`); the executable is `build/simulator/vacask`. Run tests with `ctest --test-dir build -R <name> --output-on-failure`.
- Two repos: FFI in `/home/pepijn/code/nyanodide/NetlistParse.rs`; everything else in `/home/pepijn/code/nyanodide/VACASK`. The VACASK build consumes the FFI crate via corrosion/cargo, so Task 1 must land (and the crate rebuild picked up) before the C++ tasks link.

---

### Task 1: Dialect-aware parsing (netlist-syntax) + dialect-string FFI (netlist-cxx)

Thread a `dialect` through the mixed parser (kill the hardcoded `Dialect::Ngspice`), then replace the FFI's `start_spice: bool` with an explicit dialect string and add the dialect to `parse_netlist_lib`. The FFI keeps using the mixed parser + `collect_scope`, so projection is byte-identical to today except the SPICE dialect is now selectable.

**Files:**
- Modify: `crates/netlist-syntax/src/spectre_parser.rs` (`Parser` struct `~60-87`; `Parser::new` `94-117`; `handoff_to_spice` `150-162`; `parse` `1278-1280`; `parse_with` `1285-1290`)
- Modify: `crates/netlist-syntax/src/lib.rs` (`parse_spectre_with` `45-47`)
- Modify (callers of `parse_spectre_with`): `crates/netlist-syntax/tests/spectre_differential.rs:72`, `crates/netlist-syntax/tests/roundtrip.rs:85`, `crates/netlist-syntax/examples/dump_spectre.rs:19`
- Modify: `crates/netlist-cxx/src/lib.rs` (`use` `19`; extern block `~199-207`; `parse_netlist` `913-937`; `parse_spectre_netlist` `939-942` remove; `parse_netlist_lib` `944-989`; `#[cfg(test)]` module `~993-end`)

**Interfaces:**
- Produces (netlist-syntax public API):
  - `pub fn parse_spectre_with(src: &str, start_lang: StartLang, dialect: Dialect) -> SyntaxNode`
    (the SPICE dialect used for any SPICE region — leading `.cir` region or mid-file `simulator lang=spice`).
- Produces (FFI, called from C++ Task 2):
  - `fn parse_netlist(src: &str, language: &str) -> Netlist`
  - `fn parse_netlist_lib(src: &str, section: &str, language: &str) -> Netlist`
  - `parse_spectre_netlist` is removed (no C++ callers; `lang=spectre` covers it).
- Internal helper: `resolve_lang(name: &str) -> Option<Lang>` where `enum Lang { Spice(Dialect), Spectre }`.

- [ ] **Step 1: Thread `dialect` through the mixed parser (netlist-syntax)**

In `crates/netlist-syntax/src/spectre_parser.rs`, add a `dialect: Dialect` field to `Parser` (after `dry: bool,` at `~86`):

```rust
    /// SPICE dialect used for any SPICE region (leading `.cir` region or a
    /// mid-file `simulator lang=spice` handoff).
    dialect: Dialect,
```

Change `Parser::new` (`94`) to take and store it:

```rust
    fn new(src: &'a str, dialect: Dialect) -> Self {
        let raw = Lexer::tokenize(src, ERROR);
        let mut p = Parser {
            src, raw, p: 0, started: false,
            nt: Sig { idx: 0, kind: ERROR },
            nnt: Sig { idx: 0, kind: ERROR },
            emit_idx: 0, builder: GreenNodeBuilder::new(),
            errored: false, lang_swapped: false, dry: false,
            dialect,
        };
        p.nt = p.next_sig();
        p.nnt = p.next_sig();
        p
    }
```

In `handoff_to_spice` (`152-154`) use the field instead of the constant:

```rust
        let (builder, stop, errored) = crate::parser::parse_spice_region(
            self.src,
            self.dialect,
            builder,
```

Change `parse_with` (`1285`) and `parse` (`1279`) to thread the dialect:

```rust
pub fn parse(src: &str) -> SyntaxNode {
    parse_with(src, StartLang::Spectre, Dialect::Ngspice)
}

pub fn parse_with(src: &str, start_lang: StartLang, dialect: Dialect) -> SyntaxNode {
    let mut p = Parser::new(src, dialect);
    p.parse_toplevel(start_lang);
    SyntaxNode::new_root(p.builder.finish())
}
```

In `crates/netlist-syntax/src/lib.rs` (`45-47`) update `parse_spectre_with` and re-export `Dialect` if not already visible there (it is: `pub use lexer::Dialect;`):

```rust
/// Parse a netlist that may switch dialects via `simulator lang=`, starting in
/// `start_lang`. `dialect` selects the SPICE dialect for any SPICE region.
pub fn parse_spectre_with(src: &str, start_lang: StartLang, dialect: Dialect) -> SyntaxNode {
    spectre_parser::parse_with(src, start_lang, dialect)
}
```

Update the three external callers to pass an explicit dialect (`Dialect::Ngspice` is the sensible default for these test/example corpora):
- `crates/netlist-syntax/tests/spectre_differential.rs:72` — `parse_spectre_with(&src, start_lang, netlist_syntax::Dialect::Ngspice)`
- `crates/netlist-syntax/tests/roundtrip.rs:85` — `parse_spectre_with(&src, start_lang, netlist_syntax::Dialect::Ngspice)`
- `crates/netlist-syntax/examples/dump_spectre.rs:19` — `parse_spectre_with(&src, start, netlist_syntax::Dialect::Ngspice)`

- [ ] **Step 2: Verify netlist-syntax still builds and passes**

Run: `cargo test -p netlist-syntax`
Expected: PASS (threading refactor is behavior-preserving for the ngspice default; existing tests unaffected).

- [ ] **Step 3: Write failing tests for the new FFI signatures**

Add to the `#[cfg(test)] mod tests` in `crates/netlist-cxx/src/lib.rs`:

```rust
#[test]
fn parse_netlist_ngspice_projects_spice_block() {
    let nl = super::parse_netlist("* t\nR1 a b 1k\n", "ngspice");
    assert!(nl.errors.is_empty(), "unexpected errors: {}", nl.errors.len());
    assert_eq!(nl.spice_blocks.len(), 1);
    assert_eq!(nl.spice_blocks[0].devices.len(), 1);
}

#[test]
fn parse_netlist_accepts_all_spice_dialects() {
    for d in ["ngspice", "hspice", "pspice", "xyce"] {
        let nl = super::parse_netlist("* t\nR1 a b 1k\n", d);
        assert!(nl.errors.is_empty(), "dialect {d} errored");
        assert_eq!(nl.spice_blocks.len(), 1, "dialect {d} block count");
    }
}

#[test]
fn parse_netlist_unknown_lang_errors() {
    let nl = super::parse_netlist("* t\nR1 a b 1k\n", "bogus");
    assert!(!nl.errors.is_empty());
}

#[test]
fn parse_netlist_lib_dialect_extracts_section() {
    let src = ".lib tt\nR1 a b 1k\n.endl tt\n.lib ff\nR1 a b 2k\n.endl ff\n";
    let nl = super::parse_netlist_lib(src, "tt", "ngspice");
    assert!(nl.errors.is_empty());
    assert_eq!(nl.spice_blocks.len(), 1);
    assert_eq!(nl.spice_blocks[0].devices.len(), 1);
}

#[test]
fn parse_netlist_lib_spectre_is_unsupported() {
    let nl = super::parse_netlist_lib(".lib tt\n.endl tt\n", "tt", "spectre");
    assert!(!nl.errors.is_empty());
}
```

Also update the existing tests in this module mechanically:
- `parse_netlist(src, true)` → `parse_netlist(src, "ngspice")`
- `parse_netlist(src, false)` → `parse_netlist(src, "spectre")`
- `parse_spectre_netlist(src)` → `parse_netlist(src, "spectre")`
- `parse_netlist_lib(src, "tt")` → `parse_netlist_lib(src, "tt", "ngspice")`
- Update the `use super::{...}` import line (`~993`) to drop `parse_spectre_netlist` and keep `parse_netlist, parse_netlist_lib`.

- [ ] **Step 4: Run tests to verify they fail**

Run: `cargo test -p netlist-cxx`
Expected: compile error / FAIL — `parse_netlist` still takes `bool`, new arg-count mismatch.

- [ ] **Step 5: Implement the dialect-aware FFI**

In the `extern "Rust"` block (`crates/netlist-cxx/src/lib.rs:~199-207`) replace the three signatures with:

```rust
    extern "Rust" {
        /// Parse foreign netlist source into a flat `Netlist`. `language` is one
        /// of ngspice|hspice|pspice|xyce|spectre (case-insensitive). SPICE
        /// dialects use the SPICE parser; `spectre` uses the Spectre parser.
        fn parse_netlist(src: &str, language: &str) -> Netlist;
        /// Parse a SPICE `.lib` file (dialect per `language`) and project only
        /// the named section. `language=spectre` is unsupported (errors).
        fn parse_netlist_lib(src: &str, section: &str, language: &str) -> Netlist;
    }
```

Update the `use` at line 19 to bring in the dialect entry points (`parse_spectre_with` is now 3-arg; `parse_spice_dialect` + `Dialect` are new):

```rust
use netlist_syntax::{
    parse_spectre_with, parse_spice_dialect, Dialect, StartLang, SyntaxKind, SyntaxNode,
    SyntaxToken,
};
```

Replace `parse_netlist` / `parse_spectre_netlist` (`913-942`) with the following. Note `parse_netlist` uses the **mixed parser for both** start languages (so `collect_scope` projection is unchanged from today); the dialect only affects SPICE regions:

```rust
enum Lang {
    Spice(Dialect),
    Spectre,
}

fn resolve_lang(name: &str) -> Option<Lang> {
    match name.to_ascii_lowercase().as_str() {
        "ngspice" => Some(Lang::Spice(Dialect::Ngspice)),
        "hspice" => Some(Lang::Spice(Dialect::Hspice)),
        "pspice" => Some(Lang::Spice(Dialect::Pspice)),
        "xyce" => Some(Lang::Spice(Dialect::Xyce)),
        "spectre" => Some(Lang::Spectre),
        _ => None,
    }
}

/// A `Netlist` carrying a single synthetic parse error (byte range 0..0), used
/// when the requested dialect is invalid/unsupported. C++ surfaces it as a
/// parse error; the authoritative, user-facing message is produced in
/// `mergeForeignFile` before this is reached.
fn lang_error_netlist() -> ffi::Netlist {
    let mut nl = empty_netlist();
    nl.errors.push(ffi::ParseError { start: 0, end: 0 });
    nl
}

fn empty_netlist() -> ffi::Netlist {
    ffi::Netlist {
        params: vec![], models: vec![], subckts: vec![], instances: vec![],
        analyses: vec![], saves: vec![], ics: vec![], globals: vec![],
        includes: vec![], ahdl_includes: vec![], errors: vec![], spice_blocks: vec![],
    }
}

pub fn parse_netlist(src: &str, language: &str) -> ffi::Netlist {
    let (start_lang, dialect) = match resolve_lang(language) {
        // Spectre start: dialect is only consulted if a `simulator lang=spice`
        // region appears; Ngspice is the harmless default there.
        Some(Lang::Spectre) => (StartLang::Spectre, Dialect::Ngspice),
        Some(Lang::Spice(d)) => (StartLang::Spice, d),
        None => return lang_error_netlist(),
    };
    let root = parse_spectre_with(src, start_lang, dialect);
    let errors = collect_errors(&root);
    let source = sast::SpectreNetlistSource::cast(root).expect("root is SpectreNetlistSource");
    let scope = collect_scope(source.statements());
    ffi::Netlist {
        params: scope.params, models: scope.models, subckts: scope.subckts,
        instances: scope.instances, analyses: scope.analyses, saves: scope.saves,
        ics: scope.ics, globals: scope.globals, includes: scope.includes,
        ahdl_includes: scope.ahdl_includes, errors, spice_blocks: scope.spice_blocks,
    }
}
```

(`empty_netlist` is retained only for `lang_error_netlist`; `parse_netlist` no longer projects a SPICE block directly — `collect_scope` continues to nest SPICE content as `spice_blocks`, exactly as before.)

Replace the head of `parse_netlist_lib` (`944-989`) so it takes and honors the dialect:

```rust
/// Parse a SPICE `.lib` file (given `language` dialect) and project only the
/// matching section. `spectre` is unsupported.
pub fn parse_netlist_lib(src: &str, section: &str, language: &str) -> ffi::Netlist {
    let dialect = match resolve_lang(language) {
        Some(Lang::Spice(d)) => d,
        Some(Lang::Spectre) | None => return lang_error_netlist(),
    };

    let root = parse_spice_dialect(src, dialect);
    let errors = collect_errors(&root);

    let mut block = ffi::SpiceBlock {
        params: vec![], models: vec![], subckts: vec![], devices: vec![], includes: vec![],
    };
    for child in root.children() {
        if child.kind() == SyntaxKind::LibStatement {
            if let Some(lib) = ast::LibStatement::cast(child) {
                let name = tok_text(lib.name());
                if name.eq_ignore_ascii_case(section) {
                    let inner = project_spice_block_children(lib.statements());
                    block.params.extend(inner.params);
                    block.models.extend(inner.models);
                    block.subckts.extend(inner.subckts);
                    block.devices.extend(inner.devices);
                    block.includes.extend(inner.includes);
                }
            }
        }
    }

    let mut nl = empty_netlist();
    nl.spice_blocks.push(block);
    nl.errors = errors;
    nl
}
```

(The prior `parse_netlist_lib` used `parse_spice(src)`; it now uses `parse_spice_dialect(src, dialect)`. Remove the now-unused `use netlist_syntax::parse_spice;` line inside the old body.)

- [ ] **Step 6: Run tests to verify they pass**

Run: `cargo test -p netlist-cxx && cargo test -p netlist-syntax`
Expected: PASS (all new + updated tests in both crates).

- [ ] **Step 7: Commit**

```bash
cd /home/pepijn/code/nyanodide/NetlistParse.rs
git add crates/netlist-syntax/src/spectre_parser.rs crates/netlist-syntax/src/lib.rs \
        crates/netlist-syntax/tests/spectre_differential.rs crates/netlist-syntax/tests/roundtrip.rs \
        crates/netlist-syntax/examples/dump_spectre.rs crates/netlist-cxx/src/lib.rs
git commit -m "feat(parser): dialect-aware parse_spectre_with + dialect-string FFI (drop hardcoded ngspice / start_spice bool)"
```

---

### Task 2: Adapt the C++ bridge to the new FFI (no routing change)

Make VACASK compile and pass existing tests against the new FFI signatures, WITHOUT yet introducing `lang=`. Extension-based dialect selection is preserved here temporarily (removed in Task 4) by mapping the existing bool/extension decision to a dialect string.

**Files:**
- Modify: `lib/netlistrs.cpp` (callsites `923,927-928,1031,1035-1036,1061,1113-1114,1143,1150-1152`)
- Modify: `include/netlistrs.h` (`buildParserTables` decl `21`, keep `mergeForeignFile` `49` unchanged this task)

**Interfaces:**
- Consumes: `netlist::parse_netlist(src, rust::Str language)`, `netlist::parse_netlist_lib(src, section, rust::Str language)` from Task 1.
- Produces: a file-local helper `static const char* spiceDialectForExt(const std::string& ext)` returning `"spectre"` for `.scs`/`.spectre`, else `"ngspice"`.

- [ ] **Step 1: Add the ext→dialect helper and update all callsites**

Near the top of the anonymous namespace in `lib/netlistrs.cpp`, add:

```cpp
// Transitional: map a file extension to a foreign dialect string for the new
// FFI. Removed in the lang=-only routing task; retained for the standalone
// buildParserTables* entries which load a top-level file directly.
static const char* dialectForExt(const std::string& ext) {
    return (ext == ".scs" || ext == ".spectre") ? "spectre" : "ngspice";
}
```

Update the six FFI callsites to pass a dialect string:

- `:923` and `:1031` (`spiceBlockToTables` / `mergeNetlist` sectioned include):
```cpp
            sub = netlist::parse_netlist_lib(rust::Str(contents), rust::Str(sv(inc.section)),
                                             rust::Str(dialectForExt(iext)));
```
  (Move the `iext` computation above this branch so it is available; today it is computed only in the `else`.)

- `:927-928` and `:1035-1036` (non-sectioned include):
```cpp
            std::string iext = absPath.extension().string();
            std::transform(iext.begin(), iext.end(), iext.begin(), ::tolower);
            sub = netlist::parse_netlist(rust::Str(contents), rust::Str(dialectForExt(iext)));
```

- `:1061` (`buildParserTables(source, startSpice, …)`):
```cpp
    netlist::Netlist nl = netlist::parse_netlist(rust::Str(source),
                                                 rust::Str(startSpice ? "ngspice" : "spectre"));
```

- `:1113-1114` (`buildParserTablesFromFile`):
```cpp
    netlist::Netlist nl = netlist::parse_netlist(rust::Str(source),
                                                 rust::Str(dialectForExt(ext)));
```

- `:1143,1150-1152` (`mergeForeignFile`): replace the `bool spice` selection with:
```cpp
    std::string dialect = dialectForExt(ext);
    netlist::Netlist nl = section.empty()
        ? netlist::parse_netlist(rust::Str(source), rust::Str(dialect))
        : netlist::parse_netlist_lib(rust::Str(source), rust::Str(section), rust::Str(dialect));
```

- [ ] **Step 2: Build to verify it compiles and links**

Run: `cmake --build build --target sim 2>&1 | tail -20`
Expected: builds with no errors (the Rust crate rebuilds first via corrosion).

- [ ] **Step 3: Run the existing foreign-include tests to verify no regression**

Run: `ctest --test-dir build -R "spice_include|sky130" --output-on-failure`
Expected: `test_spice_include`, `test_spice_include_section`, and `test_sky130_nfet_include` (if PDK present) PASS.

- [ ] **Step 4: Commit**

```bash
cd /home/pepijn/code/nyanodide/VACASK
git add lib/netlistrs.cpp include/netlistrs.h
git commit -m "refactor(bridge): adapt netlistrs callsites to dialect-string FFI"
```

---

### Task 3: `lang=` plumbing (lexer → PendingForeign → grammar → bridge)

Add the `lang=` keyword and thread the dialect end-to-end. Extension still decides native-vs-foreign (transitional); `lang=`, when present, overrides the dialect. New E2E test proves `lang=ngspice` works.

**Files:**
- Modify: `include/dflscanner.h` (`:85`, add `language` scratch var)
- Modify: `lib/dfllexer.l` (state list `:76`; `INCEND` rules `:307-311`; new `LANGVALUE` rules; `INCEND \n` `:315`; `LIBEND \n` `:405`)
- Modify: `include/parseroutput.h` (`PendingForeign` `:780`; `addPendingForeign` `:781-782`)
- Modify: `lib/dflparser.y` (`:275`)
- Modify: `include/netlistrs.h` (`mergeForeignFile` decl `:49`), `lib/netlistrs.cpp` (`mergeForeignFile` `:1136`, thread dialect through `mergeNetlist`/`spiceBlockToTables`)
- Create: `test/test_lang_include.sim` and register it in `test/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `struct PendingForeign { std::string path; std::string section; std::string language; };`
  - `ParserTables& addPendingForeign(std::string path, std::string section, std::string language) &;`
  - `bool mergeForeignFile(const std::string& path, const std::string& section, const std::string& language, PTSubcircuitDefinition& top, ParserTables& tab, Parser& p, Status& s);`
  - `mergeNetlist`/`spiceBlockToTables` gain a `const std::string& language` parameter threaded through recursion; when a nested include supplies its own `inc.section`/dialect it inherits the parent `language`.

- [ ] **Step 1: Write the failing E2E test**

Create `test/test_lang_include.sim`:

```
Explicit lang= foreign include

ground 0
model vsource vsource

// Minimal ngspice-syntax model file included with an explicit dialect.
include "lang_include_models.cir" lang=ngspice

r1 (n1 0) resr r=1k
v1 (n1 0) vsource dc=1.0

control
  abort always
  analysis op1 op
endc
```

Create the included ngspice file `test/lang_include_models.cir`:

```
* ngspice model file
.model resr r
```

Register both in `test/CMakeLists.txt` next to the other `.sim` tests (follow the existing `add_test` pattern around `:11`; copy the `.cir` beside the deck if the harness runs in a build dir — mirror how `test_spice_include` stages `spice_include_models.cir`).

- [ ] **Step 2: Run to verify it fails**

Run: `ctest --test-dir build -R test_lang_include --output-on-failure`
Expected: FAIL — the lexer does not yet accept `lang=` (syntax error at the include line).

- [ ] **Step 3: Add the `language` scratch var**

In `include/dflscanner.h` after `:85` (`std::string section;`):

```cpp
    std::string language;
```

- [ ] **Step 4: Lex the `lang=` keyword**

In `lib/dfllexer.l`, add `LANGVALUE` to the `%x` state list (`:76`):

```
%x LINESTART BODY QUOTED LONGQUOTED INC INCEND LIBSECTION LIBEND SECLOOK SECLOOKNAME SECLOOKEND LANGVALUE LONGCOMMENT
```

In the `INCEND` block, before the `section=` rule (`:308`), reset+capture `lang=`:

```
<INCEND>lang[ \t]*\= {  // Found lang=
                language.clear();
                BEGIN(LANGVALUE);
            }
```

Add `LANGVALUE` rules (mirroring `LIBSECTION` at `:382-395`), returning to `INCEND` so `section=` can still follow:

```
<LANGVALUE>[ \t]* {} // skip spaces after lang=
<LANGVALUE>[a-zA-Z_$][0-9a-zA-Z_$]* { // dialect identifier
                language = yytext;
                BEGIN(INCEND);
            }
<LANGVALUE>\n {
                error(*loc, "Syntax error, expected dialect name after lang=.");
                return(token::YYerror);
            }
<LANGVALUE>. {
                error(*loc, "Syntax error, unexpected string after lang=.");
                return(token::YYerror);
            }
```

Also clear `language` when a new include starts so it never leaks across directives: in the rule that enters `INC` (the `include` keyword handler that does `BEGIN(INC)` / pushes `INC`), add `language.clear();`. (Same place `sbuf` is prepared for a new filename.)

- [ ] **Step 5: Route by `lang=` presence (transitional: fall back to extension)**

In `include/parseroutput.h`, update the struct and setter (`:780-782`):

```cpp
    struct PendingForeign { std::string path; std::string section; std::string language; };
    ParserTables& addPendingForeign(std::string path, std::string section, std::string language) & {
        pendingForeign_.push_back({std::move(path), std::move(section), std::move(language)}); return *this; };
```

In `lib/dfllexer.l`, at both foreign-dispatch points update the decision and the call. `<INCEND>\n` (`:315`, no section):

```cpp
                    if (!language.empty() || isForeignNetlistExt(fname)) {
#ifdef VACASK_WITH_SPICE
                        ...
                        tables.addPendingForeign(fname, "", language);
```

`<LIBEND>\n` (`:405`, with section):

```cpp
                    if (!language.empty() || isForeignNetlistExt(fname)) {
#ifdef VACASK_WITH_SPICE
                        ...
                        tables.addPendingForeign(fname, section, language);
```

(Leave `isForeignNetlistExt` in place this task; Task 4 removes it. When `language` is empty and the extension is foreign, the bridge still derives the dialect from the extension via Task 2's helper.)

- [ ] **Step 6: Pass the dialect through the grammar and bridge**

In `lib/dflparser.y:275`:

```cpp
            if (!sim::mergeForeignFile(fi.path, fi.section, fi.language, $2.def, tables,
                                       foreignParser, status)) {
```

In `include/netlistrs.h:49` and `lib/netlistrs.cpp:1136`, add the `language` parameter to `mergeForeignFile`:

```cpp
bool mergeForeignFile(const std::string& path, const std::string& section,
                      const std::string& language,
                      PTSubcircuitDefinition& top, ParserTables& tab,
                      Parser& p, Status& s);
```

In `mergeForeignFile`'s body, choose the dialect explicitly when given, else fall back to extension (transitional):

```cpp
    std::string dialect = language.empty() ? dialectForExt(ext) : language;
    netlist::Netlist nl = section.empty()
        ? netlist::parse_netlist(rust::Str(source), rust::Str(dialect))
        : netlist::parse_netlist_lib(rust::Str(source), rust::Str(section), rust::Str(dialect));
```

Thread `dialect` into the recursion so nested plain includes inherit it: add a `const std::string& language` parameter to `mergeNetlist` (`:954`) and `spiceBlockToTables` (`:870`), pass `dialect` from `mergeForeignFile`/`buildParserTables*`, and in the nested-include loops (`:922-928`, `:1030-1036`) use the passed-in `language` instead of `dialectForExt(iext)`:

```cpp
        netlist::Netlist sub = inc.section.empty()
            ? netlist::parse_netlist(rust::Str(contents), rust::Str(language))
            : netlist::parse_netlist_lib(rust::Str(contents), rust::Str(sv(inc.section)),
                                         rust::Str(language));
```

For the two standalone entries that still start from an extension, pass `dialectForExt(ext)` as the `language` argument into `mergeNetlist` (`buildParserTables` uses `startSpice ? "ngspice" : "spectre"`).

- [ ] **Step 7: Build and run the new test + regressions**

Run: `cmake --build build --target sim 2>&1 | tail -20 && ctest --test-dir build -R "test_lang_include|spice_include|sky130" --output-on-failure`
Expected: `test_lang_include` PASS; existing foreign-include tests still PASS.

- [ ] **Step 8: Commit**

```bash
cd /home/pepijn/code/nyanodide/VACASK
git add include/dflscanner.h lib/dfllexer.l include/parseroutput.h lib/dflparser.y include/netlistrs.h lib/netlistrs.cpp test/test_lang_include.sim test/lang_include_models.cir test/CMakeLists.txt
git commit -m "feat(include): explicit lang= dialect keyword plumbed through the bridge"
```

---

### Task 4: Remove extension inference; route solely by `lang=`

Make `lang=` the only signal: presence ⇒ foreign, absence ⇒ native. Delete `isForeignNetlistExt` and the transitional `dialectForExt` fallback, and update the existing foreign-include test decks to carry `lang=`.

**Files:**
- Modify: `lib/dfllexer.l` (delete `isForeignNetlistExt` `:23-30` and its use at `:330`,`:413`)
- Modify: `lib/netlistrs.cpp` (remove `dialectForExt` fallback in `mergeForeignFile`; require non-empty dialect; validate the value)
- Modify: `test/test_sky130_nfet_include.sim.in` (`:16-20`), `test/test_spice_include.sim` (`:11`), `test/test_spice_include_section.sim` (`:11`)

**Interfaces:**
- Consumes: everything from Task 3.
- Produces: routing where `include` with no `lang=` is lexed natively; `mergeForeignFile` rejects an empty or unrecognized `language`.

- [ ] **Step 1: Update the existing foreign-include test decks to `lang=`**

`test/test_spice_include.sim:11`:
```
include "spice_include_models.cir" lang=ngspice
```
`test/test_spice_include_section.sim:11`:
```
include "spice_include_corners.lib" lang=ngspice section=tt
```
`test/test_sky130_nfet_include.sim.in:16-20` — append ` lang=ngspice` to each of the five `include` lines, e.g.:
```
include "@SKY130_CONTINUOUS@/parameters_fet_tt.spice" lang=ngspice
```

- [ ] **Step 2: Add dialect validation in `mergeForeignFile`**

In `lib/netlistrs.cpp` `mergeForeignFile`, replace the transitional fallback with strict validation:

```cpp
    static const std::set<std::string> kDialects =
        {"ngspice", "hspice", "pspice", "xyce", "spectre"};
    if (language.empty() || !kDialects.count(language)) {
        s.set(Status::Syntax, "include of '" + path +
              "': missing or unknown lang= (expected ngspice|hspice|pspice|xyce|spectre)");
        return false;
    }
    if (!section.empty() && language == "spectre") {
        s.set(Status::Syntax, "include of '" + path +
              "': section= is not supported with lang=spectre");
        return false;
    }
    netlist::Netlist nl = section.empty()
        ? netlist::parse_netlist(rust::Str(source), rust::Str(language))
        : netlist::parse_netlist_lib(rust::Str(source), rust::Str(section), rust::Str(language));
```

Remove the `dialectForExt` helper and its remaining uses; `mergeNetlist`/`spiceBlockToTables` now receive `language` (always a valid dialect) from `mergeForeignFile`. For `buildParserTables`/`buildParserTablesFromFile` (standalone top-level file loads, not includes) keep an explicit dialect: `buildParserTables` uses `startSpice ? "ngspice" : "spectre"`; `buildParserTablesFromFile` maps its extension inline (`ext == ".scs" || ext == ".spectre" ? "spectre" : "ngspice"`) — these are direct file loads, not the `include` seam.

- [ ] **Step 3: Delete `isForeignNetlistExt` and make routing `lang=`-only**

In `lib/dfllexer.l`, delete the `isForeignNetlistExt` function (`:23-30`). At both dispatch points (`:330` in `<INCEND>\n`, `:413` in `<LIBEND>\n`) change the condition from `if (!language.empty() || isForeignNetlistExt(fname))` to:

```cpp
                    if (!language.empty()) {
```

Now a no-`lang=` include falls through to the native `else` branch (`setSection`/`pushStream`), i.e. VACASK parses it natively.

- [ ] **Step 4: Build and run the full foreign-include + native suite**

Run: `cmake --build build --target sim 2>&1 | tail -20 && ctest --test-dir build -R "test_lang_include|spice_include|sky130" --output-on-failure`
Expected: all PASS. (Confirms `lang=ngspice` decks still work and extension no longer routes.)

- [ ] **Step 5: Verify a Spectre-syntax file with a `.spice` name loads natively**

Create a throwaway check (do not commit): a `.sim` deck that `include`s a small `library/section` Spectre-syntax file named `*.spice` with **no** `lang=`, plus the needed `load "..."`; confirm it parses (native path) rather than erroring as SPICE. This is the acceptance-criterion-2 smoke check.

Run: `build/simulator/vacask -dp <deck>` and confirm no parse errors from the include.
Expected: parses natively (any remaining error is about missing OSDI masters in the throwaway deck, not syntax).

- [ ] **Step 6: Commit**

```bash
cd /home/pepijn/code/nyanodide/VACASK
git add lib/dfllexer.l lib/netlistrs.cpp test/test_sky130_nfet_include.sim.in test/test_spice_include.sim test/test_spice_include_section.sim
git commit -m "feat(include): route foreign includes solely by lang=, drop extension inference"
```

---

## Self-Review

**Spec coverage:**
- Routing rule (no `lang=` → native; `lang=` → bridge): Task 3 (plumbing) + Task 4 (native fallthrough, `isForeignNetlistExt` removed). ✓
- Accepted dialects + mapping to `Dialect`: Task 1 `resolve_lang`. ✓
- `lang=` before `section=`: Task 3 lexer (`LANGVALUE` returns to `INCEND`, which still handles `section=`). ✓
- Nested includes inherit parent dialect: Task 3 Step 6 (threaded `language`). ✓
- FFI dialect-aware, `parse_spectre_netlist` removed, `parse_netlist_lib` gains dialect, Spectre-section unsupported: Task 1. ✓
- `PendingForeign`/`addPendingForeign`/`dflparser.y`/`mergeForeignFile` changes: Task 3. ✓
- Remove `isForeignNetlistExt` + `.scs`/`.spectre` split: Task 4. ✓
- Acceptance criteria 1–5: (1) Task 1 tests + Task 3 E2E; (2) Task 4 Step 5; (3) Task 4 Step 1 sky130 update; (4) Task 4 Step 3 + validation Step 2; (5) Task 3 Step 6 threading. ✓
- Out-of-scope (no runtime translation): honored — no param/device translation tasks. ✓

**Placeholder scan:** No TBD/TODO; all code steps show code; the one throwaway check (Task 4 Step 5) is a manual verification, not committed. ✓

**Type consistency:** `parse_netlist(src, language)`, `parse_netlist_lib(src, section, language)`, `resolve_lang`/`Lang`, `PendingForeign{path,section,language}`, `addPendingForeign(path,section,language)`, `mergeForeignFile(path,section,language,…)`, `dialectForExt` (added Task 2, removed Task 4) are used consistently across tasks. ✓

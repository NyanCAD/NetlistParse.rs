//! Clean C++ binding for the Spectre netlist parser via [`cxx`].
//!
//! This crate exposes an **eager, owned projection** of the Spectre typed AST
//! ([`netlist_syntax::spectre_ast`]) as plain-old-data structs shared with C++.
//! Rather than hand callers opaque node handles to traverse across the FFI
//! boundary (one heap allocation + one boundary crossing per node touched), we
//! walk the tree once in Rust and return fully-materialised value structs
//! (`rust::String`, `rust::Vec`) in a single transfer. For a consumer that
//! reads most of the netlist — building a simulator's parser tables, lowering
//! to a flat device list — that is both nicer to use and cheaper.
//!
//! Parameter/expression values are carried as **verbatim source text**: the
//! consumer re-parses them with its own expression engine (this mirrors how
//! VACASK round-trips expressions through `parseExpression`, and keeps this
//! layer free of a bespoke Spectre expression evaluator). Two dialect
//! normalizations are applied to that text before it crosses the FFI, both
//! using the parser's own scope knowledge so the consumer needs neither:
//!
//! * the ambient-temperature builtin is canonicalized to the consumer's `$temp`
//!   (Spectre free `temp` and SPICE free `temper`), and
//! * each definition's parameter list is emitted in dependency order (an
//!   expression after the parameters it references), with a true cycle
//!   reported through the `errors` field.
//!
//! The FFI struct shapes are unchanged: order inside the existing `Vec`s and
//! the rewritten text are the only signals.

use netlist_syntax::ast::{self, AstNode};
use netlist_syntax::spectre_ast as sast;
use netlist_syntax::{
    parse_spectre_with, parse_spice_dialect, Dialect, StartLang, SyntaxKind, SyntaxNode,
    SyntaxToken,
};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

#[cxx::bridge(namespace = "netlist")]
mod ffi {
    /// A `name = value` parameter; `value` is verbatim expression source text.
    struct Param {
        name: String,
        value: String,
    }

    /// A device/subckt instance line.
    struct Instance {
        name: String,
        master: String,
        nodes: Vec<String>,
        params: Vec<Param>,
    }

    /// A `model` card.
    struct Model {
        name: String,
        master: String,
        params: Vec<Param>,
    }

    /// An analysis statement (`<name> <type> params…`).
    struct Analysis {
        name: String,
        analysis_type: String,
        nodes: Vec<String>,
        params: Vec<Param>,
    }

    /// One saved signal (`node` and optional `:modifier`).
    struct SaveItem {
        signal: String,
        modifier: String,
    }

    /// One initial-condition assignment (`node = value`).
    struct IcItem {
        node: String,
        value: String,
    }

    /// An `include`/`ahdl_include`; `section` is empty when absent.
    struct Include {
        path: String,
        section: String,
    }

    /// The grouped content of one `{ … }` body (a conditional clause body).
    /// Mirrors `Subckt`/`SpiceBlock`: a Spectre structural `if` selects a whole
    /// block of statements, not a single instance.
    struct Block {
        params: Vec<Param>,
        models: Vec<Model>,
        instances: Vec<Instance>,
        subckts: Vec<Subckt>,
        conditionals: Vec<Conditional>,
        includes: Vec<Include>,
    }

    /// One clause of a structural conditional. `condition` is empty for the
    /// trailing `else`.
    struct CondClause {
        condition: String,
        body: Block,
    }

    /// A structural conditional block (`if/else if/else` over a statement block).
    struct Conditional {
        clauses: Vec<CondClause>,
    }

    /// A Spectre user-defined function (`real f(real a, …) { return expr; }`).
    /// `body` is the verbatim source text of the returned expression.
    struct FunctionDef {
        name: String,
        args: Vec<String>,
        body: String,
    }

    /// A `subckt`/`inline subckt` definition. Nested definitions are kept nested
    /// under `subckts`.
    struct Subckt {
        name: String,
        is_inline: bool,
        ports: Vec<String>,
        params: Vec<Param>,
        models: Vec<Model>,
        instances: Vec<Instance>,
        subckts: Vec<Subckt>,
        conditionals: Vec<Conditional>,
        includes: Vec<Include>,
        spice_blocks: Vec<SpiceBlock>,
    }

    /// The byte span `[start, end)` of an error node in the source.
    struct ParseError {
        start: u32,
        end: u32,
    }

    // --- SPICE block schema (full enum + structs; later tasks populate more
    // device kinds — defined in full now so the C++ side compiles against the
    // complete schema) ---

    #[derive(Debug, Clone, Copy)]
    enum SpiceDeviceKind {
        Resistor,
        Capacitor,
        Inductor,
        VSource,
        ISource,
        Diode,
        Mosfet,
        Bjt,
        Jfet,
        SubcktCall,
        Vcvs,
        Vccs,
        Ccvs,
        Cccs,
        MutualInductor,
        Behavioral,
        Switch,
        Osdi,
    }

    /// Voltage/current source waveform data (DC, AC, and transient).
    struct SpiceSource {
        dc: String,
        ac_mag: String,
        ac_phase: String,
        tran_kind: String,
        tran_args: Vec<String>,
    }

    /// A single SPICE device instance (R, C, L, V, I, D, M, …).
    struct SpiceDevice {
        kind: SpiceDeviceKind,
        name: String,
        nodes: Vec<String>,
        value: String,
        model: String,
        params: Vec<Param>,
        ctrl_nodes: Vec<String>,
        ctrl_value: String,
        source: SpiceSource,
    }

    /// A SPICE `.model` card.
    struct SpiceModel {
        name: String,
        model_type: String,
        level: String,
        params: Vec<Param>,
    }

    /// A SPICE `.subckt` definition (nested definitions kept nested).
    struct SpiceSubckt {
        name: String,
        ports: Vec<String>,
        params: Vec<Param>,
        devices: Vec<SpiceDevice>,
        models: Vec<SpiceModel>,
        subckts: Vec<SpiceSubckt>,
        includes: Vec<Include>,
        conditionals: Vec<SpiceConditional>,
    }

    /// One `.if`/`.elseif`/`.else` clause. An empty condition denotes `.else`;
    /// other conditions are unevaluated expression source text.
    struct SpiceCondClause {
        condition: String,
        body: SpiceBlock,
    }

    /// A SPICE conditional block, with clauses in source order.
    struct SpiceConditional {
        clauses: Vec<SpiceCondClause>,
    }

    /// SPICE statements grouped by kind within one scope: a contiguous SPICE
    /// region, a subcircuit body, a library section, or a conditional clause.
    struct SpiceBlock {
        params: Vec<Param>,
        models: Vec<SpiceModel>,
        subckts: Vec<SpiceSubckt>,
        devices: Vec<SpiceDevice>,
        includes: Vec<Include>,
        conditionals: Vec<SpiceConditional>,
    }

    /// The whole netlist, projected into grouped value structs.
    struct Netlist {
        params: Vec<Param>,
        models: Vec<Model>,
        subckts: Vec<Subckt>,
        instances: Vec<Instance>,
        analyses: Vec<Analysis>,
        saves: Vec<SaveItem>,
        ics: Vec<IcItem>,
        globals: Vec<String>,
        includes: Vec<Include>,
        ahdl_includes: Vec<String>,
        functions: Vec<FunctionDef>,
        conditionals: Vec<Conditional>,
        errors: Vec<ParseError>,
        /// Byte spans of `Incomplete` nodes. Distinct from `errors`: an
        /// `Incomplete` means a production could not be completed and its
        /// content may have been dropped, even when no `Error` token was
        /// emitted (e.g. an unterminated `subckt` at EOF).
        incompletes: Vec<ParseError>,
        spice_blocks: Vec<SpiceBlock>,
    }

    extern "Rust" {
        /// Parse foreign netlist source into a flat `Netlist`. `language` is one
        /// of ngspice|hspice|pspice|xyce|spectre (case-insensitive). SPICE
        /// dialects use the SPICE parser; `spectre` uses the Spectre parser.
        fn parse_netlist(src: &str, language: &str) -> Netlist;
        /// Like `parse_netlist`, but with `inherited_names` — the parameter
        /// names declared in the enclosing file(s) and their include chain —
        /// visible to the temperature-builtin shadowing. The parser reads no
        /// files: the consumer resolves includes, collects the names they
        /// declare, and passes them here. Matching is case-sensitive for
        /// Spectre and case-folded for SPICE. An empty slice reproduces
        /// `parse_netlist` exactly.
        fn parse_netlist_with_inherited_names(
            src: &str,
            language: &str,
            inherited_names: &[String],
        ) -> Netlist;
        /// Parse a SPICE `.lib` file (dialect per `language`) and project only
        /// the named section. `language=spectre` is unsupported (errors).
        fn parse_netlist_lib(src: &str, section: &str, language: &str) -> Netlist;
        /// Like `parse_netlist_lib`, with `inherited_names` visible to the
        /// temperature-builtin shadowing (see
        /// `parse_netlist_with_inherited_names`).
        fn parse_netlist_lib_with_inherited_names(
            src: &str,
            section: &str,
            language: &str,
            inherited_names: &[String],
        ) -> Netlist;
    }
}

// --- small text helpers ---

fn tok_text(t: Option<SyntaxToken>) -> String {
    t.map(|t| t.text().to_string()).unwrap_or_default()
}

/// Strip one layer of matching surrounding quotes from a string literal.
fn unquote(s: &str) -> String {
    let s = s.trim();
    let b = s.as_bytes();
    if b.len() >= 2 && (b[0] == b'"' || b[0] == b'\'') && b[b.len() - 1] == b[0] {
        s[1..s.len() - 1].to_string()
    } else {
        s.to_string()
    }
}

// --- dialect temperature builtins and parameter ordering ---

/// Bytes that may appear inside a dialect identifier (alnum, `_`, `$`).
fn ident_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_' || b == b'$'
}

/// Rewrite free dialect temperature builtins to `$temp` in verbatim expression
/// source text.
///
/// Spectre's `temp` and SPICE's `temper` both name the ambient simulation
/// temperature; the consumer (VACASK) spells that builtin `$temp`. Doing the
/// rewrite here — next to the parser's own scope knowledge — lets the consumer
/// drop its post-parse scope re-derivation entirely.
///
/// An identifier is rewritten only when it is genuinely free:
/// * not immediately a call name (`temp(...)`),
/// * not qualified with `.` or `::`,
/// * not inside a quoted string (backslash escapes respected),
/// * not inside a `v(...)`/`i(...)` argument list (those name nodes/branches),
/// * not declared as a parameter in `scope` (a declaration shadows the builtin).
///
/// Spectre matches `temp` case-sensitively (its identifiers are case-sensitive);
/// SPICE matches `temper` case-insensitively (its identifiers are case-folded,
/// mirroring the consumer's `spiceExpr`).
fn rewrite_temp(text: &str, spice: bool, scope: &HashSet<String>) -> String {
    let needle = if spice { "temper" } else { "temp" };
    let bytes = text.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(text.len() + 8);
    let mut access_frames: Vec<bool> = Vec::new();
    let mut access_depth = 0usize;
    let mut quote: Option<u8> = None;
    let mut i = 0usize;
    while i < bytes.len() {
        let c = bytes[i];
        if let Some(q) = quote {
            out.push(c);
            if c == b'\\' && i + 1 < bytes.len() {
                out.push(bytes[i + 1]);
                i += 2;
                continue;
            }
            if c == q {
                quote = None;
            }
            i += 1;
            continue;
        }
        if c == b'"' || c == b'\'' {
            quote = Some(c);
            out.push(c);
            i += 1;
            continue;
        }
        if c.is_ascii_alphabetic() || c == b'_' || c == b'$' {
            let start = i;
            let mut j = i;
            while j < bytes.len() && ident_byte(bytes[j]) {
                j += 1;
            }
            let ident = &text[start..j];
            let mut k = j;
            while k < bytes.len() && bytes[k].is_ascii_whitespace() {
                k += 1;
            }
            let is_call = k < bytes.len() && bytes[k] == b'(';
            let prev = if start > 0 { bytes[start - 1] } else { 0 };
            let next = if j < bytes.len() { bytes[j] } else { 0 };
            let qualified = matches!(prev, b'.' | b':') || matches!(next, b'.' | b':');
            let hit = if spice {
                ident.eq_ignore_ascii_case(needle)
            } else {
                ident == needle
            };
            let declared = if spice {
                scope.contains(&ident.to_ascii_lowercase())
            } else {
                scope.contains(ident)
            };
            if hit && !is_call && !qualified && access_depth == 0 && !declared {
                out.extend_from_slice(b"$temp");
            } else {
                out.extend_from_slice(ident.as_bytes());
            }
            if is_call {
                let is_access = ident.eq_ignore_ascii_case("v") || ident.eq_ignore_ascii_case("i");
                out.extend_from_slice(&bytes[j..k]);
                out.push(b'(');
                access_frames.push(is_access);
                if is_access {
                    access_depth += 1;
                }
                i = k + 1;
            } else {
                i = j;
            }
            continue;
        }
        if c == b'(' {
            access_frames.push(false);
        } else if c == b')' && access_frames.pop() == Some(true) {
            access_depth = access_depth.saturating_sub(1);
        }
        out.push(c);
        i += 1;
    }
    // Only ASCII was inserted; the input was valid UTF-8, so this cannot fail.
    String::from_utf8(out).expect("temperature rewrite preserves UTF-8")
}

/// Free identifiers referenced by an expression: those that are not call names
/// (`f(`), not qualified (`.x`/`::x`) and not inside a `v(...)`/`i(...)`
/// argument list. Used to build parameter dependency edges.
fn expr_identifiers(text: &str) -> Vec<String> {
    let bytes = text.as_bytes();
    let mut out = Vec::new();
    let mut access_frames: Vec<bool> = Vec::new();
    let mut access_depth = 0usize;
    let mut quote: Option<u8> = None;
    let mut i = 0usize;
    while i < bytes.len() {
        let c = bytes[i];
        if let Some(q) = quote {
            if c == b'\\' && i + 1 < bytes.len() {
                i += 2;
                continue;
            }
            if c == q {
                quote = None;
            }
            i += 1;
            continue;
        }
        if c == b'"' || c == b'\'' {
            quote = Some(c);
            i += 1;
            continue;
        }
        if c.is_ascii_alphabetic() || c == b'_' || c == b'$' {
            let start = i;
            let mut j = i;
            while j < bytes.len() && ident_byte(bytes[j]) {
                j += 1;
            }
            let ident = &text[start..j];
            let mut k = j;
            while k < bytes.len() && bytes[k].is_ascii_whitespace() {
                k += 1;
            }
            let is_call = k < bytes.len() && bytes[k] == b'(';
            let prev = if start > 0 { bytes[start - 1] } else { 0 };
            let next = if j < bytes.len() { bytes[j] } else { 0 };
            let qualified = matches!(prev, b'.' | b':') || matches!(next, b'.' | b':');
            if !is_call && !qualified && access_depth == 0 {
                out.push(ident.to_string());
            }
            if is_call {
                let is_access = ident.eq_ignore_ascii_case("v") || ident.eq_ignore_ascii_case("i");
                access_frames.push(is_access);
                if is_access {
                    access_depth += 1;
                }
                i = k + 1;
            } else {
                i = j;
            }
            continue;
        }
        if c == b'(' {
            access_frames.push(false);
        } else if c == b')' && access_frames.pop() == Some(true) {
            access_depth = access_depth.saturating_sub(1);
        }
        i += 1;
    }
    out
}

/// Reorder a definition's parameter list so each expression follows the
/// parameters it references.
///
/// A self-reference (a reference to one's own name) denotes the enclosing scope
/// and adds no edge — the same rule VACASK applies in
/// `parameterExpressionOrder`. Independent parameters keep their source order
/// (stable: a min-heap on the source index breaks ties), and a reference to a
/// later parameter is an edge, so forward references are resolved.
///
/// Returns `Err(the input)` on a true cycle so the caller can surface it through
/// the FFI `errors` field rather than dropping or silently reordering it.
fn order_params(params: Vec<ffi::Param>) -> Result<Vec<ffi::Param>, Vec<ffi::Param>> {
    let n = params.len();
    if n <= 1 {
        return Ok(params);
    }
    // A reference resolves to the parameter's last definition (later wins).
    let mut index: HashMap<&str, usize> = HashMap::with_capacity(n);
    for (i, p) in params.iter().enumerate() {
        index.insert(p.name.as_str(), i);
    }
    let mut succ: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg = vec![0usize; n];
    for (i, p) in params.iter().enumerate() {
        for r in expr_identifiers(&p.value) {
            if r == p.name {
                // Self-reference: resolves in the enclosing scope, no edge.
                continue;
            }
            if let Some(&j) = index.get(r.as_str()) {
                if j != i {
                    succ[j].push(i);
                    indeg[i] += 1;
                }
            }
        }
    }
    drop(index);
    let mut ready: BinaryHeap<Reverse<usize>> = BinaryHeap::new();
    for (i, &d) in indeg.iter().enumerate() {
        if d == 0 {
            ready.push(Reverse(i));
        }
    }
    let mut order = Vec::with_capacity(n);
    while let Some(Reverse(i)) = ready.pop() {
        order.push(i);
        for &k in &succ[i] {
            indeg[k] -= 1;
            if indeg[k] == 0 {
                ready.push(Reverse(k));
            }
        }
    }
    if order.len() != n {
        return Err(params);
    }
    let mut slots: Vec<Option<ffi::Param>> = params.into_iter().map(Some).collect();
    let mut out = Vec::with_capacity(n);
    for i in order {
        out.push(slots[i].take().expect("each source index appears once"));
    }
    Ok(out)
}

/// Order a parameter list, surfacing a cycle through `errors` (a synthetic,
/// zero-width `ParseError` — the FFI carries no message channel) while keeping
/// the parameters in source order rather than dropping them.
fn ordered(params: Vec<ffi::Param>, errors: &mut Vec<ffi::ParseError>) -> Vec<ffi::Param> {
    match order_params(params) {
        Ok(p) => p,
        Err(orig) => {
            errors.push(ffi::ParseError { start: 0, end: 0 });
            orig
        }
    }
}

// --- element projections ---

fn project_param(p: &sast::Parameter, scope: &HashSet<String>) -> ffi::Param {
    ffi::Param {
        name: tok_text(p.name()),
        value: rewrite_temp(&p.value_text().unwrap_or_default(), false, scope),
    }
}

fn project_instance(i: &sast::Instance, scope: &HashSet<String>) -> ffi::Instance {
    ffi::Instance {
        name: tok_text(i.name()),
        master: tok_text(i.master()),
        nodes: i.nodes().iter().map(sast::SNode::path).collect(),
        params: i.params().map(|p| project_param(&p, scope)).collect(),
    }
}

fn project_model(m: &sast::Model, scope: &HashSet<String>) -> ffi::Model {
    ffi::Model {
        name: tok_text(m.name()),
        master: tok_text(m.master()),
        params: m.params().map(|p| project_param(&p, scope)).collect(),
    }
}

fn project_analysis(a: &sast::Analysis, scope: &HashSet<String>) -> ffi::Analysis {
    ffi::Analysis {
        name: tok_text(a.name()),
        analysis_type: tok_text(a.analysis_type()),
        nodes: a.nodes().iter().map(sast::SNode::path).collect(),
        params: a.params().map(|p| project_param(&p, scope)).collect(),
    }
}

fn project_function(f: &sast::FunctionDecl, scope: &HashSet<String>) -> ffi::FunctionDef {
    // Function arguments are visible in the body: a `temp` argument keeps its
    // binding, so extend the scope with them.
    let mut body_scope = scope.clone();
    let args: Vec<String> = f.args().iter().map(|a| tok_text(a.name())).collect();
    for a in &args {
        body_scope.insert(a.clone());
    }
    ffi::FunctionDef {
        name: tok_text(f.name()),
        args,
        body: rewrite_temp(&f.return_expr().unwrap_or_default(), false, &body_scope),
    }
}

/// Project a clause body (an iterator of statement nodes) into a `Block`.
fn project_block(
    stmts: impl Iterator<Item = SyntaxNode>,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::Block {
    let scope = collect_scope(stmts, inherited, errors);
    ffi::Block {
        params: scope.params,
        models: scope.models,
        instances: scope.instances,
        subckts: scope.subckts,
        conditionals: scope.conditionals,
        includes: scope.includes,
    }
}

fn project_conditional(
    c: &sast::ConditionalBlock,
    scope: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::Conditional {
    let mut clauses = Vec::new();
    if let Some(iff) = c.if_clause() {
        clauses.push(ffi::CondClause {
            condition: rewrite_temp(&iff.condition().unwrap_or_default(), false, scope),
            body: project_block(iff.body(), scope, errors),
        });
    }
    for ei in c.else_ifs() {
        clauses.push(ffi::CondClause {
            condition: rewrite_temp(&ei.condition().unwrap_or_default(), false, scope),
            body: project_block(ei.body(), scope, errors),
        });
    }
    if let Some(els) = c.else_clause() {
        clauses.push(ffi::CondClause {
            condition: String::new(),
            body: project_block(els.body(), scope, errors),
        });
    }
    ffi::Conditional { clauses }
}

fn project_subckt(
    s: &sast::Subckt,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::Subckt {
    let scope = collect_scope(s.body(), inherited, errors);
    ffi::Subckt {
        name: tok_text(s.name()),
        is_inline: s.is_inline(),
        ports: s.ports().iter().map(sast::SNode::path).collect(),
        params: scope.params,
        models: scope.models,
        instances: scope.instances,
        subckts: scope.subckts,
        conditionals: scope.conditionals,
        includes: scope.includes,
        spice_blocks: scope.spice_blocks,
    }
}

// --- SPICE projection helpers ---

/// Source text of an optional `HierarchialNode`.
fn hier_text(h: Option<ast::HierarchialNode>) -> String {
    h.map(|n| n.text()).unwrap_or_default()
}

/// Project a SPICE `Parameter` (name-only or `name=value`).
fn project_spice_param(p: &ast::Parameter, scope: &HashSet<String>) -> ffi::Param {
    ffi::Param {
        name: tok_text(p.name()),
        value: rewrite_temp(
            &p.value().map(|e| e.text()).unwrap_or_default(),
            true,
            scope,
        ),
    }
}

/// Extract the value text from a `VoltageControl` or `CurrentControl` syntax node.
///
/// Controlled-source gain/transconductance values are emitted by the parser as
/// bare `NumberLiteral` tokens rather than wrapped `LiteralExpr` nodes, so the
/// typed-AST `value()` accessor (which uses `expr_children`) misses them.  This
/// helper tries expression-node children first (covers `BinaryExpression`, etc.)
/// then falls back to scanning direct token children for `NumberLiteral`/`Literal`.
fn ctrl_value_text(node: &SyntaxNode) -> String {
    // In VoltageControl / CurrentControl / MutualInductor, HierarchialNode children are
    // control-node names; the first non-HierarchialNode child node is the gain/coupling
    // expression. (Plain numeric gains are bare tokens, handled by the token fallback below.)
    // Expression-node children first (wraps complex expressions).
    for child in node.children() {
        if !matches!(child.kind(), SyntaxKind::HierarchialNode) {
            return child.text().to_string();
        }
    }
    // Fall back: bare NumberLiteral or Literal token child.
    for el in node.children_with_tokens() {
        if let Some(tok) = el.into_token() {
            if matches!(tok.kind(), SyntaxKind::NumberLiteral | SyntaxKind::Literal) {
                return tok.text().to_string();
            }
        }
    }
    String::new()
}

fn empty_source() -> ffi::SpiceSource {
    ffi::SpiceSource {
        dc: String::new(),
        ac_mag: String::new(),
        ac_phase: String::new(),
        tran_kind: String::new(),
        tran_args: vec![],
    }
}

/// Build a `SpiceSource` by walking the `DCSource`/`ACSource`/`TranSource`
/// child nodes yielded by `Voltage::sources()` or `Current::sources()`.
fn project_spice_source(
    sources: impl Iterator<Item = SyntaxNode>,
    scope: &HashSet<String>,
) -> ffi::SpiceSource {
    let mut out = empty_source();
    for src in sources {
        match src.kind() {
            SyntaxKind::DCSource => {
                if let Some(dc) = ast::DCSource::cast(src) {
                    out.dc = rewrite_temp(
                        &dc.value().map(|e| e.text()).unwrap_or_default(),
                        true,
                        scope,
                    );
                }
            }
            SyntaxKind::ACSource => {
                if let Some(ac) = ast::ACSource::cast(src) {
                    out.ac_mag = rewrite_temp(
                        &ac.magnitude().map(|e| e.text()).unwrap_or_default(),
                        true,
                        scope,
                    );
                    out.ac_phase = rewrite_temp(
                        &ac.phase().map(|e| e.text()).unwrap_or_default(),
                        true,
                        scope,
                    );
                }
            }
            SyntaxKind::TranSource => {
                if let Some(tran) = ast::TranSource::cast(src) {
                    out.tran_kind = tok_text(tran.function());
                    out.tran_args = tran
                        .values()
                        .map(|e| rewrite_temp(&e.text(), true, scope))
                        .collect();
                }
            }
            _ => {}
        }
    }
    out
}

/// Project a single SPICE device `SyntaxNode` into a `SpiceDevice`, or return
/// `None` for non-device nodes (statements, `.model`, `.subckt`, …).
///
/// This helper is shared by `project_spice_block` and `project_spice_subckt`
/// so that subckt bodies receive the same device coverage as the top-level block.
fn project_spice_device(child: SyntaxNode, scope: &HashSet<String>) -> Option<ffi::SpiceDevice> {
    match child.kind() {
        SyntaxKind::Resistor => ast::Resistor::cast(child).map(|r| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Resistor,
            name: hier_text(r.name()),
            nodes: vec![hier_text(r.pos()), hier_text(r.neg())],
            value: rewrite_temp(
                &r.value().map(|e| e.text()).unwrap_or_default(),
                true,
                scope,
            ),
            model: String::new(),
            params: r.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),
        SyntaxKind::Capacitor => ast::Capacitor::cast(child).map(|c| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Capacitor,
            name: hier_text(c.name()),
            nodes: vec![hier_text(c.pos()), hier_text(c.neg())],
            value: rewrite_temp(
                &c.value().map(|e| e.text()).unwrap_or_default(),
                true,
                scope,
            ),
            model: String::new(),
            params: c.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),
        SyntaxKind::Inductor => ast::Inductor::cast(child).map(|l| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Inductor,
            name: hier_text(l.name()),
            nodes: vec![hier_text(l.pos()), hier_text(l.neg())],
            value: rewrite_temp(
                &l.value().map(|e| e.text()).unwrap_or_default(),
                true,
                scope,
            ),
            model: String::new(),
            params: l.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),
        SyntaxKind::Voltage => ast::Voltage::cast(child).map(|v| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::VSource,
            name: hier_text(v.name()),
            nodes: vec![hier_text(v.pos()), hier_text(v.neg())],
            value: String::new(),
            model: String::new(),
            params: vec![],
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: project_spice_source(v.sources(), scope),
        }),
        SyntaxKind::Current => ast::Current::cast(child).map(|i| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::ISource,
            name: hier_text(i.name()),
            nodes: vec![hier_text(i.pos()), hier_text(i.neg())],
            value: String::new(),
            model: String::new(),
            params: vec![],
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: project_spice_source(i.sources(), scope),
        }),
        SyntaxKind::Diode => ast::Diode::cast(child).map(|d| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Diode,
            name: hier_text(d.name()),
            nodes: vec![hier_text(d.pos()), hier_text(d.neg())],
            value: String::new(),
            model: hier_text(d.model()),
            params: d.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),
        SyntaxKind::MOSFET => ast::MOSFET::cast(child).map(|m| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Mosfet,
            name: hier_text(m.name()),
            nodes: vec![
                hier_text(m.drain()),
                hier_text(m.gate()),
                hier_text(m.source()),
                hier_text(m.bulk()),
            ],
            value: String::new(),
            model: hier_text(m.model()),
            params: m.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),
        SyntaxKind::BipolarTransistor => ast::BipolarTransistor::cast(child).map(|q| {
            // `nodes()` yields ALL HierarchialNode children in source order:
            // [instance-name, c, b, e, [s,] model] — mirroring ngspice syntax
            // `Q <c> <b> <e> [<s>] <model>`. Skip the first entry (instance name),
            // treat the last entry as the model reference, and the remainder as
            // terminal connections. A follow-up can refine substrate (4th terminal)
            // vs model disambiguation if needed.
            let all: Vec<String> = q.nodes().map(|n| n.text()).collect();
            // Guard: a malformed bare `Q1` has len<=1 → slice [1..0] would panic.
            let (terminals, model) = if all.len() < 2 {
                (Vec::new(), String::new())
            } else {
                (all[1..all.len() - 1].to_vec(), all[all.len() - 1].clone())
            };
            ffi::SpiceDevice {
                kind: ffi::SpiceDeviceKind::Bjt,
                name: hier_text(q.name()),
                nodes: terminals,
                value: String::new(),
                model,
                params: q.params().map(|p| project_spice_param(&p, scope)).collect(),
                ctrl_nodes: vec![],
                ctrl_value: String::new(),
                source: empty_source(),
            }
        }),
        SyntaxKind::SubcktCall => ast::SubcktCall::cast(child).map(|x| {
            // `nodes()` yields ALL HierarchialNode children in source order:
            // [instance-name, conn1, conn2, ..., master-subckt]. Skip the first
            // entry (instance name), treat the last as the master reference, and
            // the remainder as the connection nodes.
            let all: Vec<String> = x.nodes().map(|n| n.text()).collect();
            // Guard: a malformed bare `X1` has len<=1 → slice [1..0] would panic.
            let (conn_nodes, model) = if all.len() < 2 {
                (Vec::new(), String::new())
            } else {
                (all[1..all.len() - 1].to_vec(), all[all.len() - 1].clone())
            };
            ffi::SpiceDevice {
                kind: ffi::SpiceDeviceKind::SubcktCall,
                name: hier_text(x.name()),
                nodes: conn_nodes,
                value: String::new(),
                model,
                params: x.params().map(|p| project_spice_param(&p, scope)).collect(),
                ctrl_nodes: vec![],
                ctrl_value: String::new(),
                source: empty_source(),
            }
        }),
        // --- Controlled sources (E/F/G/H) ---
        //
        // The parser emits a single `ControlledSource` node for all four letters.
        // The control kind distinguishes voltage- vs current-controlled:
        //   VoltageControl → E (Vcvs) or G (Vccs)
        //   CurrentControl → F (Cccs) or H (Ccvs)
        // The final E-vs-G / F-vs-H disambiguation uses the device-name prefix.
        // ctrl_nodes: VoltageControl → control_nodes(); CurrentControl → [vnam()]
        // ctrl_value: gain/transconductance (bare NumberLiteral token in the CST)
        // POLY/TABLE: captured best-effort as ctrl_value text; full parsing deferred.
        SyntaxKind::ControlledSource => ast::ControlledSource::cast(child).and_then(|cs| {
            let name = hier_text(cs.name());
            let kind = match name.chars().next().map(|c| c.to_ascii_uppercase()) {
                Some('E') => ffi::SpiceDeviceKind::Vcvs,
                Some('F') => ffi::SpiceDeviceKind::Cccs,
                Some('G') => ffi::SpiceDeviceKind::Vccs,
                Some('H') => ffi::SpiceDeviceKind::Ccvs,
                _ => return None,
            };
            let nodes = vec![hier_text(cs.pos()), hier_text(cs.neg())];
            let (ctrl_nodes, ctrl_value, params) = match cs.control() {
                Some(ctrl) => match ctrl.kind() {
                    SyntaxKind::VoltageControl => {
                        if let Some(vc) = ast::VoltageControl::cast(ctrl) {
                            let cnodes: Vec<String> =
                                vc.control_nodes().map(|n| n.text()).collect();
                            let cval = rewrite_temp(&ctrl_value_text(vc.syntax()), true, scope);
                            let ps: Vec<ffi::Param> = vc
                                .params()
                                .map(|p| project_spice_param(&p, scope))
                                .collect();
                            (cnodes, cval, ps)
                        } else {
                            (vec![], String::new(), vec![])
                        }
                    }
                    SyntaxKind::CurrentControl => {
                        if let Some(cc) = ast::CurrentControl::cast(ctrl) {
                            let vnam = hier_text(cc.vnam());
                            let cval = rewrite_temp(&ctrl_value_text(cc.syntax()), true, scope);
                            let ps: Vec<ffi::Param> = cc
                                .params()
                                .map(|p| project_spice_param(&p, scope))
                                .collect();
                            (vec![vnam], cval, ps)
                        } else {
                            (vec![], String::new(), vec![])
                        }
                    }
                    // POLY/TABLE: capture full node text as ctrl_value (deferred).
                    _ => (
                        vec![],
                        rewrite_temp(&ctrl.text().to_string(), true, scope),
                        vec![],
                    ),
                },
                None => (vec![], String::new(), vec![]),
            };
            Some(ffi::SpiceDevice {
                kind,
                name,
                nodes,
                value: String::new(),
                model: String::new(),
                params,
                ctrl_nodes,
                ctrl_value,
                source: empty_source(),
            })
        }),

        // --- Mutual inductor (K) ---
        // nodes = [L1, L2, ...]; coupling value stored in `value`.
        SyntaxKind::MutualInductor => ast::MutualInductor::cast(child).map(|k| {
            let nodes: Vec<String> = k.inductors().map(|n| n.text()).collect();
            let coupling = rewrite_temp(&ctrl_value_text(k.syntax()), true, scope);
            ffi::SpiceDevice {
                kind: ffi::SpiceDeviceKind::MutualInductor,
                name: hier_text(k.name()),
                nodes,
                value: coupling,
                model: String::new(),
                params: vec![],
                ctrl_nodes: vec![],
                ctrl_value: String::new(),
                source: empty_source(),
            }
        }),

        // --- Voltage-controlled / current-controlled switch (S/W) ---
        // nodes() yields nd1, nd2, cnd1, cnd2, model in order (after name).
        SyntaxKind::Switch => ast::Switch::cast(child).map(|sw| {
            let nodes: Vec<String> = sw.nodes().map(|n| n.text()).collect();
            ffi::SpiceDevice {
                kind: ffi::SpiceDeviceKind::Switch,
                name: hier_text(sw.name()),
                nodes,
                value: String::new(),
                model: String::new(),
                params: vec![],
                ctrl_nodes: vec![],
                ctrl_value: String::new(),
                source: empty_source(),
            }
        }),

        // --- Behavioral source (B) ---
        SyntaxKind::Behavioral => ast::Behavioral::cast(child).map(|b| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Behavioral,
            name: hier_text(b.name()),
            nodes: vec![hier_text(b.pos()), hier_text(b.neg())],
            value: String::new(),
            model: String::new(),
            params: b.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),

        // --- JFET (J) ---
        // Accessor layout mirrors MOSFET (drain/gate/source/model via fet_device!).
        SyntaxKind::JFET => ast::JFET::cast(child).map(|j| ffi::SpiceDevice {
            kind: ffi::SpiceDeviceKind::Jfet,
            name: hier_text(j.name()),
            nodes: vec![
                hier_text(j.drain()),
                hier_text(j.gate()),
                hier_text(j.source()),
            ],
            value: rewrite_temp(&j.area().map(|e| e.text()).unwrap_or_default(), true, scope),
            model: hier_text(j.model()),
            params: j.params().map(|p| project_spice_param(&p, scope)).collect(),
            ctrl_nodes: vec![],
            ctrl_value: String::new(),
            source: empty_source(),
        }),

        // --- OSDI device (N) ---
        // nodes() = [conn..., model] (all after name); last is the model reference.
        // Guard: empty list → no panic.
        SyntaxKind::OSDIDevice => ast::OSDIDevice::cast(child).map(|n| {
            let all: Vec<String> = n.nodes().map(|h| h.text()).collect();
            let (terminals, model) = if all.is_empty() {
                (Vec::new(), String::new())
            } else {
                (all[..all.len() - 1].to_vec(), all[all.len() - 1].clone())
            };
            ffi::SpiceDevice {
                kind: ffi::SpiceDeviceKind::Osdi,
                name: hier_text(n.name()),
                nodes: terminals,
                value: String::new(),
                model,
                params: n.params().map(|p| project_spice_param(&p, scope)).collect(),
                ctrl_nodes: vec![],
                ctrl_value: String::new(),
                source: empty_source(),
            }
        }),

        _ => None,
    }
}

/// Project a SPICE `.model` card into a `SpiceModel`.
/// Extracts `level` from the params list if present (case-insensitive match).
fn project_spice_model(m: &ast::Model, scope: &HashSet<String>) -> ffi::SpiceModel {
    let params: Vec<ffi::Param> = m.params().map(|p| project_spice_param(&p, scope)).collect();
    let level = params
        .iter()
        .find(|p| p.name.eq_ignore_ascii_case("level"))
        .map(|p| p.value.clone())
        .unwrap_or_default();
    ffi::SpiceModel {
        name: hier_text(m.name()),
        model_type: tok_text(m.model_type()),
        level,
        params,
    }
}

/// Preserve each branch as its own scope. Conditions depend on the eventual
/// instance parameters, so projecting must neither evaluate nor flatten them.
fn project_spice_conditional(
    c: &ast::IfBlock,
    scope: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::SpiceConditional {
    ffi::SpiceConditional {
        clauses: c
            .cases()
            .map(|clause| ffi::SpiceCondClause {
                condition: rewrite_temp(
                    &clause
                        .condition()
                        .and_then(|condition| condition.expr())
                        .map(|expr| expr.text().trim().to_string())
                        .unwrap_or_default(),
                    true,
                    scope,
                ),
                body: project_spice_block_children(clause.body(), scope, errors),
            })
            .collect(),
    }
}

/// Project a SPICE `.subckt` without moving branch bodies or includes out of
/// their defining scope. Include resolution is the consumer's responsibility.
fn project_spice_subckt(
    s: &ast::Subckt,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::SpiceSubckt {
    // Formal header params (`.subckt … w=1 l=1`) are visible to the body, so
    // seed the body scope with them.
    let header: Vec<ast::Parameter> = s.params().collect();
    let mut scope = inherited.clone();
    for p in &header {
        scope.insert(tok_text(p.name()).to_ascii_lowercase());
    }
    let body = project_spice_block_children(s.body(), &scope, errors);
    // Formal params first, then body `.param` statements, then ordered so each
    // expression follows the parameters it references.
    let mut params: Vec<ffi::Param> = header
        .iter()
        .map(|p| project_spice_param(p, &scope))
        .collect();
    params.extend(body.params);
    let params = ordered(params, errors);
    ffi::SpiceSubckt {
        name: tok_text(s.name()),
        ports: s.ports().map(|p| p.text()).collect(),
        params,
        devices: body.devices,
        models: body.models,
        subckts: body.subckts,
        includes: body.includes,
        conditionals: body.conditionals,
    }
}

/// Collect SPICE `.param` names declared in a scope (including inside `.if`
/// clause bodies), lowercased because SPICE identifiers are case-folded.
fn collect_spice_param_names(children: &[SyntaxNode], out: &mut HashSet<String>) {
    for child in children {
        match child.kind() {
            SyntaxKind::ParamStatement => {
                if let Some(ps) = ast::ParamStatement::cast(child.clone()) {
                    for p in ps.params() {
                        out.insert(tok_text(p.name()).to_ascii_lowercase());
                    }
                }
            }
            SyntaxKind::IfBlock => {
                if let Some(c) = ast::IfBlock::cast(child.clone()) {
                    for clause in c.cases() {
                        let body: Vec<SyntaxNode> = clause.body().collect();
                        collect_spice_param_names(&body, out);
                    }
                }
            }
            _ => {}
        }
    }
}

/// Project an iterator of SPICE children (params, models, subckts, devices,
/// includes, conditionals) into a `SpiceBlock`.
fn project_spice_block_children(
    children: impl Iterator<Item = SyntaxNode>,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::SpiceBlock {
    let children: Vec<SyntaxNode> = children.collect();
    // `.param` declarations (including those inside `.if` clause bodies) are
    // visible to every expression in this scope.
    let mut scope = inherited.clone();
    collect_spice_param_names(&children, &mut scope);

    let mut block = ffi::SpiceBlock {
        params: vec![],
        models: vec![],
        subckts: vec![],
        devices: vec![],
        includes: vec![],
        conditionals: vec![],
    };
    for child in &children {
        match child.kind() {
            SyntaxKind::ParamStatement => {
                if let Some(ps) = ast::ParamStatement::cast(child.clone()) {
                    for p in ps.params() {
                        block.params.push(project_spice_param(&p, &scope));
                    }
                }
            }
            SyntaxKind::Model => {
                if let Some(m) = ast::Model::cast(child.clone()) {
                    block.models.push(project_spice_model(&m, &scope));
                }
            }
            SyntaxKind::Subckt => {
                if let Some(s) = ast::Subckt::cast(child.clone()) {
                    block.subckts.push(project_spice_subckt(&s, &scope, errors));
                }
            }
            SyntaxKind::IncludeStatement => {
                if let Some(inc) = ast::IncludeStatement::cast(child.clone()) {
                    block.includes.push(ffi::Include {
                        path: unquote(&tok_text(inc.path())),
                        section: String::new(),
                    });
                }
            }
            SyntaxKind::LibInclude => {
                if let Some(lib) = ast::LibInclude::cast(child.clone()) {
                    block.includes.push(ffi::Include {
                        path: unquote(&tok_text(lib.path())),
                        section: tok_text(lib.section()),
                    });
                }
            }
            SyntaxKind::IfBlock => {
                if let Some(conditional) = ast::IfBlock::cast(child.clone()) {
                    block.conditionals.push(project_spice_conditional(
                        &conditional,
                        &scope,
                        errors,
                    ));
                }
            }
            _ => {
                if let Some(dev) = project_spice_device(child.clone(), &scope) {
                    block.devices.push(dev);
                }
            }
        }
    }
    block.params = ordered(block.params, errors);
    block
}

/// Project a `SPICENetlistSource` node into a `SpiceBlock`.
fn project_spice_block(
    node: SyntaxNode,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> ffi::SpiceBlock {
    project_spice_block_children(node.children(), inherited, errors)
}

/// The definition-level content shared by a netlist and a subckt body.
#[derive(Default)]
struct Scope {
    params: Vec<ffi::Param>,
    models: Vec<ffi::Model>,
    instances: Vec<ffi::Instance>,
    subckts: Vec<ffi::Subckt>,
    conditionals: Vec<ffi::Conditional>,
    analyses: Vec<ffi::Analysis>,
    saves: Vec<ffi::SaveItem>,
    ics: Vec<ffi::IcItem>,
    globals: Vec<String>,
    includes: Vec<ffi::Include>,
    ahdl_includes: Vec<String>,
    spice_blocks: Vec<ffi::SpiceBlock>,
    functions: Vec<ffi::FunctionDef>,
}

/// Collect parameter names declared inside conditional clause bodies straight
/// from the syntax tree (before projection), so the enclosing definition's
/// rewrite scope already contains them.
fn collect_conditional_names_from_ast(stmts: &[SyntaxNode], out: &mut HashSet<String>) {
    for stmt in stmts {
        if let Some(c) = sast::ConditionalBlock::cast(stmt.clone()) {
            for body in conditional_clause_bodies(&c) {
                for s in &body {
                    if s.kind() == SyntaxKind::Parameters {
                        if let Some(p) = sast::Parameters::cast(s.clone()) {
                            for prm in p.params() {
                                if let Some(t) = prm.name() {
                                    out.insert(t.text().to_string());
                                }
                            }
                        }
                    }
                }
                collect_conditional_names_from_ast(&body, out);
            }
        }
    }
}

/// The statement bodies of every clause of a conditional block, in source order.
fn conditional_clause_bodies(c: &sast::ConditionalBlock) -> Vec<Vec<SyntaxNode>> {
    let mut bodies = Vec::new();
    if let Some(iff) = c.if_clause() {
        bodies.push(iff.body().collect());
    }
    for ei in c.else_ifs() {
        bodies.push(ei.body().collect());
    }
    if let Some(els) = c.else_clause() {
        bodies.push(els.body().collect());
    }
    bodies
}

fn collect_scope(
    stmts: impl Iterator<Item = SyntaxNode>,
    inherited: &HashSet<String>,
    errors: &mut Vec<ffi::ParseError>,
) -> Scope {
    let stmts: Vec<SyntaxNode> = stmts.collect();

    // Visible parameter names: this definition's `parameters` plus any declared
    // inside its conditional clauses. A parameter of that name shadows the
    // `temp` builtin for every expression in this definition.
    let mut scope_names = inherited.clone();
    for stmt in &stmts {
        if stmt.kind() == SyntaxKind::Parameters {
            if let Some(p) = sast::Parameters::cast(stmt.clone()) {
                for prm in p.params() {
                    if let Some(t) = prm.name() {
                        scope_names.insert(t.text().to_string());
                    }
                }
            }
        }
    }
    collect_conditional_names_from_ast(&stmts, &mut scope_names);
    let scope = &scope_names;

    let mut collected = Scope::default();
    for stmt in stmts {
        match stmt.kind() {
            SyntaxKind::Model => {
                if let Some(m) = sast::Model::cast(stmt) {
                    collected.models.push(project_model(&m, scope));
                }
            }
            SyntaxKind::Instance => {
                if let Some(i) = sast::Instance::cast(stmt) {
                    collected.instances.push(project_instance(&i, scope));
                }
            }
            SyntaxKind::Subckt => {
                if let Some(s) = sast::Subckt::cast(stmt) {
                    collected.subckts.push(project_subckt(&s, scope, errors));
                }
            }
            SyntaxKind::FunctionDecl => {
                if let Some(f) = sast::FunctionDecl::cast(stmt) {
                    collected.functions.push(project_function(&f, scope));
                }
            }
            SyntaxKind::Parameters => {
                if let Some(p) = sast::Parameters::cast(stmt) {
                    collected
                        .params
                        .extend(p.params().map(|p| project_param(&p, scope)));
                }
            }
            // Conditionals are projected with the definition's full scope,
            // which already includes their own parameter declarations.
            SyntaxKind::ConditionalBlock => {
                if let Some(c) = sast::ConditionalBlock::cast(stmt) {
                    collected
                        .conditionals
                        .push(project_conditional(&c, scope, errors));
                }
            }
            SyntaxKind::Analysis => {
                if let Some(a) = sast::Analysis::cast(stmt) {
                    collected.analyses.push(project_analysis(&a, scope));
                }
            }
            SyntaxKind::Save => {
                if let Some(s) = sast::Save::cast(stmt) {
                    collected.saves.extend(s.signals().map(|sig| ffi::SaveItem {
                        signal: sig.node().map(|n| n.path()).unwrap_or_default(),
                        modifier: sig.modifier().and_then(|m| m.value()).unwrap_or_default(),
                    }));
                }
            }
            SyntaxKind::Ic => {
                if let Some(ic) = sast::Ic::cast(stmt) {
                    collected.ics.extend(ic.params().map(|p| ffi::IcItem {
                        node: p.node().map(|n| n.path()).unwrap_or_default(),
                        value: p.value_text().unwrap_or_default(),
                    }));
                }
            }
            SyntaxKind::Global => {
                if let Some(g) = sast::Global::cast(stmt) {
                    collected.globals.extend(g.nodes().map(|n| n.path()));
                }
            }
            SyntaxKind::Include => {
                if let Some(inc) = sast::Include::cast(stmt) {
                    collected.includes.push(ffi::Include {
                        path: unquote(&tok_text(inc.path())),
                        section: inc
                            .section()
                            .and_then(|s| s.id())
                            .map(|t| t.text().to_string())
                            .unwrap_or_default(),
                    });
                }
            }
            SyntaxKind::AHDLInclude => {
                if let Some(a) = sast::AHDLInclude::cast(stmt) {
                    collected.ahdl_includes.push(unquote(&tok_text(a.path())));
                }
            }
            SyntaxKind::SPICENetlistSource => {
                let mut inner = Vec::new();
                collected
                    .spice_blocks
                    .push(project_spice_block(stmt, scope, &mut inner));
                errors.extend(inner);
            }
            _ => {}
        }
    }
    collected.params = ordered(collected.params, errors);
    collected
}

fn collect_errors(root: &SyntaxNode) -> Vec<ffi::ParseError> {
    root.descendants_with_tokens()
        .filter(|e| e.kind() == SyntaxKind::Error)
        .map(|e| {
            let r = e.text_range();
            ffi::ParseError {
                start: r.start().into(),
                end: r.end().into(),
            }
        })
        .collect()
}

/// Collect the spans of `Incomplete` nodes. These carry no `Error` token, so
/// they are invisible to `collect_errors`; a consumer that wants to reject a
/// parse whose content was silently dropped must check both.
fn collect_incompletes(root: &SyntaxNode) -> Vec<ffi::ParseError> {
    root.descendants()
        .filter(|e| e.kind() == SyntaxKind::Incomplete)
        .map(|e| {
            let r = e.text_range();
            ffi::ParseError {
                start: r.start().into(),
                end: r.end().into(),
            }
        })
        .collect()
}

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

/// Build the projection's inherited-name set from a caller-supplied list,
/// applying the dialect's case rule — the same rule `rewrite_temp` uses when it
/// looks a name up: SPICE identifiers are case-folded (its `temper` match is
/// case-insensitive), Spectre identifiers are kept verbatim (Spectre is
/// case-sensitive). Exactly one set is allocated per projection call.
fn inherited_names_set(names: &[String], spice: bool) -> HashSet<String> {
    names
        .iter()
        .map(|n| {
            if spice {
                n.to_ascii_lowercase()
            } else {
                n.clone()
            }
        })
        .collect()
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
        params: vec![],
        models: vec![],
        subckts: vec![],
        instances: vec![],
        analyses: vec![],
        saves: vec![],
        ics: vec![],
        globals: vec![],
        includes: vec![],
        ahdl_includes: vec![],
        functions: vec![],
        conditionals: vec![],
        errors: vec![],
        incompletes: vec![],
        spice_blocks: vec![],
    }
}

/// Build a `Netlist` from a collected Spectre scope plus the parse diagnostics.
fn netlist_from_scope(
    scope: Scope,
    errors: Vec<ffi::ParseError>,
    incompletes: Vec<ffi::ParseError>,
) -> ffi::Netlist {
    ffi::Netlist {
        params: scope.params,
        models: scope.models,
        subckts: scope.subckts,
        instances: scope.instances,
        analyses: scope.analyses,
        saves: scope.saves,
        ics: scope.ics,
        globals: scope.globals,
        includes: scope.includes,
        ahdl_includes: scope.ahdl_includes,
        functions: scope.functions,
        conditionals: scope.conditionals,
        errors,
        incompletes,
        spice_blocks: scope.spice_blocks,
    }
}

pub fn parse_netlist(src: &str, language: &str) -> ffi::Netlist {
    parse_netlist_with_inherited_names(src, language, &[])
}

/// Parse `src` with `inherited_names` — the parameter names declared in the
/// enclosing file(s) and their include chain — visible to the projection's
/// temperature-builtin shadowing. This is the additive entry point the consumer
/// uses to reproduce cross-include shadowing without the parser reading files:
/// the consumer resolves includes, collects the names they declare, and passes
/// them here (cheap to call repeatedly — no file I/O, no global state, one set
/// built per call). An empty slice is exactly `parse_netlist`.
pub fn parse_netlist_with_inherited_names(
    src: &str,
    language: &str,
    inherited_names: &[String],
) -> ffi::Netlist {
    let (start_lang, dialect) = match resolve_lang(language) {
        // Spectre start: dialect is only consulted if a `simulator lang=spice`
        // region appears; Ngspice is the harmless default there.
        Some(Lang::Spectre) => (StartLang::Spectre, Dialect::Ngspice),
        Some(Lang::Spice(d)) => (StartLang::Spice, d),
        None => return lang_error_netlist(),
    };
    let inherited = inherited_names_set(inherited_names, matches!(start_lang, StartLang::Spice));
    let root = parse_spectre_with(src, start_lang, dialect);
    let mut errors = collect_errors(&root);
    let incompletes = collect_incompletes(&root);
    let source = sast::SpectreNetlistSource::cast(root).expect("root is SpectreNetlistSource");
    let scope = collect_scope(source.statements(), &inherited, &mut errors);
    netlist_from_scope(scope, errors, incompletes)
}

/// Parse a `.lib` file and project only the named section. SPICE dialects use
/// the `LibStatement`/`.endl` form; `spectre` uses `library`/`section`. The
/// selected section's statements populate the netlist's top-level scope fields
/// (Spectre has no separate block type), so a consumer merges them into the
/// current scope exactly as it would a flat Spectre netlist.
pub fn parse_netlist_lib(src: &str, section: &str, language: &str) -> ffi::Netlist {
    parse_netlist_lib_with_inherited_names(src, section, language, &[])
}

/// Like `parse_netlist_lib`, with `inherited_names` visible to the projection's
/// temperature-builtin shadowing (see `parse_netlist_with_inherited_names`).
pub fn parse_netlist_lib_with_inherited_names(
    src: &str,
    section: &str,
    language: &str,
    inherited_names: &[String],
) -> ffi::Netlist {
    match resolve_lang(language) {
        Some(Lang::Spice(dialect)) => parse_spice_lib(
            src,
            section,
            dialect,
            &inherited_names_set(inherited_names, true),
        ),
        Some(Lang::Spectre) => {
            parse_spectre_lib(src, section, &inherited_names_set(inherited_names, false))
        }
        None => lang_error_netlist(),
    }
}

fn parse_spice_lib(
    src: &str,
    section: &str,
    dialect: Dialect,
    inherited: &HashSet<String>,
) -> ffi::Netlist {
    let root = parse_spice_dialect(src, dialect);
    let mut errors = collect_errors(&root);
    let incompletes = collect_incompletes(&root);

    let mut block = ffi::SpiceBlock {
        params: vec![],
        models: vec![],
        subckts: vec![],
        devices: vec![],
        includes: vec![],
        conditionals: vec![],
    };
    for child in root.children() {
        if child.kind() == SyntaxKind::LibStatement {
            if let Some(lib) = ast::LibStatement::cast(child) {
                let name = tok_text(lib.name());
                if name.eq_ignore_ascii_case(section) {
                    let inner =
                        project_spice_block_children(lib.statements(), inherited, &mut errors);
                    block.params.extend(inner.params);
                    block.models.extend(inner.models);
                    block.subckts.extend(inner.subckts);
                    block.devices.extend(inner.devices);
                    block.includes.extend(inner.includes);
                    block.conditionals.extend(inner.conditionals);
                }
            }
        }
    }
    block.params = ordered(block.params, &mut errors);

    let mut nl = empty_netlist();
    nl.spice_blocks.push(block);
    nl.errors = errors;
    nl.incompletes = incompletes;
    nl
}

/// True when a Spectre `section`'s name matches `wanted` (case-insensitive,
/// mirroring the SPICE `.lib` path).
fn spectre_section_matches(sec: &sast::Section, wanted: &str) -> bool {
    sec.name()
        .map(|t| t.text().eq_ignore_ascii_case(wanted))
        .unwrap_or(false)
}

fn parse_spectre_lib(src: &str, section: &str, inherited: &HashSet<String>) -> ffi::Netlist {
    let root = parse_spectre_with(src, StartLang::Spectre, Dialect::Ngspice);
    let mut errors = collect_errors(&root);
    let incompletes = collect_incompletes(&root);
    let source = sast::SpectreNetlistSource::cast(root).expect("root is SpectreNetlistSource");

    // Collect the matching section's statements. A section is either a child of
    // a top-level `library` or a bare top-level `section`.
    let mut selected: Vec<SyntaxNode> = Vec::new();
    for stmt in source.statements() {
        match stmt.kind() {
            SyntaxKind::Library => {
                if let Some(lib) = sast::Library::cast(stmt) {
                    for sec in lib.sections() {
                        if spectre_section_matches(&sec, section) {
                            selected.extend(sec.body());
                        }
                    }
                }
            }
            SyntaxKind::Section => {
                if let Some(sec) = sast::Section::cast(stmt) {
                    if spectre_section_matches(&sec, section) {
                        selected.extend(sec.body());
                    }
                }
            }
            _ => {}
        }
    }

    let scope = collect_scope(selected.into_iter(), inherited, &mut errors);
    netlist_from_scope(scope, errors, incompletes)
}

#[cfg(test)]
mod tests {
    use super::{
        parse_netlist, parse_netlist_lib, parse_netlist_lib_with_inherited_names,
        parse_netlist_with_inherited_names,
    };

    #[test]
    fn projects_spice_scoped_includes_and_nested_conditionals() {
        let src = include_str!("../tests/fixtures/spice_scopes.cir");
        let nl = parse_netlist(src, "ngspice");
        assert!(nl.errors.is_empty());
        let top = &nl.spice_blocks[0];
        assert!(top.devices.is_empty());
        assert!(top.models.is_empty());
        assert!(top.includes.is_empty());
        assert!(top.conditionals.is_empty());
        assert_eq!(top.subckts.len(), 2);

        let sub = &top.subckts[0];
        assert_eq!(sub.name, "probe");
        assert_eq!(sub.ports, ["d", "g", "s", "b"]);
        assert_eq!(
            sub.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["rfmode", "ng", "m", "width"]
        );
        assert!(
            sub.devices.is_empty(),
            "conditional devices must not be flattened"
        );
        assert!(sub.models.is_empty(), "conditional models must not leak");
        assert!(
            sub.subckts.is_empty(),
            "conditional definitions must not leak"
        );
        assert_eq!(sub.includes.len(), 2);
        assert_eq!(sub.includes[0].path, "shared models.lib");
        assert_eq!(sub.includes[0].section, "");
        assert_eq!(sub.includes[1].path, "corners.lib");
        assert_eq!(sub.includes[1].section, "tt");
        // A repeated reference is meaningful in each subcircuit scope.
        assert_eq!(top.subckts[1].includes.len(), 1);
        assert_eq!(top.subckts[1].includes[0].path, "shared models.lib");

        assert_eq!(sub.conditionals.len(), 1);
        let clauses = &sub.conditionals[0].clauses;
        assert_eq!(clauses.len(), 2);
        assert_eq!(clauses[0].condition, "rfmode == 0");
        assert_eq!(clauses[1].condition, "");
        let normal = &clauses[0].body;
        assert_eq!(normal.params.len(), 1);
        assert_eq!(normal.params[0].name, "branch_scale");
        assert_eq!(normal.params[0].value, "ng*2");
        assert_eq!(normal.models.len(), 1);
        assert_eq!(normal.models[0].name, "local_psp");
        assert_eq!(normal.models[0].model_type, "psp103va");
        assert_eq!(normal.includes.len(), 1);
        assert_eq!(normal.includes[0].path, "normal.inc");
        assert!(normal.devices.is_empty());
        assert_eq!(normal.conditionals.len(), 1);
        let nested = &normal.conditionals[0].clauses;
        assert_eq!(nested.len(), 3);
        assert_eq!(nested[0].condition, "ng == 1");
        assert_eq!(nested[1].condition, "ng == 2");
        assert_eq!(nested[2].condition, "");
        assert_eq!(nested[0].body.devices.len(), 2);
        assert_eq!(
            nested[0].body.devices[0].kind,
            super::ffi::SpiceDeviceKind::Osdi
        );
        assert_eq!(nested[0].body.devices[0].nodes, ["d", "g", "s", "b"]);
        assert_eq!(nested[0].body.devices[0].model, "local_psp");
        assert_eq!(nested[0].body.devices[0].params[0].value, "ng");
        assert_eq!(nested[0].body.devices[0].params[1].value, "m");
        assert_eq!(nested[0].body.devices[1].name, "R1");
        assert_eq!(nested[1].body.devices.len(), 1);
        assert_eq!(nested[1].body.devices[0].params[0].value, "2");
        assert_eq!(nested[2].body.devices.len(), 1);

        let rf = &clauses[1].body;
        assert_eq!(rf.includes.len(), 1);
        assert_eq!(rf.includes[0].section, "rf");
        assert_eq!(rf.subckts.len(), 1);
        assert_eq!(rf.subckts[0].name, "rf_helper");
        assert_eq!(rf.subckts[0].includes[0].path, "shared models.lib");
        assert_eq!(rf.subckts[0].devices[0].name, "Nrf");
        assert_eq!(rf.devices.len(), 1);
        assert_eq!(rf.devices[0].model, "rf_helper");
    }

    #[test]
    fn projects_spice_conditionals_in_top_level_and_library_sections() {
        let body = ".if (1)\nR1 a b 1k\n.endif\n\
                    .if (gain > 2)\n.include \"high.inc\"\n\
                    .else\n.lib \"low.lib\" tt\n.endif\n";
        let source = format!("* t\n{body}");
        let library = format!(
            "* t\n.lib tt\n{body}.endl tt\n\
                               .lib ff\n.if (0)\nRff a b 2k\n.endif\n.endl ff\n"
        );
        for dialect in ["ngspice", "xyce"] {
            for nl in [
                parse_netlist(&source, dialect),
                parse_netlist_lib(&library, "tt", dialect),
            ] {
                assert!(nl.errors.is_empty());
                let block = &nl.spice_blocks[0];
                assert!(block.devices.is_empty());
                assert!(block.includes.is_empty());
                assert_eq!(block.conditionals.len(), 2);
                let first = &block.conditionals[0].clauses;
                assert_eq!(first.len(), 1);
                assert_eq!(first[0].condition, "1");
                assert_eq!(first[0].body.devices.len(), 1);
                assert_eq!(first[0].body.devices[0].name, "R1");
                let second = &block.conditionals[1].clauses;
                assert_eq!(second.len(), 2);
                assert_eq!(second[0].condition, "gain > 2");
                assert_eq!(second[0].body.includes[0].path, "high.inc");
                assert_eq!(second[1].condition, "");
                assert_eq!(second[1].body.includes[0].section, "tt");
            }
            let ff = parse_netlist_lib(&library, "ff", dialect);
            let clause = &ff.spice_blocks[0].conditionals[0].clauses[0];
            // False branches are retained for the consumer, not evaluated here.
            assert_eq!(clause.condition, "0");
            assert_eq!(clause.body.devices[0].name, "Rff");
        }
    }

    #[test]
    fn projects_top_level() {
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             parameters vdd=1.8\n\
             r1 (a b) resistor r=1k\n\
             model nch bsim4 l=1u\n\
             stepResponse tran stop=100ns\n",
            "spectre",
        );
        assert_eq!(nl.params.len(), 1);
        assert_eq!(nl.params[0].name, "vdd");
        assert_eq!(nl.params[0].value, "1.8");
        assert_eq!(nl.instances.len(), 1);
        assert_eq!(nl.instances[0].name, "r1");
        assert_eq!(nl.instances[0].master, "resistor");
        assert_eq!(nl.instances[0].nodes, vec!["a", "b"]);
        assert_eq!(nl.models.len(), 1);
        assert_eq!(nl.models[0].master, "bsim4");
        assert_eq!(nl.analyses.len(), 1);
        assert_eq!(nl.analyses[0].analysis_type, "tran");
    }

    #[test]
    fn projects_user_function() {
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             real distr(real b, real n, real w, real x) {\n\
             return n + x*(w-n) ;\n\
             }\n\
             parameters p=distr(1,2,3,0.5)\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(nl.functions.len(), 1);
        let f = &nl.functions[0];
        assert_eq!(f.name, "distr");
        assert_eq!(f.args, vec!["b", "n", "w", "x"]);
        assert_eq!(f.body, "n + x*(w-n)");
    }

    #[test]
    fn projects_nested_subckt_and_conditional() {
        let nl = parse_netlist(
            "subckt s1 (d g s b)\n\
             parameters l=1u\n\
             if ( l < 0.5u ) {\n\
             m1 (d g s b) shortmod l=l\n\
             } else {\n\
             m2 (d g s b) longmod l=l\n\
             }\n\
             ends s1\n",
            "spectre",
        );
        assert_eq!(nl.subckts.len(), 1);
        let s = &nl.subckts[0];
        assert_eq!(s.name, "s1");
        assert_eq!(s.ports, vec!["d", "g", "s", "b"]);
        assert_eq!(s.params.len(), 1);
        assert_eq!(s.conditionals.len(), 1);
        let cl = &s.conditionals[0].clauses;
        assert_eq!(cl.len(), 2);
        assert_eq!(cl[0].condition, "l < 0.5u");
        assert_eq!(cl[0].body.instances.len(), 1);
        assert_eq!(cl[0].body.instances[0].name, "m1");
        assert_eq!(cl[1].condition, "");
        assert_eq!(cl[1].body.instances.len(), 1);
        assert_eq!(cl[1].body.instances[0].master, "longmod");
    }

    #[test]
    fn projects_multi_statement_conditional_body() {
        // A real PDK-shaped body: an instance plus an `include`, and a
        // brace on the line after the condition.
        let nl = parse_netlist(
            "subckt s1 (a b)\n\
             if (sxcalc > 0)\n\
             {\n\
             xsub a waferBack! sxmodel\n\
             include \"./extra.scs\"\n\
             }\n\
             ends s1\n",
            "spectre",
        );
        assert_eq!(nl.errors.len(), 0);
        let cl = &nl.subckts[0].conditionals[0].clauses;
        assert_eq!(cl.len(), 1);
        assert_eq!(cl[0].condition, "sxcalc > 0");
        assert_eq!(cl[0].body.instances.len(), 1);
        assert_eq!(cl[0].body.instances[0].master, "sxmodel");
        assert_eq!(cl[0].body.includes.len(), 1);
        assert_eq!(cl[0].body.includes[0].path, "./extra.scs");
    }

    #[test]
    fn collects_include_and_errors() {
        let nl = parse_netlist("include \"models.scs\" section=tt\n", "spectre");
        assert_eq!(nl.includes.len(), 1);
        assert_eq!(nl.includes[0].path, "models.scs");
        assert_eq!(nl.includes[0].section, "tt");
    }

    #[test]
    fn projects_spice_block_passives() {
        let nl = super::parse_netlist("* t\nR1 a b 1k\nC1 b 0 1u\nL1 a b 2n\n", "ngspice");
        assert!(nl.errors.is_empty());
        assert_eq!(nl.spice_blocks.len(), 1);
        let d = &nl.spice_blocks[0].devices;
        assert_eq!(d.len(), 3);
        assert_eq!(d[0].kind, super::ffi::SpiceDeviceKind::Resistor);
        assert_eq!(d[0].name, "R1");
        assert_eq!(d[0].nodes, vec!["a", "b"]);
        assert_eq!(d[0].value, "1k");
        assert_eq!(d[1].kind, super::ffi::SpiceDeviceKind::Capacitor);
        assert_eq!(d[1].name, "C1");
        assert_eq!(d[1].nodes, vec!["b", "0"]);
        assert_eq!(d[1].value, "1u");
        assert_eq!(d[2].kind, super::ffi::SpiceDeviceKind::Inductor);
        assert_eq!(d[2].name, "L1");
        assert_eq!(d[2].nodes, vec!["a", "b"]);
        assert_eq!(d[2].value, "2n");
    }

    #[test]
    fn projects_spice_subckt_body_params() {
        // `.param` statements inside a SPICE `.subckt` body must be projected
        // (they were previously dropped). They are appended *after* the formal
        // header params, in source order, since body params may reference the
        // formal ones.
        let nl = super::parse_netlist(
            "* t\n\
             .subckt rcblock a b w=1u l=2u\n\
             .param rint=3k\n\
             .param cint=1p\n\
             R1 a b rint\n\
             .ends\n",
            "ngspice",
        );
        assert!(nl.errors.is_empty(), "unexpected parse errors");
        assert_eq!(nl.spice_blocks.len(), 1);
        let subs = &nl.spice_blocks[0].subckts;
        assert_eq!(subs.len(), 1);
        let s = &subs[0];
        assert_eq!(s.name, "rcblock");
        assert_eq!(s.ports, vec!["a", "b"]);
        // Formal header params (w, l) first, then body `.param`s (rint, cint).
        let names: Vec<&str> = s.params.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, vec!["w", "l", "rint", "cint"]);
        let vals: Vec<&str> = s.params.iter().map(|p| p.value.as_str()).collect();
        assert_eq!(vals, vec!["1u", "2u", "3k", "1p"]);
    }

    #[test]
    fn projects_spice_sources() {
        let nl = super::parse_netlist(
            "* t\nV1 1 0 DC 5 AC 1 PULSE(0 5 1m 1u 1u 4m 10m)\nI1 0 2 DC 1m\n",
            "ngspice",
        );
        assert!(nl.errors.is_empty(), "unexpected parse errors");
        assert_eq!(nl.spice_blocks.len(), 1);
        let d = &nl.spice_blocks[0].devices;
        // V1
        let v = d.iter().find(|x| x.name == "V1").unwrap();
        assert_eq!(v.kind, super::ffi::SpiceDeviceKind::VSource);
        assert_eq!(v.nodes, vec!["1", "0"]);
        assert_eq!(v.source.dc, "5");
        assert_eq!(v.source.ac_mag, "1");
        assert_eq!(v.source.ac_phase, "");
        assert_eq!(v.source.tran_kind.to_lowercase(), "pulse");
        assert_eq!(
            v.source.tran_args,
            vec!["0", "5", "1m", "1u", "1u", "4m", "10m"]
        );
        // I1
        let cur = d.iter().find(|x| x.name == "I1").unwrap();
        assert_eq!(cur.kind, super::ffi::SpiceDeviceKind::ISource);
        assert_eq!(cur.nodes, vec!["0", "2"]);
        assert_eq!(cur.source.dc, "1m");
        assert_eq!(cur.source.tran_kind, "");
        assert!(cur.source.tran_args.is_empty());
    }

    #[test]
    fn projects_spice_semiconductors_and_subckt() {
        let src = "* t\n\
            D1 a k dmod\n\
            M1 d g s b nch w=1u l=0.1u\n\
            Q1 c b e qmod\n\
            X1 in out amp gain=2\n\
            .model dmod d is=1e-14\n\
            .subckt amp in out\n R1 in out 1k\n .ends\n";
        let nl = super::parse_netlist(src, "ngspice");
        let b = &nl.spice_blocks[0];
        // MOSFET
        let m = b.devices.iter().find(|x| x.name == "M1").unwrap();
        assert_eq!(m.kind, super::ffi::SpiceDeviceKind::Mosfet);
        assert_eq!(m.nodes, vec!["d", "g", "s", "b"]);
        assert_eq!(m.model, "nch");
        // Diode
        let d = b.devices.iter().find(|x| x.name == "D1").unwrap();
        assert_eq!(d.kind, super::ffi::SpiceDeviceKind::Diode);
        assert_eq!(d.nodes, vec!["a", "k"]);
        assert_eq!(d.model, "dmod");
        // BJT: nodes() = [Q1, c, b, e, qmod]; skip name, last is model
        let q = b.devices.iter().find(|x| x.name == "Q1").unwrap();
        assert_eq!(q.kind, super::ffi::SpiceDeviceKind::Bjt);
        assert_eq!(q.nodes, vec!["c", "b", "e"]);
        assert_eq!(q.model, "qmod");
        // SubcktCall: nodes() = [X1, in, out, amp]; skip name, last is master
        let x = b.devices.iter().find(|x| x.name == "X1").unwrap();
        assert_eq!(x.kind, super::ffi::SpiceDeviceKind::SubcktCall);
        assert_eq!(x.nodes, vec!["in", "out"]);
        assert_eq!(x.model, "amp");
        // .model card
        assert_eq!(b.models.len(), 1);
        assert_eq!(b.models[0].name, "dmod");
        assert_eq!(b.models[0].model_type, "d");
        // .subckt definition
        assert_eq!(b.subckts.len(), 1);
        assert_eq!(b.subckts[0].name, "amp");
        assert_eq!(b.subckts[0].ports, vec!["in", "out"]);
        assert_eq!(b.subckts[0].devices.len(), 1);
        assert_eq!(b.subckts[0].devices[0].name, "R1");
    }

    /// Regression test: malformed BJT/SubcktCall with ≤1 node children must not
    /// panic. A bare `Q1` and `X1` (no terminals, no model) hit the len<2 guard
    /// and project with empty nodes and model rather than slicing out-of-bounds.
    #[test]
    fn projects_malformed_bjt_subckt_no_panic() {
        // Bare device names only — parser produces minimal/error AST nodes with
        // very few HierarchialNode children, exercising the len<2 guard.
        let nl = super::parse_netlist("* t\nQ1\nX1\n", "ngspice");
        // Must not panic; any devices that project should have empty nodes/model.
        for dev in &nl.spice_blocks[0].devices {
            assert!(
                dev.nodes.is_empty(),
                "malformed device should have no nodes"
            );
            assert!(
                dev.model.is_empty(),
                "malformed device should have no model"
            );
        }
    }

    /// E/F/G/H controlled sources + .include collected into SpiceBlock.includes.
    #[test]
    fn projects_controlled_sources_and_include() {
        let src = "* t\n\
            E1 out 0 in 0 2.0\n\
            G1 o 0 a b 1e-3\n\
            F1 o 0 vsense 10\n\
            .include \"models.spice\"\n";
        let nl = super::parse_netlist(src, "ngspice");
        let b = &nl.spice_blocks[0];
        // E1 — voltage-controlled voltage source
        let e = b.devices.iter().find(|x| x.name == "E1").unwrap();
        assert_eq!(e.kind, super::ffi::SpiceDeviceKind::Vcvs);
        assert_eq!(e.nodes, vec!["out", "0"]);
        assert_eq!(e.ctrl_nodes, vec!["in", "0"]);
        assert_eq!(e.ctrl_value, "2.0");
        // G1 — voltage-controlled current source
        let g = b.devices.iter().find(|x| x.name == "G1").unwrap();
        assert_eq!(g.kind, super::ffi::SpiceDeviceKind::Vccs);
        assert_eq!(g.ctrl_nodes, vec!["a", "b"]);
        assert_eq!(g.ctrl_value, "1e-3");
        // F1 — current-controlled current source
        let f = b.devices.iter().find(|x| x.name == "F1").unwrap();
        assert_eq!(f.kind, super::ffi::SpiceDeviceKind::Cccs);
        assert_eq!(f.ctrl_nodes, vec!["vsense"]);
        assert_eq!(f.ctrl_value, "10");
        // include
        assert_eq!(b.includes.len(), 1);
        assert_eq!(b.includes[0].path, "models.spice");
    }

    /// Smoke test: one netlist covering every device kind; checks count + no errors.
    #[test]
    fn projects_full_breadth() {
        let src = "* full breadth\n\
            R1 a b 1k\n\
            C1 b 0 1u\n\
            L1 a b 2n\n\
            V1 vin 0 DC 5\n\
            I1 0 vin DC 1m\n\
            D1 a k dmod\n\
            M1 d g s b nch w=1u\n\
            Q1 c b e qmod\n\
            X1 in out amp\n\
            E1 out 0 in 0 2.0\n\
            G1 o 0 a b 1e-3\n\
            F1 o 0 vsense 10\n\
            H1 o 0 vsense 5\n\
            K1 L1 L2 0.9\n\
            S1 nd1 nd2 cnd1 cnd2 smod ON\n\
            B1 out 0 V=1+2\n\
            J1 d g s jmod\n\
            .model dmod d\n\
            .subckt amp in out\n R1 in out 1k\n .ends\n\
            .include \"models.spice\"\n\
            .lib \"mylib.sp\" tt\n";
        let nl = super::parse_netlist(src, "ngspice");
        assert!(
            nl.errors.is_empty(),
            "unexpected parse errors: {} error(s)",
            nl.errors.len()
        );
        let b = &nl.spice_blocks[0];
        // 17 device lines (R C L V I D M Q X E G F H K S B J)
        assert_eq!(
            b.devices.len(),
            17,
            "expected 17 devices, got {}: {:?}",
            b.devices.len(),
            b.devices.iter().map(|d| &d.name).collect::<Vec<_>>()
        );
        // .include + .lib → 2 includes
        assert_eq!(b.includes.len(), 2);
        assert_eq!(b.includes[0].path, "models.spice");
        assert_eq!(b.includes[1].path, "mylib.sp");
        assert_eq!(b.includes[1].section, "tt");
    }

    #[test]
    fn parse_lib_extracts_section() {
        let src = "* SKY130 models\n\
.lib tt\n\
.param vth0_nfet = 0.4\n\
.model nfet nmos level=54 version=4.5 tnom=27\n\
+ vth0=0.3\n\
.endl tt\n\
.lib ff\n\
.param vth0_nfet = 0.35\n\
.model nfet_ff nmos level=54\n\
.endl ff\n";
        let nl = parse_netlist_lib(src, "tt", "ngspice");
        assert!(
            nl.errors.is_empty(),
            "parse errors: {} error(s)",
            nl.errors.len()
        );
        assert_eq!(nl.spice_blocks.len(), 1);
        let b = &nl.spice_blocks[0];
        assert_eq!(b.params.len(), 1);
        assert_eq!(b.params[0].name, "vth0_nfet");
        assert_eq!(b.params[0].value, "0.4");
        assert_eq!(b.models.len(), 1);
        assert_eq!(b.models[0].name, "nfet");

        let nl_ff = parse_netlist_lib(src, "ff", "ngspice");
        let b_ff = &nl_ff.spice_blocks[0];
        assert_eq!(b_ff.params.len(), 1);
        assert_eq!(b_ff.params[0].value, "0.35");
        assert_eq!(b_ff.models[0].name, "nfet_ff");

        let nl_miss = parse_netlist_lib(src, "ss", "ngspice");
        let b_miss = &nl_miss.spice_blocks[0];
        assert!(b_miss.params.is_empty());
        assert!(b_miss.models.is_empty());
    }

    #[test]
    fn parse_lib_case_insensitive() {
        let src = "* t\n.lib TT\n.param x=1\n.endl TT\n";
        let nl = parse_netlist_lib(src, "tt", "ngspice");
        assert_eq!(nl.spice_blocks[0].params.len(), 1);
    }

    #[test]
    fn start_language_dispatch() {
        // SPICE block, then switch to Spectre, then a Spectre instance.
        let src = "* title\nR1 a b 1k\nsimulator lang=spectre\nr2 (a b) resistor r=2k\n";

        // Start in SPICE: the leading SPICE block parses cleanly; after the
        // `simulator lang=spectre` switch, control returns to the Spectre driver
        // and the trailing Spectre instance r2 projects. (SPICE-device projection
        // of the leading block is deferred to Task 7 — not asserted here.)
        let spice = parse_netlist(src, "ngspice");
        assert!(spice.errors.is_empty(), "spice-start should parse cleanly");
        assert_eq!(spice.instances.len(), 1);
        assert_eq!(spice.instances[0].name, "r2");

        // Start in Spectre: the same leading SPICE line `R1 a b 1k` is invalid
        // Spectre and produces error node(s).
        let spectre = parse_netlist(src, "spectre");
        assert!(
            !spectre.errors.is_empty(),
            "spectre-start should error on the SPICE line"
        );
    }

    #[test]
    fn clean_parse_has_no_incompletes() {
        let nl = super::parse_netlist(
            "simulator lang=spectre\nr1 (a b) resistor r=1k\n",
            "spectre",
        );
        assert!(nl.errors.is_empty());
        assert!(
            nl.incompletes.is_empty(),
            "clean netlist must not be incomplete"
        );
    }

    #[test]
    fn unterminated_subckt_is_incomplete_but_not_error() {
        // An unterminated `subckt` at EOF yields an `Incomplete` node with no
        // `Error` token, so it is invisible to `nl.errors`. Consumers must check
        // `nl.incompletes` to notice that content was dropped.
        let src = "simulator lang=spectre\nsubckt s (a b)\nparameters + x=1\n";
        let nl = super::parse_netlist(src, "spectre");
        assert!(
            nl.errors.is_empty(),
            "unexpected error token(s): {}",
            nl.errors.len()
        );
        assert_eq!(
            nl.incompletes.len(),
            1,
            "expected one Incomplete (the subckt)"
        );
    }

    #[test]
    fn parse_netlist_ngspice_projects_spice_block() {
        let nl = super::parse_netlist("* t\nR1 a b 1k\n", "ngspice");
        assert!(
            nl.errors.is_empty(),
            "unexpected errors: {}",
            nl.errors.len()
        );
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
        // A SPICE title line is required (first line is always consumed as title).
        let src = "* t\n.lib tt\nR1 a b 1k\n.endl tt\n.lib ff\nR1 a b 2k\n.endl ff\n";
        let nl = super::parse_netlist_lib(src, "tt", "ngspice");
        assert!(nl.errors.is_empty());
        assert_eq!(nl.spice_blocks.len(), 1);
        assert_eq!(nl.spice_blocks[0].devices.len(), 1);
    }

    #[test]
    fn parse_netlist_lib_spectre_extracts_section() {
        // A Spectre library: two sections with different models/params. Selecting
        // a section must pick exactly that section's content (case-insensitive).
        let src = "\
simulator lang=spectre\n\
library global\n\
section tt\n\
parameters vth0_nfet = 0.4\n\
model nfet bsimsoi type=n\n\
endsection tt\n\
section ff\n\
parameters vth0_nfet = 0.35\n\
model nfet_ff bsimsoi type=n\n\
endsection ff\n\
endlibrary global\n";
        let nl = parse_netlist_lib(src, "tt", "spectre");
        assert!(
            nl.errors.is_empty(),
            "unexpected errors: {}",
            nl.errors.len()
        );
        assert_eq!(nl.params.len(), 1);
        assert_eq!(nl.params[0].name, "vth0_nfet");
        assert_eq!(nl.params[0].value, "0.4");
        assert_eq!(nl.models.len(), 1);
        assert_eq!(nl.models[0].name, "nfet");

        // Case-insensitive match, matching the SPICE `.lib` path.
        let nl_ff = parse_netlist_lib(src, "FF", "spectre");
        assert_eq!(nl_ff.params[0].value, "0.35");
        assert_eq!(nl_ff.models[0].name, "nfet_ff");

        // Missing section: empty scope, no errors.
        let nl_miss = parse_netlist_lib(src, "ss", "spectre");
        assert!(nl_miss.errors.is_empty());
        assert!(nl_miss.params.is_empty());
        assert!(nl_miss.models.is_empty());
    }

    #[test]
    fn parse_netlist_lib_spectre_bare_top_level_section() {
        let src =
            "simulator lang=spectre\nsection cmos\nmodel nch bsimsoi type=n\nendsection cmos\n";
        let nl = parse_netlist_lib(src, "cmos", "spectre");
        assert!(nl.errors.is_empty());
        assert_eq!(nl.models.len(), 1);
        assert_eq!(nl.models[0].master, "bsimsoi");
    }

    #[test]
    fn projects_spectre_subckt_include_with_section() {
        // A subckt-level `include … section=` (the PDK FET-wrapper shape) must be
        // projected, not dropped.
        let src = "\
simulator lang=spectre\n\
inline subckt adnfet (d g s x)\n\
parameters\n\
+ w=1u\n\
include \"./lle_param_rxj.lib.scs\" section=adnfet\n\
model m1 nch\n\
ends adnfet\n";
        let nl = parse_netlist(src, "spectre");
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(nl.subckts.len(), 1);
        let s = &nl.subckts[0];
        assert_eq!(s.includes.len(), 1);
        assert_eq!(s.includes[0].path, "./lle_param_rxj.lib.scs");
        assert_eq!(s.includes[0].section, "adnfet");
        assert!(
            nl.includes.is_empty(),
            "subckt include must not leak to top level"
        );
    }

    // --- Change 1: dialect temperature builtins ---

    #[test]
    fn rewrites_spectre_temp_builtin_in_parameters() {
        // A free `temp` becomes `$temp`; a call name that merely starts with
        // `temp` is not touched, but a `temp` argument to another call is.
        let nl = parse_netlist(
            "simulator lang=spectre\nparameters tp=temp+25 rate=tempo(temp)\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let val = |name: &str| {
            nl.params
                .iter()
                .find(|p| p.name == name)
                .map(|p| p.value.clone())
                .unwrap()
        };
        assert_eq!(val("tp"), "$temp+25");
        assert_eq!(val("rate"), "tempo($temp)");
    }

    #[test]
    fn declared_spectre_parameter_shadows_temp() {
        let nl = parse_netlist(
            "simulator lang=spectre\nparameters temp=300 vt=P_K*temp/P_Q\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        // `temp` is declared, so it keeps its binding everywhere in the scope.
        let tmp = nl.params.iter().find(|p| p.name == "temp").unwrap();
        assert_eq!(tmp.value, "300");
        let vt = nl.params.iter().find(|p| p.name == "vt").unwrap();
        assert_eq!(vt.value, "P_K*temp/P_Q");
        // Ordering must place `temp` before `vt` (which references it).
        assert_eq!(
            nl.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["temp", "vt"]
        );
    }

    #[test]
    fn spectre_temp_inside_access_or_string_is_untouched() {
        // `temp` inside v()/i() names a node/branch; inside quotes it is a
        // string literal. Neither is a free identifier.
        let nl = parse_netlist(
            "simulator lang=spectre\nparameters p1=v(temp) p2=i(temp) s=\"temp\"\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let val = |name: &str| {
            nl.params
                .iter()
                .find(|p| p.name == name)
                .map(|p| p.value.clone())
                .unwrap()
        };
        assert_eq!(val("p1"), "v(temp)");
        assert_eq!(val("p2"), "i(temp)");
        assert_eq!(val("s"), "\"temp\"");
    }

    #[test]
    fn spectre_temp_call_name_is_not_rewritten() {
        let nl = parse_netlist("simulator lang=spectre\nparameters c=temp(1)\n", "spectre");
        // The call name itself is a function, not the ambient-temperature builtin.
        let c = nl.params.iter().find(|p| p.name == "c").unwrap();
        assert_eq!(c.value, "temp(1)");
    }

    #[test]
    fn spectre_temp_conditional_declaration_shadows() {
        // A `temp` declared in a conditional clause joins the definition's
        // translation namespace, so it shadows the builtin for the whole
        // definition (mirroring the consumer's scope model).
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             parameters y=temp\n\
             if (1) {\n\
             parameters temp=1\n\
             }\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let y = nl.params.iter().find(|p| p.name == "y").unwrap();
        assert_eq!(y.value, "temp", "conditional declaration must shadow");
        assert_eq!(nl.conditionals[0].clauses[0].body.params[0].value, "1");
    }

    #[test]
    fn spectre_conditional_condition_is_rewritten() {
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             parameters x=1\n\
             if (temp > 0) {\n\
             r1 (a b) resistor r=1k\n\
             }\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(nl.conditionals[0].clauses[0].condition, "$temp > 0");
    }

    #[test]
    fn spectre_function_bodies_are_rewritten_with_arg_scope() {
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             real f(real a) {\n\
             return a*temp ;\n\
             }\n\
             real g(real temp) {\n\
             return temp*2 ;\n\
             }\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let f = nl.functions.iter().find(|f| f.name == "f").unwrap();
        assert_eq!(f.body, "a*$temp");
        let g = nl.functions.iter().find(|f| f.name == "g").unwrap();
        assert_eq!(g.body, "temp*2", "a `temp` argument keeps its binding");
    }

    #[test]
    fn rewrite_temp_in_instance_and_model_params() {
        let nl = parse_netlist(
            "simulator lang=spectre\n\
             model nch bsim4 tnom=temp\n\
             x1 (a b) nch w=temp\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(nl.models[0].params[0].value, "$temp");
        assert_eq!(nl.instances[0].params[0].value, "$temp");
    }

    #[test]
    fn rewrites_spectre_temper_not_at_all() {
        // Spectre's builtin is `temp`; `temper` is an ordinary identifier there.
        let nl = parse_netlist("simulator lang=spectre\nparameters x=temper\n", "spectre");
        assert_eq!(nl.params[0].value, "temper");
    }

    #[test]
    fn rewrites_spice_temper_builtin() {
        let nl = parse_netlist("* t\n.param tp=temper\nR1 a b tp\n", "ngspice");
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let b = &nl.spice_blocks[0];
        assert_eq!(b.params[0].name, "tp");
        assert_eq!(b.params[0].value, "$temp");
    }

    #[test]
    fn declared_spice_parameter_shadows_temper() {
        let nl = parse_netlist(
            "* t\n.param temper=27\n.param tp=temper\nR1 a b tp\n",
            "ngspice",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let b = &nl.spice_blocks[0];
        let tp = b.params.iter().find(|p| p.name == "tp").unwrap();
        assert_eq!(tp.value, "temper", "declared `temper` keeps its binding");
    }

    #[test]
    fn spice_native_temp_is_left_alone() {
        // `temp` is not the SPICE builtin (that is `temper`) and only the SPECTRE
        // dialect rewrites `temp`.
        let nl = parse_netlist("* t\n.param t=temp\nR1 a b t\n", "ngspice");
        let b = &nl.spice_blocks[0];
        assert_eq!(b.params[0].value, "temp");
    }

    #[test]
    fn spice_temper_in_device_and_model_params() {
        let nl = parse_netlist(
            "* t\nR1 a b 1k tc1=temper\n.model dm d is=temper\n",
            "ngspice",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let b = &nl.spice_blocks[0];
        assert_eq!(b.devices[0].params[0].value, "$temp");
        assert_eq!(b.models[0].params[0].value, "$temp");
    }

    // --- Change 2: dependency-ordered parameter lists ---

    #[test]
    fn spectre_parameters_are_dependency_ordered() {
        // `b` is declared before `a` but references it: ordering resolves the
        // forward reference without touching the values.
        let nl = parse_netlist("simulator lang=spectre\nparameters b=a*2 a=1\n", "spectre");
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(
            nl.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["a", "b"]
        );
    }

    #[test]
    fn spectre_independent_parameters_keep_source_order() {
        let nl = parse_netlist(
            "simulator lang=spectre\nparameters a=1 b=2 c=3\n",
            "spectre",
        );
        assert_eq!(
            nl.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["a", "b", "c"]
        );
    }

    #[test]
    fn spectre_self_reference_is_not_a_cycle() {
        // `x=x+1` denotes the enclosing scope: no edge, so this is not a cycle
        // and `x` is still ordered before `y`.
        let nl = parse_netlist(
            "simulator lang=spectre\nparameters y=x*2 x=x+1\n",
            "spectre",
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        assert_eq!(
            nl.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["x", "y"]
        );
        let x = nl.params.iter().find(|p| p.name == "x").unwrap();
        assert_eq!(x.value, "x+1");
    }

    #[test]
    fn spectre_parameter_cycle_is_reported() {
        let nl = parse_netlist("simulator lang=spectre\nparameters a=b b=a\n", "spectre");
        assert!(
            !nl.errors.is_empty(),
            "a parameter cycle must surface through the FFI errors field"
        );
        // Parameters are kept (in source order), not dropped.
        assert_eq!(
            nl.params
                .iter()
                .map(|p| p.name.as_str())
                .collect::<Vec<_>>(),
            ["a", "b"]
        );
    }

    #[test]
    fn spice_parameters_are_dependency_ordered() {
        let nl = parse_netlist("* t\n.param b=a*2\n.param a=1\nR1 x y b\n", "ngspice");
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let b = &nl.spice_blocks[0];
        assert_eq!(
            b.params.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(),
            ["a", "b"]
        );
    }

    #[test]
    fn spice_subckt_parameters_are_dependency_ordered() {
        let nl = parse_netlist(
            "* t\n.subckt rcblock a b w=1u l=2u\n.param rint=3k\n.param cint=1p\nR1 a b rint\n.ends\n",
            "ngspice",
        );
        let s = &nl.spice_blocks[0].subckts[0];
        // Independent parameters keep source order (header first, then body).
        assert_eq!(
            s.params.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(),
            ["w", "l", "rint", "cint"]
        );
    }

    // --- Inherited (enclosing include-chain) parameter names ---

    #[test]
    fn spectre_inherited_temp_shadows_builtin() {
        // Two-file contract: the included file declares `temp`; the enclosing
        // file's free `temp` must keep that binding, not become the ambient
        // temperature builtin. With an empty inherited set it is the builtin.
        let enclosing = "simulator lang=spectre\nparameters y=temp+1\n";
        let alone = parse_netlist(enclosing, "spectre");
        let y = alone.params.iter().find(|p| p.name == "y").unwrap();
        assert_eq!(
            y.value, "$temp+1",
            "empty inherited set rewrites to builtin"
        );

        let inherited =
            parse_netlist_with_inherited_names(enclosing, "spectre", &["temp".to_string()]);
        let y = inherited.params.iter().find(|p| p.name == "y").unwrap();
        assert_eq!(y.value, "temp+1", "inherited `temp` shadows the builtin");
    }

    #[test]
    fn spectre_inherited_names_are_case_sensitive() {
        // Spectre identifiers are case-sensitive: an inherited `TEMP` must not
        // shadow the lowercase builtin `temp`.
        let nl = parse_netlist_with_inherited_names(
            "simulator lang=spectre\nparameters y=temp\n",
            "spectre",
            &["TEMP".to_string()],
        );
        assert_eq!(nl.params[0].value, "$temp");
    }

    #[test]
    fn spectre_inherited_combines_with_local_declaration() {
        // Inherited `temp` plus a locally declared `k` are both preserved in
        // the same expression (the visible scope is their union).
        let nl = parse_netlist_with_inherited_names(
            "simulator lang=spectre\nparameters k=2 y=temp+k\n",
            "spectre",
            &["temp".to_string()],
        );
        let y = nl.params.iter().find(|p| p.name == "y").unwrap();
        assert_eq!(y.value, "temp+k");
    }

    #[test]
    fn spectre_inherited_combines_with_conditional_declaration() {
        // A `temp` declared in a conditional clause joins the definition scope;
        // an unrelated inherited name (`a`) is honoured at the same time.
        let nl = parse_netlist_with_inherited_names(
            "simulator lang=spectre\n\
             parameters y=temp+a\n\
             if (1) {\n\
             parameters temp=1\n\
             }\n",
            "spectre",
            &["a".to_string()],
        );
        assert!(nl.errors.is_empty(), "errors: {}", nl.errors.len());
        let y = nl.params.iter().find(|p| p.name == "y").unwrap();
        assert_eq!(y.value, "temp+a");
    }

    #[test]
    fn spice_inherited_temper_shadows_builtin_case_folded() {
        // SPICE equivalent: an include declares `temper`; the enclosing `.param`
        // keeps the declared binding. The inherited name is case-folded, so its
        // casing does not matter.
        let enclosing = "* t\n.param tp=temper\nR1 a b tp\n";
        let alone = parse_netlist(enclosing, "ngspice");
        assert_eq!(alone.spice_blocks[0].params[0].value, "$temp");

        let inherited =
            parse_netlist_with_inherited_names(enclosing, "ngspice", &["TEMPER".to_string()]);
        let tp = inherited.spice_blocks[0]
            .params
            .iter()
            .find(|p| p.name == "tp")
            .unwrap();
        assert_eq!(tp.value, "temper", "inherited `temper` shadows the builtin");
    }

    #[test]
    fn lib_entry_point_accepts_inherited_names() {
        // The `.lib` entry point threads the inherited set as well.
        let src = "\
simulator lang=spectre\n\
library global\n\
section tt\n\
parameters y=temp\n\
endsection tt\n\
endlibrary global\n";
        let alone = parse_netlist_lib(src, "tt", "spectre");
        assert_eq!(alone.params[0].value, "$temp");

        let inherited =
            parse_netlist_lib_with_inherited_names(src, "tt", "spectre", &["temp".to_string()]);
        assert_eq!(inherited.params[0].value, "temp");
    }
}

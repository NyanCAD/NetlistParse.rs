//! Spectre parse harness: parse one `.scs` file (optionally recursing through
//! `include`/`ahdl_include`), and report parse errors and a rough inventory.
//!
//! Usage:
//!   spectre_parse [-r] <file.scs>

use std::collections::BTreeMap;
use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use netlist_syntax::ast::AstNode;
use netlist_syntax::spectre_ast::{AHDLInclude, Include, SpectreNetlistSource};
use netlist_syntax::syntax_kind::SyntaxKind;
use netlist_syntax::{parse_spectre, SyntaxNode};

#[derive(Default)]
struct Stats {
    files: usize,
    errors: usize,
    incompletes: usize,
    subckts: usize,
    models: usize,
    instances: usize,
    params: usize,
    includes: usize,
    ahdl_includes: usize,
    kinds: BTreeMap<String, usize>,
}

fn unquote(s: &str) -> String {
    let t = s.trim();
    let t = t.strip_prefix('"').unwrap_or(t);
    let t = t.strip_suffix('"').unwrap_or(t);
    t.to_string()
}

fn line_of(src: &str, byte: u32) -> usize {
    src.as_bytes()[..(byte as usize).min(src.len())]
        .iter()
        .filter(|&&b| b == b'\n')
        .count()
        + 1
}

fn snippet(src: &str, start: u32, end: u32) -> String {
    let s = (start as usize).min(src.len());
    let e = (end as usize).max(s + 1).min(src.len());
    let mut lo = s;
    while lo > 0 && src.as_bytes()[lo - 1] != b'\n' {
        lo -= 1;
    }
    let line = src[lo..e].trim().to_string();
    if line.len() > 100 {
        format!("{}…", &line[..100])
    } else {
        line
    }
}

fn walk_kinds(node: &SyntaxNode, stats: &mut Stats) {
    for child in node.children() {
        let label = format!("{:?}", child.kind());
        *stats.kinds.entry(label).or_insert(0) += 1;
        walk_kinds(&child, stats);
    }
}

fn collect_includes(root: &SyntaxNode, incs: &mut Vec<(PathBuf, String)>) {
    for node in root.descendants() {
        if node.kind() == SyntaxKind::Include {
            if let Some(inc) = Include::cast(node) {
                if let Some(path) = inc.path() {
                    let section = inc
                        .section()
                        .and_then(|s| s.id())
                        .map(|t| t.text().to_string())
                        .unwrap_or_default();
                    incs.push((PathBuf::from(unquote(path.text())), section));
                }
            }
        }
    }
}

fn collect_ahdl(root: &SyntaxNode, out: &mut Vec<String>) {
    for node in root.descendants() {
        if node.kind() == SyntaxKind::AHDLInclude {
            if let Some(a) = AHDLInclude::cast(node) {
                if let Some(path) = a.path() {
                    out.push(unquote(path.text()));
                }
            }
        }
    }
}

fn parse_file(path: &Path, recurse: bool, visited: &mut HashSet<PathBuf>, stats: &mut Stats) {
    let abs = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    if !visited.insert(abs.clone()) {
        return;
    }
    let src = match std::fs::read_to_string(&abs) {
        Ok(s) => s,
        Err(e) => {
            println!("  [missing] {}: {}", abs.display(), e);
            return;
        }
    };
    stats.files += 1;

    let root = parse_spectre(&src);
    let source = match SpectreNetlistSource::cast(root.clone()) {
        Some(s) => s,
        None => {
            println!("  [root] {}: not a SpectreNetlistSource", abs.display());
            return;
        }
    };

    let mut file_errors = 0usize;
    let mut file_incompletes = 0usize;
    for node in root.descendants_with_tokens() {
        match node.kind() {
            SyntaxKind::Error => {
                file_errors += 1;
                stats.errors += 1;
                let r = node.text_range();
                let (s, e) = (u32::from(r.start()), u32::from(r.end()));
                println!(
                    "  ERROR {}:{}..{} line {}: {}",
                    abs.display(),
                    s,
                    e,
                    line_of(&src, s),
                    snippet(&src, s, e)
                );
            }
            SyntaxKind::Incomplete => {
                stats.incompletes += 1;
                file_incompletes += 1;
                let r = node.text_range();
                let (s, e) = (u32::from(r.start()), u32::from(r.end()));
                println!(
                    "  INCOMPLETE {}:{}..{} line {}: {}",
                    abs.display(),
                    s,
                    e,
                    line_of(&src, s),
                    snippet(&src, s, e)
                );
            }
            SyntaxKind::Subckt => stats.subckts += 1,
            SyntaxKind::Model => stats.models += 1,
            SyntaxKind::Instance => stats.instances += 1,
            SyntaxKind::Analysis => {}
            SyntaxKind::Parameter => stats.params += 1,
            _ => {}
        }
    }
    walk_kinds(&root, stats);

    let mut incs = Vec::new();
    collect_includes(&root, &mut incs);
    let mut ahdls = Vec::new();
    collect_ahdl(&root, &mut ahdls);
    stats.includes += incs.len();
    stats.ahdl_includes += ahdls.len();

    let _ = source;
    if recurse {
        for (inc, _section) in &incs {
            let base = abs.parent().unwrap_or_else(|| Path::new("."));
            let child = base.join(inc);
            println!("  include {}", child.display());
            parse_file(&child, recurse, visited, stats);
        }
        for a in &ahdls {
            let base = abs.parent().unwrap_or_else(|| Path::new("."));
            let child = base.join(a);
            println!("  ahdl_include {} [not parsed as netlist]", child.display());
        }
    }

    println!(
        "FILE {} errors={} incompletes={}",
        abs.display(),
        file_errors,
        file_incompletes
    );
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let mut recurse = false;
    let mut file: Option<String> = None;
    for a in args.by_ref() {
        match a.as_str() {
            "-r" | "--recurse" => recurse = true,
            _ => {
                file = Some(a);
                break;
            }
        }
    }
    let file = match file {
        Some(f) => f,
        None => {
            eprintln!("usage: spectre_parse [-r] <file.scs>");
            return ExitCode::FAILURE;
        }
    };

    println!("=== spectre_parse {} (recurse={}) ===", file, recurse);
    let mut stats = Stats::default();
    let mut visited = HashSet::new();
    parse_file(Path::new(&file), recurse, &mut visited, &mut stats);

    println!("\n=== summary ===");
    println!(
        "files={} errors={} incompletes={} subckts={} models={} instances={} params={} includes={} ahdl_includes={}",
        stats.files,
        stats.errors,
        stats.incompletes,
        stats.subckts,
        stats.models,
        stats.instances,
        stats.params,
        stats.includes,
        stats.ahdl_includes
    );

    if stats.errors == 0 && stats.incompletes == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

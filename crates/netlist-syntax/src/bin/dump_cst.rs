//! Rust side of the differential test: parse a SPICE or Spectre file and print
//! the canonical CST dump (see `netlist_syntax::dump`).
//!
//! Usage: `dump_cst <file> [ngspice|hspice|pspice|xyce|spectre]`

use std::process::ExitCode;

fn main() -> ExitCode {
    let path = match std::env::args().nth(1) {
        Some(p) => p,
        None => {
            eprintln!("usage: dump_cst <file> [ngspice|hspice|pspice|xyce|spectre]");
            return ExitCode::FAILURE;
        }
    };
    let lang = std::env::args().nth(2);
    let src = match std::fs::read_to_string(&path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error reading {path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let tree = if lang.as_deref() == Some("spectre") {
        // `.scs` corpus files open in Spectre, `.cir` in SPICE (mirrors the
        // differential test's `start_lang`); either may switch via `lang=`.
        let start_lang = if path.ends_with(".cir") {
            netlist_syntax::StartLang::Spice
        } else {
            netlist_syntax::StartLang::Spectre
        };
        netlist_syntax::parse_spectre_with(&src, start_lang, netlist_syntax::Dialect::Ngspice)
    } else {
        let dialect = match lang.as_deref() {
            Some("hspice") => netlist_syntax::Dialect::Hspice,
            Some("pspice") => netlist_syntax::Dialect::Pspice,
            Some("xyce") => netlist_syntax::Dialect::Xyce,
            _ => netlist_syntax::Dialect::Ngspice,
        };
        netlist_syntax::parse_spice_dialect(&src, dialect)
    };
    print!("{}", netlist_syntax::dump::dump(&tree));
    ExitCode::SUCCESS
}

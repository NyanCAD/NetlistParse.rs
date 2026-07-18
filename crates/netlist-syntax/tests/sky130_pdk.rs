//! Integration tests: parse real SkyWater SKY130 PDK model files and inverter
//! netlists. The PDK file tests are skipped when the sibling checkout
//! `skywater-pdk-libs-sky130_fd_pr` is not present.

use netlist_syntax::{parse_spice, parse_spectre, SyntaxKind, SyntaxNode};
use rowan::NodeOrToken;
use std::fs;
use std::path::{Path, PathBuf};

fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn pdk_root() -> Option<PathBuf> {
    let p = project_root().join("../skywater-pdk-libs-sky130_fd_pr");
    p.is_dir().then_some(p)
}

fn vacask_root() -> Option<PathBuf> {
    let p = project_root().join("../VACASK");
    p.is_dir().then_some(p)
}

fn error_count(tree: &SyntaxNode) -> usize {
    tree.descendants_with_tokens()
        .filter(|el| {
            let kind = match el {
                NodeOrToken::Node(n) => n.kind(),
                NodeOrToken::Token(t) => t.kind(),
            };
            kind == SyntaxKind::Error || kind == SyntaxKind::Incomplete
        })
        .count()
}

// ── Corpus inverter netlists (always run) ─────────────────────────────

#[test]
fn spice_inverter_roundtrip() {
    let src = fs::read_to_string(project_root().join("tests/corpus/sky130_inverter.sp")).unwrap();
    let tree = parse_spice(&src);
    assert_eq!(tree.text().to_string(), src, "lossless roundtrip failed");
    let errors = error_count(&tree);
    assert_eq!(errors, 0, "expected 0 parse errors, got {errors}");
}

#[test]
fn spectre_inverter_roundtrip() {
    let src = fs::read_to_string(
        project_root().join("tests/corpus/spectre/instance/sky130_inverter.scs"),
    )
    .unwrap();
    let tree = parse_spectre(&src);
    assert_eq!(tree.text().to_string(), src, "lossless roundtrip failed");
    let errors = error_count(&tree);
    assert_eq!(errors, 0, "expected 0 parse errors, got {errors}");
}

// ── SkyWater PDK SPICE model files (skipped when PDK absent) ──────────

fn collect_spice_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            collect_spice_files(&path, out);
        } else if path.extension().and_then(|s| s.to_str()) == Some("spice") {
            out.push(path);
        }
    }
}

#[test]
fn sky130_combined_models_roundtrip() {
    let pdk = match pdk_root() {
        Some(p) => p,
        None => {
            eprintln!("SKIP: skywater-pdk-libs-sky130_fd_pr not found, skipping PDK tests");
            return;
        }
    };

    let models_dir = pdk.join("combined_models");
    assert!(models_dir.is_dir(), "combined_models/ not found in PDK");

    let mut files = Vec::new();
    collect_spice_files(&models_dir, &mut files);
    files.sort();
    assert!(!files.is_empty(), "no .spice files found");

    let mut total = 0usize;
    let mut total_errors = 0usize;
    let mut failures = Vec::new();

    for path in &files {
        let src = fs::read_to_string(path).unwrap();
        let tree = parse_spice(&src);
        total += 1;

        if tree.text().to_string() != src {
            failures.push(format!("{}: lossless roundtrip FAILED", path.display()));
            continue;
        }

        let errors = error_count(&tree);
        total_errors += errors;
    }

    assert!(
        failures.is_empty(),
        "roundtrip failures:\n{}",
        failures.join("\n")
    );
    eprintln!(
        "sky130 combined_models: {total} files parsed, {total_errors} total error nodes"
    );
}

#[test]
fn sky130_nfet_pfet_models_clean() {
    let pdk = match pdk_root() {
        Some(p) => p,
        None => {
            eprintln!("SKIP: skywater-pdk-libs-sky130_fd_pr not found");
            return;
        }
    };

    let models = [
        "combined_models/continuous/models_fet/sky130_fd_pr__nfet_01v8.spice",
        "combined_models/continuous/models_fet/sky130_fd_pr__pfet_01v8.spice",
        "combined_models/continuous/models_global.spice",
        "combined_models/continuous/models_fet.spice",
        "combined_models/continuous/parameters_fet_tt.spice",
    ];

    for rel in &models {
        let path = pdk.join(rel);
        if !path.exists() {
            eprintln!("SKIP: {rel} not found");
            continue;
        }
        let src = fs::read_to_string(&path).unwrap();
        let tree = parse_spice(&src);
        assert_eq!(
            tree.text().to_string(),
            src,
            "lossless roundtrip failed for {rel}"
        );
        let errors = error_count(&tree);
        eprintln!("{rel}: {errors} error nodes ({} lines)", src.lines().count());
    }
}

// ── SkyWater PDK top-level lib file ───────────────────────────────────

#[test]
fn sky130_lib_file_roundtrip() {
    let pdk = match pdk_root() {
        Some(p) => p,
        None => {
            eprintln!("SKIP: skywater-pdk-libs-sky130_fd_pr not found");
            return;
        }
    };

    let path = pdk.join("combined_models/sky130.lib.spice");
    let src = fs::read_to_string(&path).unwrap();
    let tree = parse_spice(&src);
    assert_eq!(tree.text().to_string(), src, "lossless roundtrip failed");
    let errors = error_count(&tree);
    eprintln!(
        "sky130.lib.spice: {} lines, {errors} error nodes",
        src.lines().count()
    );
}

// ── VACASK demo netlists (Spectre-like syntax) ────────────────────────

#[test]
fn vacask_inverter_demos_roundtrip() {
    let vacask = match vacask_root() {
        Some(p) => p,
        None => {
            eprintln!("SKIP: VACASK not found, skipping VACASK tests");
            return;
        }
    };

    let demos = [
        "demo/spice/mos1inv.sim",
        "demo/spice/bsim3v3inv.sim",
        "demo/spice/bsim4v8inv.sim",
        "test/test_inverter.sim",
    ];

    for rel in &demos {
        let path = vacask.join(rel);
        if !path.exists() {
            eprintln!("SKIP: {rel} not found");
            continue;
        }
        let src = fs::read_to_string(&path).unwrap();
        let tree = parse_spectre(&src);
        assert_eq!(
            tree.text().to_string(),
            src,
            "lossless roundtrip failed for {rel}"
        );
        let errors = error_count(&tree);
        eprintln!("{rel}: {errors} error nodes ({} lines)", src.lines().count());
    }
}

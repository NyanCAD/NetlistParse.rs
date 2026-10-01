use netlist_syntax::spectre_parser::parse_vacask;
use netlist_syntax::syntax_kind::SyntaxKind;

#[test]
fn native_library_extensions_are_lossless_and_complete() {
    let source = "load \"device.va\"\nsection tt\nparameters scale=1\nsubckt device(p n)\nparameters m=1\nmodel rm resistor (\n r=(m>0?max(m,1k):missing)\n)\n@if (m==1)\nr1 (p n) rm $mfactor=m\n@else\nr2 (p n) rm r=$temp+273\n@end\nends\nendsection\n";
    let tree = parse_vacask(source);
    assert_eq!(tree.text().to_string(), source);
    assert!(!tree
        .descendants()
        .any(|n| n.kind() == SyntaxKind::Incomplete));
    assert!(!tree
        .descendants_with_tokens()
        .any(|n| n.kind() == SyntaxKind::Error));
}

#[test]
fn truncated_native_libraries_recover_losslessly() {
    for source in [
        "section tt\n",
        "load\n",
        "@if (1)\n",
        "subckt device(p n)\nparameters x=\n",
    ] {
        let tree = parse_vacask(source);
        assert_eq!(tree.text().to_string(), source);
        assert!(tree
            .descendants()
            .any(|n| n.kind() == SyntaxKind::Incomplete));
    }
}

"""Smoke tests for the netlist_parser Python bindings."""
import pathlib

import netlist_parser
import pytest

FIXTURES = pathlib.Path(__file__).parent / "fixtures"


def find_child(node, kind):
    """Return first direct child with the given kind, or None."""
    for child in node.children:
        if child.kind == kind:
            return child
    return None


def walk(node):
    """Yield node and all descendants (pre-order)."""
    yield node
    for child in node.children:
        yield from walk(child)


def test_parse_basic():
    src = "* t\nR1 a b 1k\n"
    root = netlist_parser.parse_spice(src)

    # Root kind
    assert root.kind == "SPICENetlistSource", f"expected SPICENetlistSource, got {root.kind!r}"

    # Direct children include Title and Resistor
    kinds = [c.kind for c in root.children]
    assert "Title" in kinds, f"no Title child; children: {kinds}"
    assert "Resistor" in kinds, f"no Resistor child; children: {kinds}"

    # Resistor span is (4, 14) — covers "R1 a b 1k\n"
    resistor = find_child(root, "Resistor")
    assert resistor is not None
    assert resistor.span == (4, 14), f"Resistor span: {resistor.span}"

    # Deep walk: find Identifier token with text "R1"
    idents = [n for n in walk(root) if n.kind == "Identifier" and n.text == "R1"]
    assert len(idents) >= 1, "no Identifier with text 'R1' found in tree"


def test_errors():
    # ".model" without a name is malformed — parser emits exactly one Error token
    src = "* t\n.model\nR2 a b 2k\n"
    root = netlist_parser.parse_spice(src)
    errs = netlist_parser.errors(root)
    assert len(errs) == 1, f"expected 1 error, got {len(errs)}: {errs}"
    start, end = errs[0]
    assert end - start == 1, f"expected error span width 1, got {end - start}: ({start}, {end})"


def test_parse_netlist_spectre():
    src = "parameters r=1k\nmodel rm resistor r=r\nr1 (a 0) rm\n"
    root = netlist_parser.parse_netlist(src, language="spectre")
    assert root.kind == "SpectreNetlistSource"
    assert root.text == src
    assert netlist_parser.errors(root) == []
    assert find_child(root, "Model") is not None
    assert find_child(root, "Instance") is not None


def test_parse_netlist_accepts_all_spice_dialects():
    src = "* t\nR1 a b 1k\n"
    for language in ("ngspice", "hspice", "pspice", "xyce"):
        root = netlist_parser.parse_netlist(src, language=language)
        assert root.text == src
        assert netlist_parser.errors(root) == [], f"{language}: {netlist_parser.errors(root)}"


def test_parse_netlist_handles_lang_switch():
    # The mixed parser follows a mid-file `simulator lang=` handoff.
    src = "simulator lang=spice\n* t\nR1 a b 1k\n"
    root = netlist_parser.parse_netlist(src, language="spectre")
    assert root.text == src
    assert netlist_parser.errors(root) == []


def test_parse_netlist_rejects_unknown_language():
    with pytest.raises(ValueError, match="unknown dialect"):
        netlist_parser.parse_netlist("* t\n", language="bogus")


def test_parse_netlist_ihp_ngspice_fixture():
    # Verbatim IHP SG13G2 ngspice model library (Apache-2.0, IHP PDK Authors):
    # https://github.com/IHP-GmbH/IHP-Open-PDK/blob/main/ihp-sg13g2/libs.tech/ngspice/models/diodes.lib
    # Exercises the constructs the Circulax/IHP integration feeds in: grouped
    # multiline `.params` blocks, `.model` cards and `.subckt`/`.ends` bodies.
    src = (FIXTURES / "ihp_sg13g2_diodes.lib").read_text()
    root = netlist_parser.parse_netlist(src, language="ngspice")
    assert root.text == src
    assert netlist_parser.errors(root) == []

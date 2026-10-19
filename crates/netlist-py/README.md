# Python binding

The extension exposes the lossless CST through `Node.kind`, `Node.text`,
`Node.span`, `Node.children` and `Node.is_trivia`.

```python
import netlist_parser

root = netlist_parser.parse_spice('* circuit\nR1 a b 1k\n')          # pure SPICE
root = netlist_parser.parse_netlist('r1 (a b) resistor r=1k\n', language='spectre')
errors = netlist_parser.errors(root)
```

`parse_netlist(source, language="ngspice")` is the dialect-aware entry point and
mirrors `netlist-cxx::parse_netlist`. `language` accepts `"ngspice"`, `"hspice"`,
`"pspice"`, `"xyce"` and `"spectre"` (case-insensitive); any other value raises
`ValueError`. It uses the mixed parser, so a mid-file `simulator lang=` switch is
handled, and the CST is always rooted at `SpectreNetlistSource` (a SPICE region
appears as a nested `SPICENetlistSource`). `parse_spice` remains a pure-SPICE
convenience wrapper rooted at `SPICENetlistSource`.

This is a lossless syntax parser, not a simulator: callers resolve includes,
scopes, expressions, model aliases and circuit topology themselves. Inspect both
Error tokens and Incomplete nodes before consuming syntax.

Build with `uv pip install ./crates/netlist-py`. Test with
`python -m pytest crates/netlist-py/tests`.

# Python binding

The extension exposes the lossless CST through `Node.kind`, `Node.text`,
`Node.span`, `Node.children` and `Node.is_trivia`.

```python
import netlist_parser

root = netlist_parser.parse_spice('* circuit\nR1 a b 1k\n')
root = netlist_parser.parse_spectre('r1 (a b) resistor r=1k\n')
errors = netlist_parser.errors(root)
```

The optional `parse_spectre(source, vacask=True)` mode supports native VACASK
library extensions: load directives, section/endsection blocks, grouped model
parameters, `$` parameter names and @if/@else/@end conditional bodies.
Default Spectre parsing retains its existing grammar and differential behavior.
The extension is not a simulator: callers resolve includes, scopes, expressions,
model aliases and circuit topology themselves. Inspect both Error tokens and
Incomplete nodes before consuming syntax.

Build with `uv pip install ./crates/netlist-py`. Test with
`python -m pytest crates/netlist-py/tests` and
`cargo test -p netlist-syntax --test vacask --test spectre_differential`.

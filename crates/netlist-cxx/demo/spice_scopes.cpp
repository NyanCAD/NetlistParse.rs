// Exercise recursive SPICE scopes through the generated C++ interface.
#include "netlist_cxx/netlist.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: spice_scopes <fixture.cir>");
        std::ifstream file(argv[1]);
        require(file.good(), "cannot open fixture");
        std::ostringstream buffer;
        buffer << file.rdbuf();
        const std::string source = buffer.str();
        const auto nl = netlist::parse_netlist(source, "ngspice");
        require(nl.errors.empty(), "parse errors");
        require(nl.spice_blocks.size() == 1, "missing SPICE block");
        const auto& top = nl.spice_blocks[0];
        require(top.subckts.size() == 2, "missing subcircuits");
        const auto& sub = top.subckts[0];
        require(sub.devices.empty(), "conditional devices leaked into subcircuit");
        require(sub.includes.size() == 2, "lost local includes");
        require(std::string(sub.includes[0].path) == "shared models.lib", "wrong include path");
        require(top.subckts[1].includes.size() == 1, "lost sibling include");
        require(sub.conditionals.size() == 1, "lost conditional");
        const auto& clauses = sub.conditionals[0].clauses;
        require(clauses.size() == 2, "lost if/else branch");
        require(std::string(clauses[0].condition) == "rfmode == 0", "wrong condition");
        require(clauses[1].condition.empty(), "else has a condition");
        require(clauses[0].body.conditionals.size() == 1, "lost nested conditional");
        const auto& nested = clauses[0].body.conditionals[0].clauses;
        require(nested.size() == 3, "lost elseif branch");
        require(nested[0].body.devices.size() == 2, "lost conditional devices");
        require(nested[0].body.devices[0].kind == netlist::SpiceDeviceKind::Osdi, "lost OSDI device");
        require(clauses[1].body.subckts.size() == 1, "lost branch-local definition");
        require(clauses[1].body.subckts[0].includes.size() == 1, "lost nested local include");
        std::cout << "SPICE scoped includes and nested conditional bodies preserved\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

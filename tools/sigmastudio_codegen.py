#!/usr/bin/env python3
"""Generate a small C++ address map from a SigmaStudio PARAM.h file."""

import argparse
import json
import re
import sys
from pathlib import Path

INPUT_BANDS_PER_EQ = 8
OUTPUT_BANDS_PER_EQ = 12


def normalize(text):
    """normalize SigmaStudio module names"""
    return re.sub(r"\s+", "", text.strip().upper())


def _macro_definitions(block, prefix):
    """yield (macro name, value) definitions belonging to one module"""
    for line in block.splitlines():
        match = re.match(r"\s*#define\s+(\w+)\s+(.+?)\s*$", line)
        if not match:
            continue

        macro, rhs = match.groups()
        if macro.startswith(prefix):
            yield macro, rhs


def _parse_eq_bands(block, module, band_count):
    """read the exported coefficient addresses (5) for each band

    format: TARGB2{band}{coefficient}, e.g. band 10 coefficient 2 = TARGB2101 (2 - 1 = 1, 0-indexed)"""

    prefix = f"MOD_{module}_"
    bands = {}

    for macro, rhs in _macro_definitions(block, prefix):
        if not macro.endswith("_ADDR"):
            continue

        parameter_name = macro[len(prefix):-len("_ADDR")]
        match = re.search(r"TARGB2(\d+)([0-4])$", parameter_name)

        if not match:
            continue

        band_number = int(match.group(1))
        coefficient_index = int(match.group(2))

        if not 1 <= band_number <= band_count:
            raise ValueError(
                f"Band {band_number} outside 1..{band_count} in {module}: {macro}"
            )

        # only store the address of the first coefficient
        if coefficient_index != 0:
            continue

        key = band_number - 1

        if key in bands:
            raise ValueError(f"Duplicate EQ coefficient address: {macro}")

        bands[key] = int(rhs, 0)

    expected = set(range(band_count))

    missing = sorted(expected - bands.keys())

    if missing:
        raise ValueError(
            f"Invalid EQ coefficient layout in {module}: missing first coefficient addresses for bands {missing}")

    return [bands[band] for band in range(band_count)]


def _find_single_address(block, module, suffix, role):
    """find the address macro whose parameter name has the given suffix"""
    prefix = f"MOD_{module}_"
    addresses = []

    for macro, rhs in _macro_definitions(block, prefix):
        if not macro.endswith("_ADDR"):
            continue
        parameter_name = macro[len(prefix):-len("_ADDR")]
        if parameter_name.endswith(suffix):
            addresses.append(int(rhs, 0))

    if len(addresses) != 1:
        raise ValueError(f"Expected one {role} addresses in {module}, found {len(addresses)}")
    return addresses[0]


def _module_role(child, direction):
    """return the logical role for a processing comment"""
    if child == "VOLUME" or child.startswith("VOLUME_"):
        return "volume"
    if child == "USER_MUTE" or child.startswith("USER_MUTE_"):
        return "mute"

    if direction == "input":
        if child == "EQ_A" or child.startswith("EQ_A_"):
            return "eq_a"
        if child == "EQ_B" or child.startswith("EQ_B_"):
            return "eq_b"
    else:
        # The output modules are named PEQ_A/PEQ_B in this PARAM.h export.
        if child == "PEQ_A" or child.startswith("PEQ_A_"):
            return "eq_a"
        if child == "PEQ_B" or child.startswith("PEQ_B_"):
            return "eq_b"

    return None


def parse_param_header(path):
    """read parameter addresses from SigmaStudio header"""
    text = path.read_text(encoding="utf-8", errors="replace")
    groups = {
        "input": {},
        "output": {}
    }
    init_mute = None

    # SigmaStudio starts each module section with a comment
    blocks = re.split(r"(?=/\*\s*Module\s)", text)

    for block in blocks:
        comment = re.search(
            r"/\*\s*Module\s+(.+?)\s+-\s*.*?\*/",
            block,
            re.DOTALL,
        )
        count = re.search(
            r"^\s*#define\s+MOD_([A-Z0-9_]+)_COUNT\b",
            block,
            re.MULTILINE,
        )

        if not comment or not count:
            continue

        child = normalize(comment.group(1))
        module = count.group(1)

        # handle the global INIT_MUTE parameter separately
        if module == "INIT_MUTE":
            init_mute = _find_single_address(block, module, "_MUTE", "init mute")
            continue

        if module.startswith("INPUT_PROCESSING"):
            direction = "input"
            bands_per_eq = INPUT_BANDS_PER_EQ
        elif module.startswith("OUTPUT_PROCESSING"):
            direction = "output"
            bands_per_eq = OUTPUT_BANDS_PER_EQ
        else:
            continue

        role = _module_role(child, direction)
        if role is None:
            continue

        # remove the child module name to get its parent name
        suffix = "_" + child
        if not module.endswith(suffix):
            continue
        parent = module[:-len(suffix)]

        group = groups[direction].setdefault(parent, {})
        if role in group:
            raise ValueError(f"Duplicate {role} address: {parent}")

        if role == "volume":
            group[role] = _find_single_address(block, module, "TARGET", "volume")
        elif role == "mute":
            group[role] = _find_single_address(block, module, "_MUTE", "mute")
        else:
            group[role] = _parse_eq_bands(block, module, bands_per_eq)

    return groups, init_mute


def sort_key(name):
    match = re.search(r"_(\d+)$", name)
    return int(match.group(1)) if match else 1, name.lower()


def _complete_modules(groups, direction, bands_per_eq):
    required = {"volume", "mute", "eq_a", "eq_b"}
    modules = []

    for name, values in groups[direction].items():
        missing = required - values.keys()
        if missing:
            raise ValueError(f"Incomplete {direction} processing module {name}: missing {', '.join(sorted(missing))}")
        for role in ("eq_a", "eq_b"):
            if len(values[role]) != bands_per_eq:
                raise ValueError(f"Expected {bands_per_eq} {role.upper()} bands in {name}, found {len(values[role])}")
        modules.append((name, values))

    modules.sort(key=lambda item: sort_key(item[0]))
    return modules


def _format_module(name, values):
    """Format one channel; EQ_A precedes EQ_B."""
    peq_bands = values["eq_a"] + values["eq_b"]
    refs = ", ".join(f"{{0x{address:04X}}}" for address in peq_bands)

    return [
        "    {",
        f"        {json.dumps(name)},",
        f"        {{0x{values['volume']:04X}}},",
        f"        {{0x{values['mute']:04X}}},",
        f"        {{ {refs} }},",
        "    },",
    ]


def generate(param_header):
    groups, init_mute = parse_param_header(param_header)

    input_modules = _complete_modules(groups, "input", INPUT_BANDS_PER_EQ)
    output_modules = _complete_modules(groups, "output", OUTPUT_BANDS_PER_EQ)

    if not input_modules and not output_modules:
        raise ValueError("No input or output processing modules found.")

    input_peq_band_count = INPUT_BANDS_PER_EQ * 2
    output_peq_band_count = OUTPUT_BANDS_PER_EQ * 2

    output = [
        "// Generated by sigmastudio_codegen.py",
        f"// Source: {param_header.name}",
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <iterator>",
        "",
        "namespace adau1452 {",
        "",
        "struct ParameterRef {",
        "    std::uint16_t address;",
        "};",
        "",
        f"inline constexpr std::size_t input_eq_a_band_count = {INPUT_BANDS_PER_EQ};",
        f"inline constexpr std::size_t input_eq_b_offset = {INPUT_BANDS_PER_EQ};",
        f"inline constexpr std::size_t input_peq_band_count = {input_peq_band_count};",
        f"inline constexpr std::size_t output_peq_a_band_count = {OUTPUT_BANDS_PER_EQ};",
        f"inline constexpr std::size_t output_peq_b_offset = {OUTPUT_BANDS_PER_EQ};",
        f"inline constexpr std::size_t output_peq_band_count = {output_peq_band_count};",
        "",
        "template <std::size_t BandCount>",
        "struct ProcessingModule {",
        "    const char* name;",
        "    ParameterRef volume;",
        "    ParameterRef mute;",
        "    // [band]; address of the first coefficient",
        "    ParameterRef peq[BandCount];",
        "};",
        "",
        "inline constexpr ParameterRef init_mute =",
        f"    {{0x{init_mute:04X}}};",
        "",
    ]

    for direction, modules, band_count in (
            ("input", input_modules, input_peq_band_count),
            ("output", output_modules, output_peq_band_count),
    ):
        array_name = f"{direction}_processing"

        output.append(f"inline constexpr ProcessingModule<{band_count}> {array_name}[] = {{")
        for name, values in modules:
            output.extend(_format_module(name, values))
        output.extend([
            "};",
            "",
        ])

    output.extend(["} // namespace adau1452", ""])
    return "\n".join(output)


def main():
    parser = argparse.ArgumentParser(
        description="Convert SigmaStudio parameter addresses to a C++ header."
    )

    parser.add_argument(
        "param_header",
        type=Path,
        help="SigmaStudio *_PARAM.h file",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("./components/adau1452/include/dsp_mapping.hpp"),
        help="Output C++ header path",
    )

    args = parser.parse_args()

    try:
        result = generate(args.param_header)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result, encoding="utf-8", newline="\n")

    except (OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

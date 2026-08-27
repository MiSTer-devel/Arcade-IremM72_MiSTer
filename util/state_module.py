#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "anytree",
#   "sympy"
# ]
# ///

"""
State Module Generator - Automatic Savestate Support for Verilog Modules

This tool analyzes Verilog modules and automatically injects savestate functionality
by adding ports and logic for state saving/restoring. It's used in the MiSTer FPGA
project to enable save states in emulated systems.

Usage:
    python state_module.py <root_module> <output_file> <verilog_files...>

Example:
    python state_module.py tv80s rtl/tv80_auto_ss.v rtl/tv80/*.v
"""

import sys
import subprocess
import argparse
import logging
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple, Set, Union

import verible_verilog_syntax
from sympy import simplify


# Configure logging
logger = logging.getLogger(__name__)


class Config:
    """Configuration constants for state module generation"""
    PREFIX = 'auto_ss'

    # Chunked savestate interface configuration
    DATA_WIDTH = 32      # Configurable chunk size in bits
    STATE_WIDTH = 16     # Bits for state index within device
    DEVICE_WIDTH = 8     # Bits for device index
    MAX_PACK_SIZE = 32   # Maximum bits to pack together

    RESET_SIGNALS = frozenset([
        "rst", "nrst", "rstn", "n_rst", "rst_n",
        "reset", "nreset", "resetn", "n_reset", "reset_n"
    ])
    VERIBLE_FORMAT_PARAMS = [
        "--port_declarations_alignment=align",
        "--named_port_alignment=align",
        "--assignment_statement_alignment=align",
        "--formal_parameters_alignment=align",
        "--module_net_variable_alignment=align",
        "--named_parameter_alignment=align",
        "--verify_convergence=false"
    ]


# Custom exceptions
class StateModuleError(Exception):
    """Base exception for state module generation errors"""
    pass


class ModuleNotFoundError(StateModuleError):
    """Raised when specified module is not found"""
    pass


class InvalidVerilogError(StateModuleError):
    """Raised when Verilog parsing fails"""
    pass


def setup_logging(verbose: bool = False) -> None:
    """Configure logging based on verbosity"""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def find_path(root: verible_verilog_syntax.Node, tags: List[str]) -> Optional[verible_verilog_syntax.Node]:
    """Navigate through AST nodes following a path of tags"""
    node = root

    for tag in tags:
        if node is None:
            return None
        node = node.find({"tag": tag})

    return node


def format_output(s: str) -> str:
    """Format Verilog output using verible-verilog-format"""
    try:
        proc = subprocess.run(
            ["verible-verilog-format", "-"] + Config.VERIBLE_FORMAT_PARAMS,
            stdout=subprocess.PIPE,
            input=s,
            encoding="utf-8",
            check=True
        )
        return proc.stdout
    except subprocess.CalledProcessError as e:
        logger.warning(f"Formatting failed: {e}")
        return s  # Return unformatted on error


def preprocess_inputs(paths: List[Path]) -> str:
    """Preprocess Verilog files with USE_AUTO_SS define"""
    res = []
    for p in paths:
        try:
            proc = subprocess.run(
                ["verible-verilog-preprocessor", "preprocess", "+define+USE_AUTO_SS=1", str(p)],
                stdout=subprocess.PIPE,
                encoding="utf-8",
                check=True
            )
            res.append(proc.stdout)
        except subprocess.CalledProcessError as e:
            raise InvalidVerilogError(f"Failed to preprocess {p}: {e}")

    return '\n'.join(res)


def validate_inputs(files: List[Path]) -> None:
    """Validate input files exist and are readable"""
    for file in files:
        if not file.exists():
            raise FileNotFoundError(f"Input file not found: {file}")
        if not file.is_file():
            raise ValueError(f"Not a file: {file}")
        if file.suffix not in ['.v', '.sv']:
            logger.warning(f"Unexpected file extension: {file}")


def output_file(fp, node, enable_format: bool = True):
    """Output modified AST to file"""
    begin = None
    s = ""
    for tok in verible_verilog_syntax.PreOrderTreeIterator(node):
        if isinstance(tok, InsertNode):
            s += f"\n{tok.text}\n"
        elif isinstance(tok, verible_verilog_syntax.TokenNode):
            if begin is None:
                begin = tok.start
            end = tok.end
            s += tok.syntax_data.source_code[begin:end].decode("utf-8")
            begin = end

    s += "\n\n\n"

    if enable_format:
        s = format_output(s)

    fp.write(s)


class InsertNode(verible_verilog_syntax.LeafNode):
    """Node for inserting text into the AST"""
    def __init__(self, contents: str, parent: verible_verilog_syntax.Node):
        super().__init__(parent)
        self.contents = contents

    @property
    def text(self) -> str:
        return self.contents


def add_text_before(node: verible_verilog_syntax.Node, text: str):
    """Insert text before a node in the AST"""
    parent = node.parent
    children = list(parent.children)
    idx = children.index(node)
    new_node = InsertNode(text, parent)
    children.insert(idx, new_node)
    parent.children = children


def add_text_after(node: verible_verilog_syntax.Node, text: str):
    """Insert text after a node in the AST"""
    parent = node.parent
    children = list(parent.children)
    idx = children.index(node)
    new_node = InsertNode(text, parent)
    children.insert(idx + 1, new_node)
    parent.children = children


@dataclass
class Dimension:
    """Represents a dimension in Verilog (e.g., [7:0])"""
    end: Union[int, str]
    begin: Optional[Union[int, str]] = None

    def __post_init__(self):
        self.end = simplify(self.end)
        self.begin = simplify(self.begin) if self.begin else self.begin
        self._normalize()

    def _normalize(self):
        """Normalize dimension ordering"""
        if self.end == 0 and self.begin:
            self.end, self.begin = self.begin, self.end

    @property
    def size(self) -> Union[int, str]:
        """Calculate dimension size"""
        if self.begin is not None:
            return simplify(f"1+({self.end})-({self.begin})")
        return self.end

    def to_verilog(self) -> str:
        """Convert to Verilog syntax"""
        if self.begin == self.end:
            return f"[{self.begin}]"
        return f"[{self.begin} +: {self.size}]"

    def __str__(self) -> str:
        return self.to_verilog()

    def __eq__(self, other) -> bool:
        return self.end == other.end and self.begin == other.begin


@dataclass
class Register:
    """Represents a Verilog register with dimensions"""
    name: str
    packed: Optional[Dimension] = None
    unpacked: Optional[Dimension] = None
    allocated: Optional[Dimension] = None

    # Chunked savestate allocation
    chunk_index: Optional[int] = None
    bit_offset: Optional[int] = None
    chunk_count: Optional[int] = None

    def size(self) -> str:
        """Calculate total size of register"""
        if self.packed and self.unpacked:
            return f"({self.packed.size})*({self.unpacked.size})"
        elif self.packed:
            return str(self.packed.size)
        elif self.unpacked:
            return str(self.unpacked.size)
        else:
            return "1"

    def known_size(self) -> bool:
        """Check if size can be evaluated to a constant"""
        try:
            int(self.size())
            return True
        except (ValueError, TypeError):
            return False

    def allocate(self, offset: str) -> str:
        """Allocate register at given offset and return next offset"""
        self.allocated = Dimension(f"({offset})+({self.size()})-1", offset)
        return f"({offset})+({self.size()})"

    def unpacked_dim(self, index: str) -> Dimension:
        """Get dimension for accessing unpacked array element"""
        base = self.allocated.begin
        return Dimension(
            f"(({self.packed.size}) * (({index}) + 1)) + ({base}) - 1",
            f"(({self.packed.size}) * ({index})) + ({base})"
        )

    def is_param_sized(self) -> bool:
        """Check if unpacked dimension uses parameters"""
        if not self.unpacked:
            return False
        try:
            int(self.unpacked.size)
            return False
        except (ValueError, TypeError):
            return True

    def is_packable(self) -> bool:
        """Check if register can be packed with others"""
        if self.unpacked:
            return False  # Arrays get their own chunks
        if not self.known_size():
            return False  # Parameter-sized registers can't be packed
        try:
            size = int(self.size())
            return size <= Config.MAX_PACK_SIZE
        except (ValueError, TypeError):
            return False

    def get_packed_size(self) -> int:
        """Get size in bits for packing (must be known size)"""
        if not self.packed:
            return 1
        try:
            return int(self.packed.size)
        except (ValueError, TypeError):
            raise ValueError(f"Cannot get packed size for parameter-sized register {self.name}")

    def allocate_chunk(self, chunk_idx: int, bit_offset: int = 0, chunk_count: int = 1):
        """Allocate register to specific chunk(s)"""
        self.chunk_index = chunk_idx
        self.bit_offset = bit_offset
        self.chunk_count = chunk_count

    def get_bounds_check(self, base_offset=None) -> str:
        """Generate bounds checking expression for unpacked arrays"""
        if not self.unpacked:
            base = base_offset if base_offset is not None else self.chunk_index
            return f"auto_ss_state_idx == {base}"

        # For arrays, use the chunk_index (which might be symbolic)
        start_expr = str(self.chunk_index) if hasattr(self, 'chunk_index') else str(base_offset or 0)

        if self.is_param_sized():
            # Use sympy to simplify the expression
            end_expr = f"({start_expr} + {self.unpacked.size})"
            try:
                start_simplified = str(simplify(start_expr))
                end_simplified = str(simplify(end_expr))
                # Don't generate >= 0 checks as they're always true and cause warnings
                if start_simplified == '0':
                    return f"auto_ss_state_idx < ({end_simplified})"
                else:
                    return f"auto_ss_state_idx >= ({start_simplified}) && auto_ss_state_idx < ({end_simplified})"
            except:
                if start_expr == '0':
                    return f"auto_ss_state_idx < ({end_expr})"
                else:
                    return f"auto_ss_state_idx >= ({start_expr}) && auto_ss_state_idx < ({end_expr})"
        else:
            size = int(self.unpacked.size)
            try:
                start_simplified = str(simplify(start_expr))
                end_simplified = str(simplify(f"{start_expr} + {size}"))
                # Don't generate >= 0 checks as they're always true and cause warnings
                if start_simplified == '0':
                    return f"auto_ss_state_idx < ({end_simplified})"
                else:
                    return f"auto_ss_state_idx >= ({start_simplified}) && auto_ss_state_idx < ({end_simplified})"
            except:
                if start_expr == '0':
                    return f"auto_ss_state_idx < ({start_expr} + {size})"
                else:
                    return f"auto_ss_state_idx >= ({start_expr}) && auto_ss_state_idx < ({start_expr} + {size})"

    def get_array_index_expr(self, base_offset=None) -> str:
        """Get expression for array indexing"""
        if not self.unpacked:
            raise ValueError(f"Register {self.name} is not an array")

        # Use the chunk_index (which might be symbolic) if available
        start_expr = str(self.chunk_index) if hasattr(self, 'chunk_index') else str(base_offset or 0)

        try:
            # Try to simplify the index expression
            index_expr = f"auto_ss_state_idx - ({start_expr})"
            simplified = str(simplify(index_expr))
            return simplified
        except:
            return f"auto_ss_state_idx - {start_expr}"

    def get_data_slice(self) -> str:
        """Get data slice expression for this register"""
        if not self.packed:
            # Single bit register - use bit_offset if it exists (when packed with others)
            if self.bit_offset is not None:
                return f"auto_ss_data_in[{self.bit_offset}]"
            else:
                return "auto_ss_data_in[0]"

        try:
            size = int(self.packed.size)
            if self.bit_offset is not None:
                end_bit = self.bit_offset + size - 1
                if size == 1:
                    return f"auto_ss_data_in[{self.bit_offset}]"
                else:
                    return f"auto_ss_data_in[{end_bit}:{self.bit_offset}]"
            else:
                if size == 1:
                    return "auto_ss_data_in[0]"
                else:
                    return f"auto_ss_data_in[{size-1}:0]"
        except (ValueError, TypeError):
            # Parameter-sized, use full width
            return f"auto_ss_data_in[{self.packed.size}-1:0]"

    def get_assignment_target(self) -> str:
        """Get the left-hand side target for assignments"""
        return self.name


    def __repr__(self) -> str:
        p = str(self.packed) if self.packed else ""
        u = str(self.unpacked) if self.unpacked else ""
        return f"reg {p} {self.name} {u}"


@dataclass
class ChunkPacker:
    """Manages packing of registers into chunks"""
    data_width: int = Config.DATA_WIDTH
    chunks: List[List[Register]] = field(default_factory=list)
    chunk_usage: List[int] = field(default_factory=list)

    def pack_registers(self, registers: List[Register]) -> int:
        """Pack registers into chunks and return total chunk count"""
        # Separate packable and non-packable registers
        packable = [r for r in registers if r.is_packable()]
        arrays = [r for r in registers if r.unpacked]
        large_regs = [r for r in registers if not r.is_packable() and not r.unpacked]

        chunk_idx = 0

        # Pack small registers together
        chunk_idx = self._pack_small_registers(packable, chunk_idx)

        # Allocate large registers to individual chunks
        chunk_idx = self._allocate_large_registers(large_regs, chunk_idx)

        # Allocate arrays (each element gets its own chunk)
        chunk_idx = self._allocate_arrays(arrays, chunk_idx)

        return chunk_idx

    def _pack_small_registers(self, registers: List[Register], start_chunk: int) -> int:
        """Pack small registers using greedy bin packing"""
        # Sort by size descending for better packing
        registers.sort(key=lambda r: r.get_packed_size(), reverse=True)

        chunk_idx = start_chunk
        current_chunk = []
        current_usage = 0

        for reg in registers:
            reg_size = reg.get_packed_size()

            if current_usage + reg_size <= self.data_width:
                # Fits in current chunk
                reg.allocate_chunk(chunk_idx, current_usage)
                current_chunk.append(reg)
                current_usage += reg_size
            else:
                # Start new chunk
                if current_chunk:
                    self.chunks.append(current_chunk)
                    self.chunk_usage.append(current_usage)
                    chunk_idx += 1

                current_chunk = [reg]
                current_usage = reg_size
                reg.allocate_chunk(chunk_idx, 0)

        # Don't forget the last chunk
        if current_chunk:
            self.chunks.append(current_chunk)
            self.chunk_usage.append(current_usage)
            chunk_idx += 1

        return chunk_idx

    def _allocate_large_registers(self, registers: List[Register], start_chunk: int) -> int:
        """Allocate large registers to individual chunks"""
        chunk_idx = start_chunk

        for reg in registers:
            reg.allocate_chunk(chunk_idx)
            chunk_idx += 1

        return chunk_idx

    def _allocate_arrays(self, arrays: List[Register], start_chunk: int) -> int:
        """Allocate arrays - each element gets its own chunk"""
        chunk_idx_expr = str(start_chunk)

        for array in arrays:
            # Set the starting chunk index for this array
            if chunk_idx_expr == str(start_chunk):
                array.allocate_chunk(start_chunk)
            else:
                # For subsequent arrays, store the symbolic expression
                array.chunk_index = chunk_idx_expr

            if array.is_param_sized():
                # Parameter-sized arrays use parameter expression for count
                array.chunk_count = None  # Will use parameter expression in bounds check
                # Update chunk_idx_expr to account for this array's size
                if chunk_idx_expr == str(start_chunk):
                    chunk_idx_expr = f"({start_chunk} + {array.unpacked.size})"
                else:
                    chunk_idx_expr = f"({chunk_idx_expr} + {array.unpacked.size})"
            else:
                try:
                    array_size = int(array.unpacked.size)
                    array.chunk_count = array_size
                    # Update chunk index for next array
                    if chunk_idx_expr == str(start_chunk):
                        chunk_idx_expr = str(start_chunk + array_size)
                    else:
                        chunk_idx_expr = f"({chunk_idx_expr} + {array_size})"
                except (ValueError, TypeError):
                    array.chunk_count = None
                    # Assume 1 for unknown size
                    if chunk_idx_expr == str(start_chunk):
                        chunk_idx_expr = str(start_chunk + 1)
                    else:
                        chunk_idx_expr = f"({chunk_idx_expr} + 1)"

        # Return a conservative estimate for the next chunk index
        # For simplicity, we'll track this separately if needed
        return start_chunk + len(arrays)  # Conservative estimate


class Assignment:
    """Represents an always block with register assignments"""
    def __init__(self, always: verible_verilog_syntax.Node, syms: List[str]):
        self.syms = sorted(set(syms))
        self.registers: List[Register] = []
        self.always = always
        self.reset_signal = None
        self.reset_polarity = False

        self._extract_reset_signal()

    def _extract_reset_signal(self):
        """Extract reset signal from sensitivity list"""
        for ev in self.always.iter_find_all({"tag": "kEventExpression"}):
            signal = ev.children[1].text
            if signal.lower() in Config.RESET_SIGNALS:
                self.reset_signal = signal
                self.reset_polarity = ev.children[0].text == "posedge"

    def modify_tree(self, use_local_signals: bool = False):
        """Modify the AST to inject state save/restore logic"""
        # Pack registers into chunks
        packer = ChunkPacker()
        chunk_count = packer.pack_registers(self.registers)

        prefix = Config.PREFIX
        if use_local_signals:
            output_signal = f"{prefix}_local_data_out"
            ack_signal = f"{prefix}_local_ack"
        else:
            output_signal = f"{prefix}_data_out"
            ack_signal = f"{prefix}_ack"

        wr_str = self._generate_chunked_write_logic(packer)
        rd_str = self._generate_chunked_read_logic(packer, output_signal, ack_signal)

        self._inject_write_logic(wr_str)
        self._inject_read_logic(rd_str)

    def _generate_chunked_write_logic_with_global_packer(self, global_packer: ChunkPacker) -> str:
        """Generate chunked write logic using global packer indices"""
        prefix = Config.PREFIX
        wr_str = f"if ({prefix}_wr && device_match) begin\n"

        # Find chunks that contain registers from this assignment
        assignment_chunks = {}
        for chunk_idx, chunk_regs in enumerate(global_packer.chunks):
            assignment_regs_in_chunk = [reg for reg in chunk_regs if reg in self.registers]
            if assignment_regs_in_chunk:
                assignment_chunks[chunk_idx] = assignment_regs_in_chunk

        if assignment_chunks:
            wr_str += f"case ({prefix}_state_idx)\n"

            for chunk_idx, regs_in_chunk in assignment_chunks.items():
                wr_str += f"{chunk_idx}: begin\n"
                for reg in regs_in_chunk:
                    wr_str += f"    {reg.get_assignment_target()} <= {reg.get_data_slice()};\n"
                wr_str += "end\n"

            wr_str += "default: begin\n"

        # Generate array handling for this assignment
        arrays = [r for r in self.registers if r.unpacked]
        if arrays:
            for array in arrays:
                bounds_check = array.get_bounds_check()
                array_idx = array.get_array_index_expr()
                data_slice = array.get_data_slice()

                wr_str += f"    if ({bounds_check}) begin\n"
                wr_str += f"        {array.name}[{array_idx}] <= {data_slice};\n"
                wr_str += f"    end\n"

        if assignment_chunks:
            wr_str += "end\n"  # Close default case
            wr_str += "endcase\n"

        wr_str += "end"
        return wr_str

    def _generate_chunked_write_logic(self, packer: ChunkPacker) -> str:
        """Generate chunked write logic for state saving"""
        prefix = Config.PREFIX
        wr_str = f"if ({prefix}_wr && device_match) begin\n"

        # Generate packed register chunks (use case statements)
        if packer.chunks:
            wr_str += f"case ({prefix}_state_idx)\n"

            for chunk_idx, chunk_regs in enumerate(packer.chunks):
                wr_str += f"{chunk_idx}: begin\n"
                for reg in chunk_regs:
                    wr_str += f"    {reg.get_assignment_target()} <= {reg.get_data_slice()};\n"
                wr_str += "end\n"

            wr_str += "default: begin\n"

        # Generate array handling (use if/else for parameters)
        arrays = [r for r in self.registers if r.unpacked]
        if arrays:
            for array in arrays:
                bounds_check = array.get_bounds_check()
                array_idx = array.get_array_index_expr()
                data_slice = array.get_data_slice()

                wr_str += f"    if ({bounds_check}) begin\n"
                wr_str += f"        {array.name}[{array_idx}] <= {data_slice};\n"
                wr_str += f"    end\n"

        if packer.chunks:
            wr_str += "end\n"  # Close default case
            wr_str += "endcase\n"

        wr_str += "end"
        return wr_str

    def _generate_chunked_read_logic(self, packer: ChunkPacker, output_signal: str = None, ack_signal: str = None) -> str:
        """Generate chunked read logic for state restoration"""
        prefix = Config.PREFIX
        if output_signal is None:
            output_signal = f"{prefix}_data_out"
        if ack_signal is None:
            ack_signal = f"{prefix}_ack"

        rd_str = f"always_comb begin\n"
        rd_str += f"    {output_signal} = 32'h0;\n"
        rd_str += f"    {ack_signal} = 1'b0;\n"
        rd_str += f"    if ({prefix}_rd && device_match) begin\n"

        # Generate packed register chunks (use case statements)
        if packer.chunks:
            rd_str += f"        case ({prefix}_state_idx)\n"

            for chunk_idx, chunk_regs in enumerate(packer.chunks):
                rd_str += f"        {chunk_idx}: begin\n"

                # Build concatenation expression
                if len(chunk_regs) == 1:
                    reg = chunk_regs[0]
                    if reg.packed:
                        # Use explicit vector slice for packed registers
                        rd_str += f"            {output_signal}[{reg.packed.size}-1:0] = {reg.name};\n"
                    else:
                        # Single bit register
                        rd_str += f"            {output_signal}[0] = {reg.name};\n"
                    rd_str += f"            {ack_signal} = 1'b1;\n"
                else:
                    # Multiple registers in chunk - build concatenation
                    concat_parts = []
                    total_width = 0
                    for reg in sorted(chunk_regs, key=lambda r: r.bit_offset or 0, reverse=True):
                        concat_parts.append(reg.name)
                        total_width += reg.get_packed_size()
                    rd_str += f"            {output_signal}[{total_width-1}:0] = {{{', '.join(concat_parts)}}};\n"
                    rd_str += f"            {ack_signal} = 1'b1;\n"

                rd_str += f"        end\n"

            rd_str += f"        default: begin\n"

        # Generate array handling (use if/else for parameters)
        arrays = [r for r in self.registers if r.unpacked]
        if arrays:
            for array in arrays:
                bounds_check = array.get_bounds_check()
                array_idx = array.get_array_index_expr()

                rd_str += f"            if ({bounds_check}) begin\n"
                if array.packed:
                    rd_str += f"                {output_signal}[{array.packed.size}-1:0] = {array.name}[{array_idx}];\n"
                else:
                    rd_str += f"                {output_signal}[0] = {array.name}[{array_idx}];\n"
                rd_str += f"                {ack_signal} = 1'b1;\n"
                rd_str += f"            end\n"

        if packer.chunks:
            rd_str += f"        end\n"  # Close default case
            rd_str += f"        endcase\n"

        rd_str += f"    end\n"
        rd_str += f"end\n"

        return rd_str

    def _inject_write_logic(self, wr_str: str):
        """Inject write logic into the AST"""
        if self.reset_signal:
            # FIXME - we are assuming that the first if clause is going to be for reset
            cond = find_path(self.always, ["kIfClause"])
            if not self.reset_signal in cond.text:
                raise Exception(f"Reset without if {cond.text}")
            add_text_after(cond, "else " + wr_str)
        else:
            ctrl = find_path(self.always, ["kProceduralTimingControlStatement", "kEventControl"])
            add_text_after(ctrl, "begin")
            add_text_after(ctrl.parent.children[-1], wr_str + "\nend\n")

    def _inject_read_logic(self, rd_str: str):
        """Inject read logic after the always block"""
        add_text_after(self.always, rd_str)

    def __repr__(self) -> str:
        return f"Assignment({self.syms})"


@dataclass
class ModuleInstance:
    """Represents an instance of a module"""
    name: str
    module_name: str
    node: verible_verilog_syntax.Node
    module: Optional['Module'] = None
    params: List[str] = field(default_factory=list)
    named_params: Dict[str, str] = field(default_factory=dict)
    allocated: Optional[Dimension] = None
    reg_size: Optional[str] = None
    sub_size: Optional[str] = None
    base_idx: Optional[int] = None

    def add_param(self, name: Optional[str], value: str):
        """Add a parameter to the instance"""
        if name is None:
            if len(self.named_params):
                raise ValueError("Adding positional parameter when named parameters already exist")
            self.params.append(value)
        else:
            if len(self.params):
                raise ValueError("Adding named parameter when positional parameters already exist")
            self.named_params[name] = value

    def assign_base_idx(self, cur: int) -> int:
        """Assign base index for hierarchical state management"""
        self.base_idx = cur
        return cur + 1

    def allocate(self, offset: str) -> str:
        """Allocate state space for this instance"""
        if not self.module:
            return offset

        module_dim = self.module.allocate()
        if module_dim is None:
            return offset

        params = self.module.eval_params(self.params, self.named_params)
        end = str(module_dim.end)
        reg_size = str(self.module.reg_dim.size)
        sub_size = str(self.module.sub_dim.size)

        # Substitute parameters
        for k, v in params.items():
            end = end.replace(k, f"({v})")
            reg_size = reg_size.replace(k, f"({v})")
            sub_size = sub_size.replace(k, f"({v})")

        self.allocated = Dimension(f"({end})+({offset})", offset)
        self.sub_size = simplify(sub_size)
        self.reg_size = simplify(reg_size)

        return f"({offset})+({self.size()})"

    def size(self) -> str:
        """Get allocated size"""
        if self.allocated:
            return str(self.allocated.size)
        return "0"

    def modify_tree(self, use_instance_signals: bool = False):
        """Modify AST to add state ports"""
        if not self.allocated:
            return

        prefix = Config.PREFIX
        if use_instance_signals:
            data_out_signal = f"{prefix}_{self.name}_data_out"
            ack_signal = f"{prefix}_{self.name}_ack"
        else:
            data_out_signal = f"{prefix}_data_out"
            ack_signal = f"{prefix}_ack"

        port_list = find_path(self.node, ["kGateInstance", "kPortActualList"])
        add_text_after(port_list, f""",
                .{prefix}_rd({prefix}_rd),
                .{prefix}_wr({prefix}_wr),
                .{prefix}_data_in({prefix}_data_in),
                .{prefix}_device_idx({prefix}_device_idx),
                .{prefix}_state_idx({prefix}_state_idx),
                .{prefix}_base_device_idx({prefix}_base_device_idx + {self.base_idx if self.base_idx is not None else 0}),
                .{prefix}_data_out({data_out_signal}),
                .{prefix}_ack({ack_signal})""")

    def __repr__(self):
        return f"ModuleInstance({self.module_name} {self.name})"


class Module:
    """Represents a Verilog module"""
    def __init__(self, node: verible_verilog_syntax.Node):
        self.node = node
        name = find_path(node, ["kModuleHeader", "SymbolIdentifier"])
        self.name = name.text
        self.instances: List[ModuleInstance] = []
        self.registers: List[Register] = []
        self.assignments: List[Assignment] = []
        self.parameters: List[Tuple[str, str]] = []
        self.state_dim: Optional[Dimension] = None
        self.reg_dim: Optional[Dimension] = None
        self.sub_dim: Optional[Dimension] = None
        self._ancestor_count: Optional[int] = None

        self.allocated = False
        self.predefined = False

        self._extract_all()

    def _extract_all(self):
        """Extract all module components"""
        self.extract_module_instances()
        self.extract_registers()
        self.extract_assignments()
        self.extract_parameters()

    def __repr__(self) -> str:
        return f"Module({self.name})"

    def ancestor_count(self) -> int:
        """Get the base device index for this module in the hierarchy"""
        if self._ancestor_count is not None:
            return self._ancestor_count

        # For chunked mode, return the device count from all child instances
        count = 0
        for inst in self.instances:
            if inst.module and inst.module.state_dim:
                count += 1 + inst.module.ancestor_count()

        self._ancestor_count = count
        return count

    def eval_params(self, positional: List[str], named: Dict[str,str]) -> Dict[str,str]:
        """Evaluate parameters for this module"""
        r = {}
        for idx, (name, default) in enumerate(self.parameters):
            if idx < len(positional):
                r[name] = positional[idx]
            elif name in named:
                r[name] = named[name]
            else:
                r[name] = default
        return r

    def extract_module_instances(self):
        """Extract module instantiations"""
        for decl in self.node.iter_find_all({"tag": "kDataDeclaration"}):
            data_type = find_path(decl, ["kInstantiationType", "kDataType"])
            ports = find_path(decl, ["kPortActualList"])
            if data_type is None or ports is None:
                continue

            instance_def = find_path(decl, ["kGateInstance"])
            ref = find_path(data_type, ["kUnqualifiedId", "SymbolIdentifier"])
            instance = ModuleInstance(
                name=instance_def.children[0].text,
                module_name=ref.text,
                node=decl
            )

            # Extract parameters
            for param in data_type.iter_find_all({"tag": "kParamByName"}):
                param_name = param.children[1].text
                param_value = param.children[2].children[1].text
                instance.add_param(param_name, param_value)

            params = find_path(data_type, ["kActualParameterPositionalList"])
            if params:
                for param in params.children[::2]:
                    instance.add_param(None, param.text)

            self.instances.append(instance)

    def extract_registers(self):
        """Extract register declarations"""
        dup_track = {}

        # Extract regular register declarations
        for decl in self.node.iter_find_all({"tag": "kDataDeclaration"}):
            dims = find_path(decl, ["kPackedDimensions", "kDimensionRange"])
            packed = None
            if dims:
                packed = Dimension(dims.children[1].text, dims.children[3].text)

            instances = decl.find({"tag": "kGateInstanceRegisterVariableList"})
            if instances:
                for variable in instances.iter_find_all({"tag": "kRegisterVariable"}):
                    unpacked = None
                    dims = find_path(variable, ["kUnpackedDimensions", "kDimensionRange"])
                    if dims:
                        unpacked = Dimension(dims.children[1].text, dims.children[3].text)

                    sym = variable.find({"tag": "SymbolIdentifier"})
                    reg = Register(sym.text, packed=packed, unpacked=unpacked)
                    self._add_register(reg, dup_track)

        # Extract port declarations
        for decl in self.node.iter_find_all({"tag": ["kPortDeclaration", "kModulePortDeclaration"]}):
            packed = None
            dims = find_path(decl, ["kPackedDimensions", "kDimensionRange"])
            if dims:
                packed = Dimension(dims.children[1].text, dims.children[3].text)

            sym = find_path(decl, ["kUnqualifiedId"])
            if sym is None:
                sym = find_path(decl, ["kIdentifierUnpackedDimensions", "SymbolIdentifier"])

            if sym:
                reg = Register(sym.text, packed=packed)
                self._add_register(reg, dup_track)

    def _add_register(self, reg: Register, dup_track: Dict[str, Register]):
        """Add register with duplicate checking"""
        if reg.name in dup_track:
            if dup_track[reg.name] != reg:
                raise ValueError(f"Conflicting register declarations: {reg} vs {dup_track[reg.name]}")
        else:
            dup_track[reg.name] = reg
            self.registers.append(reg)

    def extract_assignments(self):
        """Extract always blocks with assignments"""
        for always in self.node.iter_find_all({"tag": "kAlwaysStatement"}):
            syms = []
            for assign in always.iter_find_all({"tag": "kNonblockingAssignmentStatement"}):
                target = assign.find({"tag": "kLPValue"})
                if not target:
                    logger.warning("Assignment without target")
                    continue

                sym = target.find({"tag": "SymbolIdentifier"})
                if not sym:
                    logger.warning("Assignment without symbol")
                    continue

                syms.append(sym.text)

            if len(syms):
                self.assignments.append(Assignment(always, syms))

    def extract_parameters(self):
        """Extract module parameters"""
        for decl in self.node.iter_find_all({"tag": "kParamDeclaration"}):
            name = find_path(decl, ["kParamType", "SymbolIdentifier"])
            value = find_path(decl, ["kTrailingAssign", "kExpression"])
            self.parameters.append((name.text, value.text))

        for decl in self.node.iter_find_all({"tag": "kParameterAssign"}):
            self.parameters.append((decl.children[0].text, decl.children[2].text))

    def allocate(self) -> Optional[Dimension]:
        """Allocate state space for this module"""
        if self.allocated:
            return self.state_dim

        prefix = Config.PREFIX

        # Check for predefined state interface
        for reg in self.registers:
            if reg.name == f"{prefix}_out":
                self.predefined = True
                self.assignments = []
                self.registers = []
                reg.allocate("0")
                self.state_dim = reg.allocated
                self.allocated = True
                self.reg_dim = reg.allocated
                self.sub_dim = Dimension("0", "0")
                return self.state_dim

        # Build assignment map
        assigned = {}
        for a in self.assignments:
            for sym in a.syms:
                assigned[sym] = 1

        # Filter registers to only assigned ones
        allocated = {}
        for reg in self.registers:
            if reg.name in assigned:
                allocated[reg.name] = reg
        self.registers = list(allocated.values())

        # Link registers to assignments
        for a in self.assignments:
            for sym in a.syms:
                if sym in allocated:
                    a.registers.append(allocated[sym])

        # Assign device indices and allocate instances
        device_idx = 1  # Start at 1 (relative to current module's base)
        for inst in self.instances:
            inst.allocate("0")  # Still need to call for sub-allocation
            if inst.module and inst.module.state_dim:
                inst.assign_base_idx(device_idx)
                # Next instance starts after this one plus all its descendants
                device_idx += 1 + inst.module.ancestor_count()

        self.allocated = True

        # Determine if this module has state (either registers or sub-instances with state)
        has_registers = len(self.registers) > 0
        has_stateful_instances = any(inst.module and inst.module.state_dim for inst in self.instances)

        if has_registers or has_stateful_instances:
            # Module has state - create minimal state dimension
            self.state_dim = Dimension("0", "0")  # At least one chunk
            self.reg_dim = Dimension("0", "0")
            self.sub_dim = Dimension("0", "0")
            return self.state_dim
        else:
            return None

    def print_allocation(self):
        """Print allocation information for debugging"""
        if not self.state_dim:
            return

        logger.info(f"{self.name} ancestors={self.ancestor_count()}")
        for i in self.instances:
            if i.module:
                logger.info(f"    {i.module_name} {i.name} ancestors={i.module.ancestor_count()}")

    def modify_tree(self):
        """Modify AST to add state save/restore logic"""
        if not self.state_dim:
            return

        if self.predefined:
            return

        prefix = Config.PREFIX
        verilog1995 = find_path(self.node, ["kModuleItemList", "kModulePortDeclaration"]) is not None

        port_decl = find_path(self.node, ["kModuleHeader", "kPortDeclarationList"])
        header = find_path(self.node, ["kModuleHeader"])

        # Generate chunked interface ports
        data_width = Config.DATA_WIDTH - 1
        device_width = Config.DEVICE_WIDTH - 1
        state_width = Config.STATE_WIDTH - 1
        base_device_idx = self.ancestor_count()

        if verilog1995:
            port_list = f",\n{prefix}_rd, {prefix}_wr, {prefix}_data_in, {prefix}_device_idx, {prefix}_state_idx, {prefix}_base_device_idx, {prefix}_data_out, {prefix}_ack"
            add_text_after(port_decl, port_list)

            s =  f"input {prefix}_rd;\n"
            s += f"input {prefix}_wr;\n"
            s += f"input [{data_width}:0] {prefix}_data_in;\n"
            s += f"input [{device_width}:0] {prefix}_device_idx;\n"
            s += f"input [{state_width}:0] {prefix}_state_idx;\n"
            s += f"input [{device_width}:0] {prefix}_base_device_idx;\n"
            s += f"output logic [{data_width}:0] {prefix}_data_out;\n"
            s += f"output logic {prefix}_ack;\n"
            add_text_after(header, s)
        else:
            s = f",\ninput {prefix}_rd, input {prefix}_wr, "
            s += f"input [{data_width}:0] {prefix}_data_in, "
            s += f"input [{device_width}:0] {prefix}_device_idx, "
            s += f"input [{state_width}:0] {prefix}_state_idx, "
            s += f"input [{device_width}:0] {prefix}_base_device_idx, "
            s += f"output logic [{data_width}:0] {prefix}_data_out, "
            s += f"output logic {prefix}_ack"
            add_text_after(port_decl, s)

        # Add internal declarations
        add_text_after(header, f"genvar {prefix}_idx;")  # used by assignments

        # Add device matching logic - always exact match
        add_text_after(header, f"wire device_match = ({prefix}_device_idx == {prefix}_base_device_idx);")

        # Add data output and acknowledgment logic
        has_registers = len(self.registers) > 0
        has_instances = len([i for i in self.instances if i.module and i.module.state_dim]) > 0

        if has_instances:
            # Module has sub-instances - need to declare intermediate signals and mux
            instance_signals = []
            for i, inst in enumerate(self.instances):
                if inst.module and inst.module.state_dim:
                    instance_signals.append(f"{prefix}_{inst.name}")

            # Deduplicate signal names (can happen with conditional compilation)
            instance_signals = list(dict.fromkeys(instance_signals))

            # Declare intermediate signals
            for sig in instance_signals:
                add_text_after(header, f"wire [{data_width}:0] {sig}_data_out;")
                add_text_after(header, f"wire {sig}_ack;")

            # Generate muxing logic for data_out
            if has_registers:
                # Module has both registers and sub-instances
                add_text_after(header, f"logic [{data_width}:0] {prefix}_local_data_out;")
                add_text_after(header, f"logic {prefix}_local_ack;")
                mux_signals = [f"{prefix}_local_data_out"] + [f"{sig}_data_out" for sig in instance_signals]
                ack_signals = [f"{prefix}_local_ack"] + [f"{sig}_ack" for sig in instance_signals]
            else:
                # Module has only sub-instances
                mux_signals = [f"{sig}_data_out" for sig in instance_signals]
                ack_signals = [f"{sig}_ack" for sig in instance_signals]

            # OR together all data_out signals
            add_text_after(header, f"assign {prefix}_data_out = {' | '.join(mux_signals)};")
            # OR together all ack signals
            add_text_after(header, f"assign {prefix}_ack = {' | '.join(ack_signals)};")

        else:
            # Module has only registers - ACK generated in always_comb
            if not has_registers:
                # No state at all - provide default outputs
                add_text_after(header, f"assign {prefix}_ack = 1'b0;")
                add_text_after(header, f"assign {prefix}_data_out = {Config.DATA_WIDTH}'h0;")

        for i in self.instances:
            i.modify_tree(use_instance_signals=has_instances)

        # Generate combined read/write logic for all assignments
        self._generate_combined_savestate_logic(has_instances)

    def _generate_combined_savestate_logic(self, has_instances: bool):
        """Generate a single combined always_comb block for all assignments"""
        if not self.assignments:
            return

        prefix = Config.PREFIX
        if has_instances:
            output_signal = f"{prefix}_local_data_out"
            ack_signal = f"{prefix}_local_ack"
        else:
            output_signal = f"{prefix}_data_out"
            ack_signal = f"{prefix}_ack"

        # Pack all registers from all assignments together with global indexing
        all_registers = []
        for assignment in self.assignments:
            all_registers.extend(assignment.registers)

        if not all_registers:
            return

        # Pack registers into chunks with global indexing
        global_packer = ChunkPacker()
        chunk_count = global_packer.pack_registers(all_registers)

        # Generate write logic for each assignment using global indices
        for assignment in self.assignments:
            if assignment.registers:
                wr_str = assignment._generate_chunked_write_logic_with_global_packer(global_packer)
                assignment._inject_write_logic(wr_str)

        # Generate single combined read logic
        rd_str = self._generate_combined_read_logic(global_packer, output_signal, ack_signal)

        # Inject the combined read logic after the last assignment
        if self.assignments:
            add_text_after(self.assignments[-1].always, rd_str)

    def _generate_combined_read_logic(self, packer: ChunkPacker, output_signal: str, ack_signal: str) -> str:
        """Generate combined read logic for all assignments"""
        prefix = Config.PREFIX
        rd_str = f"always_comb begin\n"
        rd_str += f"    {output_signal} = 32'h0;\n"
        rd_str += f"    {ack_signal} = 1'b0;\n"
        rd_str += f"    if ({prefix}_rd && device_match) begin\n"

        # Generate packed register chunks (use case statements)
        if packer.chunks:
            rd_str += f"        case ({prefix}_state_idx)\n"

            for chunk_idx, chunk_regs in enumerate(packer.chunks):
                rd_str += f"        {chunk_idx}: begin\n"

                # Build concatenation expression
                if len(chunk_regs) == 1:
                    reg = chunk_regs[0]
                    if reg.packed:
                        # Use explicit vector slice for packed registers
                        rd_str += f"            {output_signal}[{reg.packed.size}-1:0] = {reg.name};\n"
                    else:
                        # Single bit register
                        rd_str += f"            {output_signal}[0] = {reg.name};\n"
                    rd_str += f"            {ack_signal} = 1'b1;\n"
                else:
                    # Multiple registers in chunk - build concatenation
                    concat_parts = []
                    total_width = 0
                    for reg in sorted(chunk_regs, key=lambda r: r.bit_offset or 0, reverse=True):
                        concat_parts.append(reg.name)
                        total_width += reg.get_packed_size()
                    rd_str += f"            {output_signal}[{total_width-1}:0] = {{{', '.join(concat_parts)}}};\n"
                    rd_str += f"            {ack_signal} = 1'b1;\n"

                rd_str += f"        end\n"

            rd_str += f"        default: begin\n"

        # Generate array handling (use if/else for parameters)
        all_registers = []
        for assignment in self.assignments:
            all_registers.extend(assignment.registers)
        arrays = [r for r in all_registers if r.unpacked]

        if arrays:
            for array in arrays:
                bounds_check = array.get_bounds_check()
                array_idx = array.get_array_index_expr()

                rd_str += f"            if ({bounds_check}) begin\n"
                if array.packed:
                    rd_str += f"                {output_signal}[{array.packed.size}-1:0] = {array.name}[{array_idx}];\n"
                else:
                    rd_str += f"                {output_signal}[0] = {array.name}[{array_idx}];\n"
                rd_str += f"                {ack_signal} = 1'b1;\n"
                rd_str += f"            end\n"

        if packer.chunks:
            rd_str += f"        end\n"  # Close default case
            rd_str += f"        endcase\n"

        rd_str += f"    end\n"
        rd_str += f"end\n"

        return rd_str

    def output_module(self, fp, enable_format: bool = True):
        """Output modified module to file"""
        s = "///////////////////////////////////////////\n"
        s += f"// MODULE {self.name}\n"
        begin = None
        for tok in verible_verilog_syntax.PreOrderTreeIterator(self.node):
            if isinstance(tok, InsertNode):
                s += f"\n{tok.text}\n"
            elif isinstance(tok, verible_verilog_syntax.TokenNode):
                if begin is None:
                    begin = tok.start
                end = tok.end
                s += tok.syntax_data.source_code[begin:end].decode("utf-8")
                begin = end

        s += "\n\n\n"

        if enable_format:
            s = format_output(s)

        fp.write(s)

    def post_order(self) -> List['Module']:
        """Get modules in post-order traversal"""
        r = []
        for inst in self.instances:
            if inst.module:
                r.extend(inst.module.post_order())
        r.append(self)
        return r


class ModuleResolver:
    """Resolves module dependencies and builds hierarchy"""

    def __init__(self, modules: List[Module]):
        self.modules = {m.name: m for m in modules}

    def resolve(self, root_name: str) -> Module:
        """Resolve module hierarchy starting from root"""
        if root_name not in self.modules:
            raise ModuleNotFoundError(f"Root module '{root_name}' not found")

        self._resolve_instances()
        return self.modules[root_name]

    def _resolve_instances(self):
        """Resolve all module instances"""
        for module in self.modules.values():
            for inst in module.instances:
                if inst.module_name not in self.modules:
                    raise ModuleNotFoundError(
                        f"Module '{inst.module_name}' referenced by "
                        f"'{module.name}.{inst.name}' not found"
                    )
                inst.module = self.modules[inst.module_name]


def process_file_data(data: verible_verilog_syntax.SyntaxData) -> List[Module]:
    """Process syntax data and extract modules"""
    if not data.tree:
        return []

    modules = []
    for module_node in data.tree.iter_find_all({"tag": "kModuleDeclaration"}):
        modules.append(Module(module_node))

    return modules


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description="Generate automatic savestate support for Verilog modules"
    )
    parser.add_argument(
        'module',
        help='Root module name to process'
    )
    parser.add_argument(
        'output',
        help='Output file path (use - for stdout)'
    )
    parser.add_argument(
        'files',
        nargs='+',
        type=Path,
        help='Verilog source files to process'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )
    parser.add_argument(
        '--no-format',
        action='store_true',
        help='Disable output formatting'
    )
    parser.add_argument(
        '--generate-csv',
        type=str,
        help='Generate CSV mapping file with device/state indices'
    )
    return parser.parse_args()


def generate_csv_mapping(root_module: 'Module', csv_path: str):
    """Generate CSV mapping of device/state indices to modules and data"""
    import csv
    
    rows = []
    
    def collect_mappings(module: 'Module', base_device_idx: int = 0):
        """Recursively collect mappings from module hierarchy"""
        
        # Add mappings for this module's registers
        if module.registers:
            # For modules with assignments, we need to use the same combined packing
            # approach as the code generation to ensure consistency
            if module.assignments:
                # Collect all registers from all assignments (same as _generate_combined_savestate_logic)
                all_registers = []
                for assignment in module.assignments:
                    all_registers.extend(assignment.registers)
                
                # Pack registers with global indexing
                packer = ChunkPacker()
                chunk_count = packer.pack_registers(all_registers)
            else:
                # Module has registers but no assignments - use simple packing
                packer = ChunkPacker()
                chunk_count = packer.pack_registers(module.registers)
            
            # Add packed register chunks
            for chunk_idx, chunk_regs in enumerate(packer.chunks):
                # Sort registers by bit_offset in reverse order to match code generation
                sorted_regs = sorted(chunk_regs, key=lambda r: r.bit_offset or 0, reverse=True)
                reg_names = [reg.name for reg in sorted_regs]
                reg_types = []
                for reg in sorted_regs:
                    if reg.packed:
                        reg_types.append(f"reg[{reg.packed.size}-1:0]")
                    else:
                        reg_types.append("reg")
                
                rows.append([
                    base_device_idx,
                    chunk_idx,
                    module.name,
                    "packed_registers",
                    f"{', '.join(reg_names)}",
                    f"{', '.join(reg_types)}",
                    f"Packed registers: {', '.join(reg_names)}"
                ])
            
            # Add array mappings
            arrays = [r for r in module.registers if r.unpacked]
            for array in arrays:
                if hasattr(array, 'chunk_index'):
                    start_idx = array.chunk_index
                    if array.is_param_sized():
                        size_desc = f"{array.unpacked.size} elements"
                        end_desc = f"start+{array.unpacked.size}-1"
                    else:
                        try:
                            size = int(array.unpacked.size)
                            size_desc = f"{size} elements"
                            end_desc = f"{start_idx + size - 1}"
                        except:
                            size_desc = "unknown size"
                            end_desc = "unknown"
                    
                    array_type = f"reg"
                    if array.packed:
                        array_type += f"[{array.packed.size}-1:0]"
                    array_type += f"[{array.unpacked.size}-1:0]"
                    
                    rows.append([
                        base_device_idx,
                        f"{start_idx}:{end_desc}",
                        module.name,
                        "array",
                        array.name,
                        array_type,
                        f"Array with {size_desc}"
                    ])
        
        # Add mappings for sub-instances
        for inst in module.instances:
            if inst.module and inst.module.state_dim and inst.base_idx is not None:
                instance_device_idx = base_device_idx + inst.base_idx
                
                # Add entry for the instance itself
                rows.append([
                    instance_device_idx,
                    "0+",
                    f"{module.name}.{inst.name}",
                    "module_instance",
                    inst.module_name,
                    "module",
                    f"Instance of {inst.module_name}"
                ])
                
                # Recursively collect from sub-module
                collect_mappings(inst.module, instance_device_idx)
    
    # Start collection from root
    collect_mappings(root_module)
    
    # Sort by device_idx, then by state_idx
    def sort_key(row):
        device_idx = row[0]
        state_idx_str = str(row[1])
        
        # Handle range notation like "1:8"
        if ':' in state_idx_str:
            start_part = state_idx_str.split(':')[0]
            try:
                return (device_idx, int(start_part))
            except:
                return (device_idx, 1000000)  # Put symbolic ranges at the end
        elif '+' in state_idx_str:
            return (device_idx, 1000000)  # Put range indicators at the end
        else:
            try:
                return (device_idx, int(state_idx_str))
            except:
                return (device_idx, 1000000)
    
    rows.sort(key=sort_key)
    
    # Write CSV file
    with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            'Device_Index',
            'State_Index',
            'Module_Path',
            'Data_Type',
            'Register_Name',
            'Verilog_Type',
            'Description'
        ])
        writer.writerows(rows)
    
    logger.info(f"Generated CSV mapping with {len(rows)} entries: {csv_path}")


def main():
    """Main entry point"""
    args = parse_args()

    # Setup logging
    setup_logging(args.verbose)

    # Validate inputs
    try:
        validate_inputs(args.files)
    except (FileNotFoundError, ValueError) as e:
        logger.error(str(e))
        return 1

    # Setup output
    out_fp = None
    if args.output == '-':
        out_fp = sys.stdout
    else:
        out_fp = open(args.output, "wt")

    try:
        # Parse Verilog files
        parser = verible_verilog_syntax.VeribleVerilogSyntax(executable="verible-verilog-syntax")
        preprocessed = preprocess_inputs(args.files)
        data = parser.parse_string(preprocessed)
        modules = process_file_data(data)

        # Resolve module hierarchy
        resolver = ModuleResolver(modules)
        root_module = resolver.resolve(args.module)

        # Allocate state space
        root_module.allocate()

        # Process modules in post-order
        output_modules = root_module.post_order()
        visited = set()

        for module in output_modules:
            if module in visited:
                continue
            module.modify_tree()
            module.print_allocation()
            module.output_module(out_fp, enable_format=not args.no_format)
            visited.add(module)

        # Generate CSV mapping if requested
        if args.generate_csv:
            generate_csv_mapping(root_module, args.generate_csv)

    except StateModuleError as e:
        logger.error(str(e))
        return 1
    except Exception as e:
        logger.exception("Unexpected error")
        return 1
    finally:
        if out_fp != sys.stdout:
            out_fp.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

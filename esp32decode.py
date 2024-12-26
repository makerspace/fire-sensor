import argparse
import subprocess
import sys
import os
import re
from pathlib import Path

def parse_arguments():
    parser = argparse.ArgumentParser(description='Decode stack trace using addr2line.')
    parser.add_argument('elf_file', type=str, help='Path to the ELF file.')
    parser.add_argument('-a', '--addr2line', type=str, default='addr2line',
                        help='Path to the addr2line tool (default is addr2line in system path)')
    return parser.parse_args()

def common_path_prefix(paths):
    """Find a common prefix to make the paths more human-readable"""
    paths_for_common = [p for p in paths if os.path.isabs(p)]
    try:
        commonpath = os.path.commonpath(paths_for_common)
    except:
        commonpath = ''
    return max([commonpath, os.getcwd()], key=len) + '/'

def decode_stack_trace(elf_file, addr2line_path, stack_trace):
    # Extracting the backtrace addresses from the stack trace string
    matches = re.findall(r' (0x[0-9a-fA-F]+):(0x[0-9a-fA-F]+)', stack_trace)
    if not matches:
        print("No addresses found in the stack trace.")
        return

    decoded_outputs = []
    for ip, sp in matches:
        cmd = [addr2line_path, '-e', elf_file, '--functions', '--demangle', '--pretty-print', ip]
        try:
            output = subprocess.run(cmd, capture_output=True, text=True)
            decoded_outputs.append(output.stdout.strip())
        except Exception as e:
            print(f"Error decoding address {ip}: {str(e)}")

    if not decoded_outputs:
        print("No output received from addr2line.")
        return

    # Remove common path prefix from file paths
    paths = [line.split(' ')[-1] for line in decoded_outputs if ' at ' in line]
    for output in decoded_outputs:
        print(output.replace(common_path_prefix(paths), ''))

if __name__ == "__main__":
    args = parse_arguments()
    stack_trace = sys.stdin.read()
    decode_stack_trace(args.elf_file, args.addr2line, stack_trace)
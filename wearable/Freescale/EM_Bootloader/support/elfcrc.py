#!/usr/bin/env python2

from __future__ import print_function
from collections import namedtuple
import argparse
import struct
import sys

#############################################################################
# argument parsing
#############################################################################

def auto_int (str):
    return int (str, 0)

def get_parser ():
    parser = argparse.ArgumentParser (description = 'insert a CRC into an Elf executable')

    parser.add_argument ('infile',
                         metavar = 'infile.elf',
                         type = argparse.FileType ('rb'),
                         help = 'original Elf file')

    # TuringSense bootloader:
    # The -p option is necessary because the KDS linker starts
    # phys addr on 32K boundary, but TuringSense flash image 1
    # starts at 0x0a000.
    #    default is "-p 0x0a000" for flash image 1,
    parser.add_argument ('-p', '--physaddr',
                         type = auto_int,
                         default = 0x0a000,
                         help = 'starting physical address')

    output_group = parser.add_mutually_exclusive_group(required = True)

    output_group.add_argument ('-e', '--elf',
                               type = argparse.FileType ('wb'),
                               metavar = 'outfile.elf',
                               help = 'output a Elf file with inserted CRC')

    output_group.add_argument ('-b', '--bin',
                               type = argparse.FileType ('wb'),
                               metavar = 'outfile.bin',
                               help = 'output a binary file with inserted CRC')

    FREESCALE_VID = 0x15a2
    FREESCALE_PID = 0x0073
    parser.add_argument('--vid', type=auto_int, help='USB vendor ID', default=FREESCALE_VID)
    parser.add_argument('--pid', type=auto_int, help='USB product ID', default=FREESCALE_PID)

    parser.add_argument ('-v', '--verbose', action = 'store_true')

    return parser

#############################################################################
# ELF file stuff
#############################################################################

elf_ident_t = namedtuple ('elf_ident', 'ei_magic ei_class ei_data ei_version ei_pad')

elf_header_t = namedtuple ('elf_header', 'e_type e_machine e_version e_entry e_phoff e_shoff e_flags e_ehsize e_phentsize e_phnum e_shentsize e_shnum e_shstrndx')

prog_header_t = namedtuple ('prog_header', 'p_type p_offset p_vaddr p_paddr p_filesz p_memsz p_flags p_align')

def get_elf_prog_headers (elf_data):
    elf_ident = elf_ident_t._make (struct.unpack_from ('4sBBB9s', elf_data, 0))

    if elf_ident.ei_magic != b'\x7fELF':
        print ('Not an ELF file', file = sys.stderr)
        print (elf_ident.ei_magic, file = sys.stderr)
        return None

    if elf_ident.ei_class != 1:  # ELFCLASS32
        print ('Not a 32-bit ELF file', file = sys.stderr)
        return None

    if elf_ident.ei_data == 1:  # ELFDATA2LSB
        endian = '<'
    elif elf_ident.ei_data == 2:  # ELFDATA2MSB
        endian = '>'
        print ('Big-endian ELF unsupported', file = sys.stderr)
        return None
    else:
        print ('Unrecognized ELF endianness', file = sys.stderr)
        return None

    if elf_ident.ei_version != 1:
        print ('Unrecognized ELF version', file = sys.stderr)
        return None

    if elf_ident.ei_pad != (9 * b'\0'):
        print ('Garbage in ELF ident padding', file = sys.stderr)

    elf_header = elf_header_t._make (struct.unpack_from (endian + 'HHIIIIIHHHHHH',
                                                         elf_data,
                                                         16));

    if elf_header.e_type != 2:  # ET_EXEC
        print ('Not an executable ELF file', file = sys.stderr)
        return None

    if elf_header.e_machine != 0x28:
        print ('Not an ARM ELF file', file = sys.stderr)
        return None

    if elf_header.e_ehsize != 52:
        print ('ELF header has wrong size (%d)' % elf_header.e_eh_size, file = sys.stderr)
        if elf_header.e_ehsize < 52:
            return None

    if elf_header.e_phentsize != 32:
        print ('ELF program header size is wrong (%d)' % elf_header.e_phentsize, file = sys.stderr)
        if elf_header.e_phentsize < 32:
            return None

    prog_headers = []
    for ph_index in range (elf_header.e_phnum):
        ph_offset = elf_header.e_phoff + ph_index * elf_header.e_phentsize
        prog_header = prog_header_t._make (struct.unpack_from (endian + 'IIIIIIII',
                                                               elf_data,
                                                               ph_offset));
        if prog_header.p_type == 1:  # PT_LOAD
            if prog_header.p_filesz != 0:
                prog_headers += [prog_header]
        elif prog_header.p_type == 0x70000000:  # PT_ARM_ARCHEXT
            pass
        elif prog_header.p_type == 0x70000001:  # PT_ARM_UNWIND
            pass
        else:
            print ('skipping unrecognized program header type %x', prog_header.p_type, file = sys.stderr)

    # sort program headers in physical address order
    prog_headers.sort (key=lambda foo: foo.p_paddr)

    return prog_headers


def find_prog_header (prog_headers, addr):
    for prog_header in prog_headers:
        if (addr >= prog_header.p_paddr and
            addr < (prog_header.p_paddr + prog_header.p_filesz)):
            return prog_header
    return None


def get_elf_flash_image_bounds (prog_headers):
    start_pa = prog_headers [0].p_paddr
    end_pa = start_pa
    for prog_header in prog_headers:
        if prog_header.p_paddr != end_pa:
            print ("flash image skipped from addr %x to %x" % (end_pa, prog_header.p_paddr), file = sys.stderr)
        end_pa = prog_header.p_paddr + prog_header.p_filesz
    return (start_pa, end_pa)


def get_elf_flash_image_length (prog_headers):
    length = 0
    for prog_header in prog_headers:
        length += prog_header.p_filesz
    return length


def elf_file_offset (prog_headers, addr):
    prog_header = find_prog_header (prog_headers, addr)
    if not prog_header:
        return None
    return addr - prog_header.p_paddr + prog_header.p_offset


def insert16 (prog_headers, elf_data, addr, data):
    offset = elf_file_offset (prog_headers, addr)
    if type(elf_data) == str:
        # Python 2.7
        s = (chr (data & 0xff) +
             chr ((data >> 8) & 0xff))
    else:
        # Python 3 - type is 'bytes', not 'str'
        s = bytes ([data & 0xff,
                    (data >> 8) & 0xff])
    return elf_data [0:offset] + s + elf_data [offset+2:]

def insert32 (prog_headers, elf_data, addr, data):
    offset = elf_file_offset (prog_headers, addr)
    if type(elf_data) == str:
        # Python 2.7
        s = (chr (data & 0xff) +
             chr ((data >> 8) & 0xff) +
             chr ((data >> 16) & 0xff) +
             chr ((data >> 24) & 0xff))
    else:
        # Python 3 - type is 'bytes', not 'str'
        s = bytes ([data & 0xff,
                    (data >> 8) & 0xff,
                    (data >> 16) & 0xff,
                    (data >> 24) & 0xff])
    return elf_data [0:offset] + s + elf_data [offset+4:]

#############################################################################
# CRC stuff
#############################################################################

def bit_reverse_8(x):
    return int('{:08b}'.format(x)[::-1], 2)
    
def bit_reverse_32(x):
    return int('{:032b}'.format(x)[::-1], 2)
    
def make_crc_table (poly):
    global crc_table
    crc_table = []
    for n in range (256):
        c = n
        for k in range (8):
            if c & 1:
                c = poly ^ (c >> 1)
            else:
                c >>= 1
        crc_table.append (c)


def compute_crc (reg, b):
    b2 = bit_reverse_8(b)
    v = (reg >> 8) ^ crc_table [(reg ^ b2) & 0xff]
    #print("byte %02x, crc %08x, bit-rev crc %08x" % (b, v, bit_reverse_32(v)))
    return v

# The 'ranges' parameter is an interable containing two-element tuples
# that give address ranges.
def elf_crc_range (prog_headers, elf_data, ranges, init_reg, final_xor):
    crc_reg = init_reg
    ph_start = -1
    ph_end = -1
    for r in ranges:
        for addr in range (r[0], r[1]):
            if addr < ph_start or addr >= ph_end:
                prog_header = find_prog_header (prog_headers, addr)
                ph_start = prog_header.p_paddr
                ph_end = prog_header.p_paddr + prog_header.p_filesz
                ph_addr_offset = prog_header.p_offset - prog_header.p_paddr
            if type(elf_data) == str:
                # Python 2.7
                b = ord (elf_data [addr + ph_addr_offset])
            else:
                # Python 3 - type is 'bytes', not 'str'
                b = elf_data [addr + ph_addr_offset]
            crc_reg = compute_crc (crc_reg, b)
    crc_reg ^= final_xor
    return bit_reverse_32(crc_reg)


#############################################################################
# main
#############################################################################

parser = get_parser ()

args = parser.parse_args ()
if args.verbose:
    print(args)

elf_data = args.infile.read ()

prog_headers = get_elf_prog_headers (elf_data)

(start_pa, end_pa) = get_elf_flash_image_bounds (prog_headers)
if args.verbose:
    print("start pa: %08x" % start_pa)
    print("  end pa: %08x" % end_pa)
image_length = get_elf_flash_image_length (prog_headers)
if args.verbose:
    print("image length: %08x" % image_length)
if args.physaddr is not None:
    start_pa = args.physaddr
    print("adjusted start pa: %08x" % start_pa)

# For info on use of the CRC-32 algorithm for application image verification,
# see Kinetis Bootloader v1.2.0 Reference Manual, section 2.7.
# Note that this is different that the bootloader protocol CRC.


bca_offset = 0x3c0
bca_size = 0x040

tag_offset                        = 0x00   # 32 bit
crcStartAddress_offset            = 0x04   # 32 bit
crcByteCount_offset               = 0x08   # 32 bit
crcExpectedValue_offset           = 0x0c   # 32 bit
enabledPeripherals_offset         = 0x10   #  8 bit
i2cSlaveAddress_offset            = 0x11   #  8 bit
peripheralDetectionTimeout_offset = 0x12   # 16 bit
usbVid_offset                     = 0x14   # 16 bit
usbPid_offset                     = 0x16   # 16 bit
usbStringsPointer_offset          = 0x18   # 32 bit
clockFlags_offset                 = 0x1c   #  8 bit
clockDivider_offset               = 0x1d   #  8 bit
bootFlags_offset                  = 0x1e   #  8 bit, 0xfe for direct boot
pad0_offset                       = 0x1f   #  8 bit, set to 0xff
mmcauConfigPointer_offset         = 0x20   # 32 bit
keyBlobPointer                    = 0x24   # 32 bit
pad1_offset                       = 0x28   #  8 bit, set to 0xff
conConfig1_offset                 = 0x29   #  8 bit
conConfig2_offset                 = 0x2a   # 16 bit
conTxId_offset                    = 0x2c   # 16 bit
conRxId_offset                    = 0x2e   # 16 bit


# clear BCA area
for offset in range(bca_offset, bca_offset + bca_size, 4):
    elf_data = insert32 (prog_headers,
                         elf_data,
                         start_pa + offset,
                         0xffffffff)

elf_data = insert16 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + usbVid_offset,
                     args.vid)

elf_data = insert16 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + usbPid_offset,
                     args.pid)

elf_data = insert32 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + tag_offset,
                     0x6766636b)  # 'kcfg', little-endian

elf_data = insert32 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + crcStartAddress_offset,
                     start_pa)

elf_data = insert32 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + crcByteCount_offset,
                     end_pa - start_pa)



# CRC-32polynomial in reversed form - normal is 0x04C11DB7
# see https://en.wikipedia.org/wiki/Polynomial_representations_of_cyclic_redundancy_checks
make_crc_table (0xedb88320)

crc_offset = bca_offset + 0x0c

crc_val = elf_crc_range (prog_headers,
                         elf_data,
                         [(start_pa, start_pa + bca_offset + crcExpectedValue_offset),
                          (start_pa + bca_offset + crcExpectedValue_offset + 4, end_pa)],
                         init_reg = 0xffffffff,
                         final_xor = 0x00000000)

# BCA crcExpectedValue
elf_data = insert32 (prog_headers,
                     elf_data,
                     start_pa + bca_offset + crcExpectedValue_offset,
                     crc_val)

# Write the modified image
if args.elf is not None:
    args.elf.write (elf_data)

if args.bin is not None:
    ph_start = -1
    ph_end = -1
    for addr in range(start_pa, end_pa):
        if addr < ph_start or addr >= ph_end:
            prog_header = find_prog_header (prog_headers, addr)
            ph_start = prog_header.p_paddr
            ph_end = prog_header.p_paddr + prog_header.p_filesz
            ph_addr_offset = prog_header.p_offset - prog_header.p_paddr
        b = elf_data [addr + ph_addr_offset]
        args.bin.write(b)

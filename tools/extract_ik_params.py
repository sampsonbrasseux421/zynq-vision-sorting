# -*- coding: utf-8 -*-
"""从 ARM 二进制里直接读出 IK 的真实参数（符号未 strip）"""
import struct
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ELF = r"D:\share\fpga\ik_zdt_pl.elf"
data = open(ELF, "rb").read()
print("文件 %d 字节" % len(data))

# ---- ELF32 header ----
assert data[:4] == b"\x7fELF", "不是 ELF"
is64 = data[4] == 2
endian = "<" if data[5] == 1 else ">"
print("ELF%d, %s" % (64 if is64 else 32, "LE" if endian == "<" else "BE"))

if is64:
    e_shoff, = struct.unpack_from(endian + "Q", data, 0x28)
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(endian + "HHH", data, 0x3A)
else:
    e_shoff, = struct.unpack_from(endian + "I", data, 0x20)
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(endian + "HHH", data, 0x2E)

# ---- section headers ----
secs = []
for i in range(e_shnum):
    off = e_shoff + i * e_shentsize
    if is64:
        name, typ, flags, addr, offset, size, link, info, align, entsize = \
            struct.unpack_from(endian + "IIQQQQIIQQ", data, off)
    else:
        name, typ, flags, addr, offset, size, link, info, align, entsize = \
            struct.unpack_from(endian + "IIIIIIIIII", data, off)
    secs.append(dict(name=name, typ=typ, addr=addr, off=offset, size=size,
                     link=link, entsize=entsize))

shstr = secs[e_shstrndx]


def sname(idx):
    p = shstr["off"] + idx
    e = data.index(b"\x00", p)
    return data[p:e].decode("latin-1")


for s in secs:
    s["sname"] = sname(s["name"])

print("section:", ", ".join(s["sname"] for s in secs if s["sname"] in (".text", ".data", ".bss", ".symtab", ".rodata")))

# ---- symbols ----
WANT = ("g_link1", "g_link2", "g_base_h", "g_tool_x", "g_tool_z",
        "g_j1off", "g_j2z", "g_j3z", "g_j4z", "g_elbow", "g_swap")

syms = {}
for s in secs:
    if s["sname"] != ".symtab":
        continue
    strtab = secs[s["link"]]
    n = s["size"] // (16 if is64 else 16)
    for i in range(n):
        off = s["off"] + i * 16
        if is64:
            nm, info, other, shndx, value, size = struct.unpack_from(endian + "IBBHQQ", data, off)
        else:
            nm, value, size, info, other, shndx = struct.unpack_from(endian + "IIIBBH", data, off)
        p = strtab["off"] + nm
        e = data.index(b"\x00", p)
        name = data[p:e].decode("latin-1")
        if name in WANT:
            syms[name] = dict(value=value, size=size, shndx=shndx, info=info)

print()
print("找到符号:", len(syms), "/", len(WANT))
if not syms:
    print("!! 没有符号，尝试直接搜索数据")
    raise SystemExit(1)

# ---- 取符号所在段，读值 ----
def read_sym(name):
    s = syms[name]
    sh = secs[s["shndx"]]
    if sh["typ"] == 8:     # NOBITS (.bss)
        return 0.0, "bss"
    off = sh["off"] + (s["value"] - sh["addr"])
    size = s["size"] or 8
    if size >= 8:
        v, = struct.unpack_from(endian + "d", data, off)
        return v, "double"
    else:
        v, = struct.unpack_from(endian + "i", data, off)
        return v, "int"


print()
print("=== 真实参数 ===")
for name in WANT:
    if name in syms:
        v, kind = read_sym(name)
        print("  %-10s = %-14s (%s)" % (name, v, kind))
    else:
        print("  %-10s 未找到" % name)

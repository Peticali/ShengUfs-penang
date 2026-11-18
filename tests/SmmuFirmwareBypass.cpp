// SPDX-License-Identifier: BSD-2-Clause-Patent
// The moto g53 (SM4350) apps SMMU as its firmware leaves it, read on hardware by native-ufs-probe=7 and =8
// (2026-11-18): the UFS stream 0x60 keeps the firmware's pass-through, and nothing is written.
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <map>
#include "../SmmuTakeover.h"

static bool gFirmware = true;
extern "C" uint32_t native_ufs_smmu_span (void) { return 0x100000; }
extern "C" bool native_ufs_smmu_firmware_bypass (void) { return gFirmware; }

struct Smmu {
  std::map<unsigned, uint32_t> regs;
  unsigned writes = 0;
  uint32_t read (unsigned offset) const { auto it = regs.find (offset); return it == regs.end () ? 0 : it->second; }
};

static Smmu penang ()
{
  Smmu s;
  s.regs[0x00] = 0x002c0406; s.regs[0x20] = 0x4c017e35; s.regs[0x24] = 0x50000030;
  s.regs[0x28] = 0x00000555;   // 48-bit IAS/OAS/UBS (IDR2 widths are not on the probe screens; 0x555 = 48 bits each)
  const unsigned pages = 64;
  // Five valid stream-match groups, all translating (probe 7: valid=5, t0=5). SMR 3 is the one the probe read; the other
  // four are stand-ins with the same type (the probe reports only their count), pointing at other banks.
  s.regs[0x800 + 4 * 0] = 0x80000400; s.regs[0xc00 + 4 * 0] = 0x00000000;
  s.regs[0x800 + 4 * 1] = 0x80000401; s.regs[0xc00 + 4 * 1] = 0x00000001;
  s.regs[0x800 + 4 * 2] = 0x80000801; s.regs[0xc00 + 4 * 2] = 0x00000003;
  s.regs[0x800 + 4 * 3] = 0x80000060; s.regs[0xc00 + 4 * 3] = 0x00000002;
  s.regs[0x800 + 4 * 4] = 0x80000c00; s.regs[0xc00 + 4 * 4] = 0x00000003;
  for (unsigned bank = 0; bank < 48; ++bank) s.regs[(pages + bank) * 4096] = 0xe0;
  for (unsigned bank : {0u, 1u, 2u, 3u}) { s.regs[0x1000 + 4 * bank] = 0x0001f000; s.regs[0x1800 + 4 * bank] = 1; }
  s.regs[(pages + 47) * 4096] = 0x19ec0104;
  return s;
}

int main ()
{
  const uint64_t dma = 0x9f000000ULL, bytes = 1 << 20;
  {
    Smmu s = penang ();
    auto read = [&s] (unsigned o) { return s.read (o); };
    auto write = [&s] (unsigned o, uint32_t v) { s.regs[o] = v; ++s.writes; };
    UfsSmmuPlan p = planUfsSmmu (0x60, dma, bytes, read);
    assert (!p.error && p.firmware && p.reuse && p.slot == 3 && p.bank == 2 && p.pages == 64);
    assert (configureUfsSmmu (p, 0x60, read, write) == nullptr);
    assert (s.writes == 0);
  }
  {
    // A bank that translates (SCTLR.M=1) is not a pass-through: the firmware path refuses it.
    Smmu s = penang ();
    s.regs[(64 + 2) * 4096] = 0xe1;
    auto read = [&s] (unsigned o) { return s.read (o); };
    UfsSmmuPlan p = planUfsSmmuFirmware (0x60, dma, bytes, read);
    assert (p.error);
  }
  {
    // Without the SoC's opt-in the stock planner runs and, as on hardware, refuses the firmware's mapping.
    gFirmware = false;
    Smmu s = penang ();
    auto read = [&s] (unsigned o) { return s.read (o); };
    UfsSmmuPlan p = planUfsSmmu (0x60, dma, bytes, read);
    assert (p.error);
    gFirmware = true;
  }
  {
    // A changed mapping between plan and configure is caught by the readback.
    Smmu s = penang ();
    auto read = [&s] (unsigned o) { return s.read (o); };
    auto write = [&s] (unsigned o, uint32_t v) { s.regs[o] = v; ++s.writes; };
    UfsSmmuPlan p = planUfsSmmu (0x60, dma, bytes, read);
    s.regs[0xc00 + 4 * 3] = 0x00000005;
    assert (configureUfsSmmu (p, 0x60, read, write) != nullptr && s.writes == 0);
  }
  std::puts ("PASS firmware pass-through reuse on the moto g53 SMMU state");
  return 0;
}

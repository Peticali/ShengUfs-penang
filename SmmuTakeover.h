// SPDX-License-Identifier: BSD-2-Clause-Patent
// Plan before any SMMU write. Interface register facts: ARM SMMUv2.
extern "C" uint32_t native_ufs_smmu_span (void);   // the SoC's apps SMMU register span (NativeUfs.cpp)
extern "C" bool native_ufs_smmu_firmware_bypass (void);   // the SoC keeps the firmware's UFS pass-through (NativeUfs.cpp)
struct UfsSmmuPlan {
  unsigned slot = 0xffffffffU, bank = 0, pages = 0;
  uint32_t smr = 0, s2cr = 0;
  bool reuse = false;
  bool firmware = false;   // the firmware's own pass-through mapping of the stream, used as it is: nothing is written
  const char *error = nullptr;
};
// ARM IHI 0062D.c sections 1.6 and 3.7: VA64=0 still rejects inputs above
// 32 bits with SCTLR.M=0. With VA64=1 the bypass range is bounded by all
// three hardware widths, not TCR2.PASize (which only limits translation).
static unsigned ufsSmmuAddressBits(unsigned code)
{
  switch (code) {
    case 0: return 32; case 1: return 36; case 2: return 40;
    case 3: return 42; case 4: return 44; case 5: return 48;
    default: return 0;
  }
}
// One attempt with a given context bank (counted from the top). The caller falls back to lower banks when this one is held by another stream.
template <class Read>
static UfsSmmuPlan planUfsSmmuBank(uint16_t stream, uint64_t dma, uint64_t bytes, Read read, unsigned bankFromTop)
{
  UfsSmmuPlan p;
  uint32_t idr0 = read(0x20), idr2 = read(0x28);
  unsigned count = idr0 & 255;
  if (!(idr0 & (1U << 30)) || !(idr2 & (1U << 12))) {
    p.error = "SMMU AArch64 stage 1 unsupported"; return p;
  }
  for (unsigned shift = 0; shift <= 8; shift += 4) {
    unsigned bits = ufsSmmuAddressBits((idr2 >> shift) & 15);
    if (!bits || !bytes || dma >= (1ULL << bits) || bytes > (1ULL << bits) - dma) {
      p.error = "DMA exceeds SMMU address width"; return p;
    }
  }
  uint32_t idr1 = read(0x24);
  unsigned banks = idr1 & 255;
  p.pages = 1U << (((idr1 >> 28) & 7) + 1);
  if ((idr1 >> 31) || !count || p.pages > 128 || banks < 2 || banks > 128 ||
      (p.pages + banks) * 4096 > native_ufs_smmu_span ()) {
    p.error = "unsupported SMMU geometry"; return p;
  }
  if (count > 128) count = 128; // Firmware's reliably accessible groups only.
  if (bankFromTop >= banks - 1) {p.error = "no free context bank"; return p;}
  p.bank = banks - 1 - bankFromTop;
  unsigned matches = 0;
  for (unsigned i = 0; i < count; ++i) {
    uint32_t smr = read(0x800 + 4 * i);
    if (!(smr & 0x80000000U)) {
      if (!p.reuse) p.slot = i;
      continue;
    }
    uint32_t s2cr = read(0xc00 + 4 * i);
    uint32_t mask = (smr >> 16) & 0x7fff;
    bool match = ((smr ^ stream) & 0xffff & ~mask) == 0;
    bool ourBank = ((s2cr >> 16) & 3) == 0 && (s2cr & 255) == p.bank;
    if (match) {
      ++matches;
      if (matches > 1 || mask || !ourBank) {
        p.slot = i; p.smr = smr; p.s2cr = s2cr;
        p.error = "conflicting UFS stream mapping"; return p;
      }
      p.reuse = true; p.slot = i;
    } else if (ourBank) {
      p.slot = i; p.smr = smr; p.s2cr = s2cr;
      p.error = "reserved context bank already used"; return p;
    }
  }
  uint32_t sctlr = read((p.pages + p.bank) * 4096);
  if ((sctlr & 1) || (p.reuse && (sctlr || read(0x1000 + 4 * p.bank) != 0x10000 ||
                                (read(0x1800 + 4 * p.bank) & ~1U)))) {
    p.error = "reserved context bank not bypass"; return p;
  }
  if (p.slot == 0xffffffffU) p.error = "no free stream-match group";
  return p;
}

// The firmware's own pass-through for the stream (moto g53, probes 7 and 8 on hardware 2026-11-18: SMR 3 = 0x80000060,
// S2CR 0x00000002 -> bank 2 with CBAR 0x0001f000 (TYPE 1, stage 1 translation with stage 2 bypass), CBA2R 1 (VA64),
// SCTLR 0x000000e0 (M=0: no translation), TTBR0 0, no fault). That is the same physical pass-through this driver would
// build, already in place: the earlier penang driver did its DMA through it without touching the SMMU. It is used only
// when the stream has exactly one exact match, translating to a bank with SCTLR.M=0, CBAR.TYPE=1 and VA64, and the DMA
// lies within the SMMU's address widths; anything else falls back to the planner below.
template <class Read>
static UfsSmmuPlan planUfsSmmuFirmware(uint16_t stream, uint64_t dma, uint64_t bytes, Read read)
{
  UfsSmmuPlan p;
  uint32_t idr0 = read(0x20), idr1 = read(0x24), idr2 = read(0x28);
  for (unsigned shift = 0; shift <= 8; shift += 4) {
    unsigned bits = ufsSmmuAddressBits((idr2 >> shift) & 15);
    if (!bits || !bytes || dma >= (1ULL << bits) || bytes > (1ULL << bits) - dma) {
      p.error = "DMA exceeds SMMU address width"; return p;
    }
  }
  unsigned count = idr0 & 255, banks = idr1 & 255;
  p.pages = 1U << (((idr1 >> 28) & 7) + 1);
  if ((idr1 >> 31) || !count || p.pages > 128 || !banks || banks > 128 ||
      (p.pages + banks) * 4096 > native_ufs_smmu_span ()) {
    p.error = "unsupported SMMU geometry"; return p;
  }
  if (count > 128) count = 128;
  unsigned matches = 0;
  for (unsigned i = 0; i < count; ++i) {
    uint32_t smr = read(0x800 + 4 * i);
    if ((smr & 0x80000000U) && (((smr ^ stream) & 0xffff & ~((smr >> 16) & 0x7fff)) == 0)) {
      if (++matches == 1) { p.slot = i; p.smr = smr; p.s2cr = read(0xc00 + 4 * i); }
    }
  }
  if (matches != 1 || p.smr != (0x80000000U | stream) || ((p.s2cr >> 16) & 3) != 0 || (p.s2cr & 255) >= banks) {
    p.error = "no exclusive firmware stream mapping"; return p;
  }
  p.bank = p.s2cr & 255;
  uint32_t cbar = read(0x1000 + 4 * p.bank), cba2r = read(0x1800 + 4 * p.bank), sctlr = read((p.pages + p.bank) * 4096);
  if (((cbar >> 16) & 3) != 1 || (sctlr & 1) || !(cba2r & 1)) {
    p.error = "firmware bank is not a VA64 pass-through"; return p;
  }
  p.reuse = true;
  p.firmware = true;
  return p;
}

// The UFS stream's context bank: the top bank first. A stream mapping left behind by an earlier run (or by another OS that used the SMMU before a
// warm reset) can still point another stream at that bank; rather than refuse to boot, the next banks down are tried, each held to the same checks
// (no other valid stream maps it, translation off). Any other refusal (a conflicting mapping of our own stream, no free group, unsupported geometry)
// is final and is reported as the first attempt saw it.
template <class Read>
static UfsSmmuPlan planUfsSmmu(uint16_t stream, uint64_t dma, uint64_t bytes, Read read)
{
  enum {kBankAttempts = 8};
  if (native_ufs_smmu_firmware_bypass ()) {
    UfsSmmuPlan firmware = planUfsSmmuFirmware(stream, dma, bytes, read);
    if (!firmware.error) return firmware;
  }
  UfsSmmuPlan first = planUfsSmmuBank(stream, dma, bytes, read, 0);
  if (!first.error) return first;
  // "conflicting" is retried too: our own stream may already sit on a lower bank from an earlier run that had to fall back.
  auto retryable = [](const char *e) {
    return !__builtin_strcmp(e, "reserved context bank already used") || !__builtin_strcmp(e, "reserved context bank not bypass") ||
           !__builtin_strcmp(e, "conflicting UFS stream mapping");
  };
  if (!retryable(first.error)) return first;
  for (unsigned attempt = 1; attempt < kBankAttempts; ++attempt) {
    UfsSmmuPlan next = planUfsSmmuBank(stream, dma, bytes, read, attempt);
    if (!next.error) return next;
    if (!retryable(next.error)) break;
  }
  return first;
}

// The caller must first quiesce the UFS queue. The planner refuses other
// streams sharing this bank and enabled translation; no live context is
// disabled here. Write callbacks must complete an MMIO barrier before return.
template <class Read, class Write>
static const char *configureUfsSmmu(const UfsSmmuPlan &p, uint16_t stream, Read read, Write write)
{
  if (p.error) return p.error;
  unsigned cb = (p.pages + p.bank) * 4096;
  if (p.firmware) {
    // Nothing is written: only check that the firmware's mapping is still the one planned.
    if (read(0x800 + 4 * p.slot) != p.smr || read(0xc00 + 4 * p.slot) != p.s2cr || (read(cb) & 1) ||
        ((read(0x1000 + 4 * p.bank) >> 16) & 3) != 1 || !(read(0x1800 + 4 * p.bank) & 1))
      return "firmware stream mapping readback";
    return nullptr;
  }
  if (!p.reuse) write(cb, 0);
  if (read(cb) != 0) return "SMMU bypass disable readback";
  // Set VA64 before exposing a newly allocated stream. Repair an exclusive
  // existing 32-bit bypass too; do not silently reuse the faulty state.
  if (read(0x1800 + 4 * p.bank) != 1) write(0x1800 + 4 * p.bank, 1);
  if (!p.reuse) write(0x1000 + 4 * p.bank, 0x10000);
  if (read(0x1800 + 4 * p.bank) != 1 || read(0x1000 + 4 * p.bank) != 0x10000 || read(cb) != 0)
    return "SMMU VA64 bypass readback";
  if (!p.reuse) {
    write(0xc00 + 4 * p.slot, p.bank);
    if (read(0xc00 + 4 * p.slot) != p.bank) return "SMMU stream context readback";
    write(0x800 + 4 * p.slot, 0x80000000U | stream);
  }
  if (read(0x800 + 4 * p.slot) != (0x80000000U | stream) ||
      read(0xc00 + 4 * p.slot) != p.bank)
    return "SMMU stream mapping readback";
  return nullptr;
}

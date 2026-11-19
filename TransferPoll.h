// SPDX-License-Identifier: BSD-2-Clause-Patent
// Single-slot completion policy. Port supplies ordered MMIO, invalidate-only
// descriptor reads, a monotonic deadline and bounded pacing. No resubmission.
// Lost: the deadline passed with the command accepted (DB=1 seen) and its doorbell bit clear again, yet the descriptor's OCS never left
// 0xf. The controller has released the slot (UFSHCI: a clear doorbell bit is a free entry) without a completion the CPU can see; the
// command may be sent again. A command never seen with DB=1 stays a Timeout (IMG_2971).
enum class UfsTransferEnd { Completed, Fatal, Timeout, Lost };
struct UfsTransferPoll {
  uint32_t polls = 0, doorbellSeen = 0, interruptSeen = 0, ocs = 15, lastDoorbell = 1;
};
static constexpr uint32_t kUfsFatalStatus = 0x71880U;

template <class Port>
static UfsTransferEnd pollUfsTransfer(Port &port, UfsTransferPoll &state)
{
  for (;;) {
    ++state.polls;
    uint32_t doorbell = port.doorbell();
    state.lastDoorbell = doorbell & 1;
    state.doorbellSeen |= doorbell;
    state.interruptSeen |= port.interrupt();
    if (state.interruptSeen & kUfsFatalStatus) return UfsTransferEnd::Fatal;
    if (!(doorbell & 1)) {
      state.ocs = port.completedOcs();
      // A zero doorbell alone does not prove that this command was accepted.
      // Also allow a completion so fast that software never observes DB=1.
      if (state.ocs != 15) return UfsTransferEnd::Completed;
    }
    if (port.expired())
      return (state.doorbellSeen & 1) && !state.lastDoorbell ? UfsTransferEnd::Lost : UfsTransferEnd::Timeout;
    port.delay();
  }
}

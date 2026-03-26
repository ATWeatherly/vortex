#pragma once
#ifdef VM_ENABLE
#include <VX_config.h>
#include <VX_types.h>
#include <common.h>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <random>
#include <util.h>

// Address mode constants matching sim/common/mem.h — needed by both sim and
// FPGA runtimes to interpret VM_ADDR_MODE without pulling in sim-only headers.
#ifndef BARE
#define BARE 0x0
#endif
#ifndef SV32
#define SV32 0x1
#endif
#ifndef SV39
#define SV39 0x8
#endif

// VM data structures (PTE_t, vAddr_t) — canonical definition lives here so
// both sim (rtlsim) and FPGA runtimes (XRT, OPAE) can access them.
// sim/common/mem.h sets VORTEX_VM_TYPES_DEFINED before including its own
// copy to prevent the duplicate definition below from being emitted there.
// Page_Fault_Exception — stripped of the ACCESS_TYPE field so it doesn't
// depend on sim-only headers. The full version (with addr/type) lives in
// sim/common/mem.h and takes precedence there via VORTEX_PAGE_FAULT_EXCEPTION_DEFINED.
#ifndef VORTEX_PAGE_FAULT_EXCEPTION_DEFINED
#define VORTEX_PAGE_FAULT_EXCEPTION_DEFINED
#include <stdexcept>
#include <string>
namespace vortex {
class Page_Fault_Exception : public std::runtime_error {
public:
    Page_Fault_Exception(const std::string& what = "") : std::runtime_error(what) {}
};
} // namespace vortex
#endif // VORTEX_PAGE_FAULT_EXCEPTION_DEFINED

#ifndef VORTEX_VM_TYPES_DEFINED
#define VORTEX_VM_TYPES_DEFINED

namespace vortex {

class PTE_t {
  private:
    uint64_t address;
    uint64_t bits(uint64_t input, uint8_t s_idx, uint8_t e_idx) {
        return (input >> s_idx) & (((uint64_t)1 << (e_idx - s_idx + 1)) - 1);
    }
    bool bit(uint64_t input, uint8_t idx) {
        return (input) & ((uint64_t)1 << idx);
    }
  public:
#if VM_ADDR_MODE == SV39
    bool N;
    uint8_t PBMT;
#endif
    uint64_t ppn;
    uint32_t rsw;
    uint32_t flags;
    uint8_t level;
    bool d, a, g, u, x, w, r, v;
    uint64_t pte_bytes;

    void set_flags(uint32_t flag) {
        this->flags = flag;
        d = bit(flags,7); a = bit(flags,6); g = bit(flags,5); u = bit(flags,4);
        x = bit(flags,3); w = bit(flags,2); r = bit(flags,1); v = bit(flags,0);
    }

    PTE_t(uint64_t address, uint32_t flags) : address(address) {
#if VM_ADDR_MODE == SV39
        N = 0; PBMT = 0; level = 3;
        ppn = address >> MEM_PAGE_LOG2_SIZE;
        set_flags(flags);
        pte_bytes = (ppn << 10) | flags;
#else
        assert((address >> 32) == 0 && "Upper 32 bits are not zero!");
        level = 2;
        ppn = address >> MEM_PAGE_LOG2_SIZE;
        set_flags(flags);
        pte_bytes = (ppn << 10) | flags;
#endif
    }

    PTE_t(uint64_t pte_bytes) : pte_bytes(pte_bytes) {
#if VM_ADDR_MODE == SV39
        N = bit(pte_bytes,63); PBMT = (uint8_t)bits(pte_bytes,61,62); level = 3;
        ppn = bits(pte_bytes,10,53);
        address = ppn << MEM_PAGE_LOG2_SIZE;
#else
        assert((pte_bytes >> 32) == 0 && "Upper 32 bits are not zero!");
        level = 2;
        ppn = bits(pte_bytes,10,31);
        address = ppn << MEM_PAGE_LOG2_SIZE;
#endif
        rsw = (uint32_t)bits(pte_bytes,8,9);
        set_flags((uint32_t)bits(pte_bytes,0,7));
    }
};

class vAddr_t {
  private:
    uint64_t address;
    uint64_t bits(uint8_t s_idx, uint8_t e_idx) {
        return (address >> s_idx) & (((uint64_t)1 << (e_idx - s_idx + 1)) - 1);
    }
  public:
    uint64_t *vpn;
    uint64_t pgoff;
    uint8_t level;

    vAddr_t(uint64_t address) : address(address) {
#if VM_ADDR_MODE == SV39
        level = 3;
        vpn = new uint64_t[level];
        vpn[2] = bits(30,38); vpn[1] = bits(21,29); vpn[0] = bits(12,20);
        pgoff = bits(0,11);
#else
        assert((address >> 32) == 0 && "Upper 32 bits are not zero!");
        level = 2;
        vpn = new uint64_t[level];
        vpn[1] = bits(22,31); vpn[0] = bits(12,21);
        pgoff = bits(0,11);
#endif
    }
    ~vAddr_t() { delete[] vpn; }
};

} // namespace vortex

#endif // VORTEX_VM_TYPES_DEFINED

// Callbacks that VMManager uses to access device memory and DCRs.
// Implementations differ per runtime (rtlsim uses RAM/Processor objects;
// FPGA runtimes use DMA upload/download + MMIO dcr_write).
struct VMDevice {
    // Write `size` bytes from `src` to physical device address `phys_addr`.
    std::function<int(uint64_t phys_addr, const uint8_t* src, uint64_t size)> mem_write;
    // Read `size` bytes from physical device address `phys_addr` into `dst`.
    std::function<int(uint64_t phys_addr, uint8_t* dst, uint64_t size)>       mem_read;
    // Write a DCR register.
    std::function<int(uint32_t addr, uint32_t value)>                         dcr_write;
};

class VMManager {
  public:
    VMManager(const VMDevice& dev);
    ~VMManager();

    int init();
    int phy_to_virt_map(uint64_t size, uint64_t* dev_pAddr, uint32_t flags);
    bool need_trans(uint64_t dev_pAddr);
    uint64_t page_table_walk(uint64_t vAddr_bits);
    uint64_t map_p2v(uint64_t ppn, uint32_t flags);
    int virtual_mem_reserve(uint64_t dev_addr, uint64_t size, int flags);

  private:
    int init_page_table(uint64_t addr, uint64_t size);
    uint8_t alloc_page_table(uint64_t* pt_addr);
    int16_t update_page_table(uint64_t ppn, uint64_t vpn, uint32_t flag);

    // helpers
    uint64_t read_pte(uint64_t addr);
    void write_pte(uint64_t addr, uint64_t value = 0xbaadf00d);
    uint64_t get_pte_address(uint64_t base_ppn, uint64_t vpn);

    // SATP helpers — replaces SATP_t from sim/common/mem.h
    int set_satp_by_addr(uint64_t pt_base_addr);
    bool is_satp_unset() const;
    uint64_t get_base_ppn() const;
    uint8_t get_mode() const;

    VMDevice dev_;

    // SATP state (replaces Processor* dependency)
    uint64_t satp_;   // raw SATP value written to hardware; 0 = not yet set

    // memory allocators
    vortex::MemoryAllocator* page_table_mem_;
    vortex::MemoryAllocator* virtual_mem_;
    std::unordered_map<uint64_t, uint64_t> addr_mapping;

    // random number generator for randomized virtual address mapping
    std::mt19937_64 rng_;
    bool randomize_va_;
};

#endif // VM_ENABLE

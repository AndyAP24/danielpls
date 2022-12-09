//=================================================================================================================================
/**
 * vmsim.c
 *
 * Allocate space that is then virtually mapped, page by page, to a simulated underlying space.  Maintain page tables and follow
 * their mappings with a simulated MMU.
 **/
// =================================================================================================================================



// =================================================================================================================================
// INCLUDES

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include "bs.h"
#include "mmu.h"
#include "vmsim.h"
// =================================================================================================================================



// =================================================================================================================================
// CONSTANTS AND MACRO FUNCTIONS

#define KB(n)      (n * 1024)
#define MB(n)      (KB(n) * 1024)
#define GB(n)      (MB(n) * 1024)
 
#define DEFAULT_REAL_MEMORY_SIZE   MB(5)
#define PAGESIZE                   KB(4)
#define PT_AREA_SIZE               (MB(4) + KB(4))

#define OFFSET_MASK           (PAGESIZE - 1)
#define PAGE_NUMBER_MASK      (~OFFSET_MASK)
#define GET_UPPER_INDEX(addr) ((addr >> 22) & 0x3ff)
#define GET_LOWER_INDEX(addr) ((addr >> 12) & 0x3ff)
#define GET_OFFSET(addr)      (addr & OFFSET_MASK)
#define GET_PAGE_ADDR(addr)   (addr & PAGE_NUMBER_MASK)
#define IS_ALIGNED(addr)      ((addr & OFFSET_MASK) == 0)

#define IS_RESIDENT(pte)      (pte & PTE_RESIDENT_BIT)
#define IS_REFERENCED(pte)    (pte & PTE_REFERENCED_BIT)
#define IS_DIRTY(pte)         (pte & PTE_DIRTY_BIT)
#define SET_RESIDENT(pte)     (pte |= PTE_RESIDENT_BIT)
#define CLEAR_RESIDENT(pte)   (pte &= ~PTE_RESIDENT_BIT)
#define CLEAR_REFERENCED(pte) (pte &= ~PTE_REFERENCED_BIT)
#define CLEAR_DIRTY(pte)      (pte &= ~PTE_DIRTY_BIT)

// The boundaries and size of the real memory region.
static void*         real_base        = NULL;
static void*         real_limit       = NULL;
static uint64_t      real_size        = DEFAULT_REAL_MEMORY_SIZE;

// Where to find the next page of real memory for page table blocks.
static vmsim_addr_t  pt_free_addr     = PAGESIZE;

// Where the normal (non-page-table) range of pages begins.
static vmsim_addr_t  normal_base_addr = PT_AREA_SIZE;

// Where to find the next page of real memory for backing simulated pages, starting at the beginning.
static vmsim_addr_t  real_free_addr   = PT_AREA_SIZE;

// The base real address of the upper page table.
static vmsim_addr_t  upper_pt         = 0;

// Used by the heap allocator, the address of the next free simulated address.
static vmsim_addr_t  sim_free_addr    = 0;

// The size of the regular (non-page-table) real space.
static unsigned int  normal_real_size;

// The number of regular (non-page-table) pages.
static unsigned int  normal_real_pages;

// A pointer to the reverse mapping table (real page #'s -> PTE addresses)
static vmsim_addr_t* reverse_page_map = NULL;

// For the clock algorithm, the clock hand.
static unsigned int  clock_hand       = 0;

// TEMP: The next available block number on the backing store.  Start at 1; starting at 0 makes for a PTE that is indistinguishable
// from a null entry.
static unsigned int  bs_free_block    = 1;

static bool debug = true; 
// =================================================================================================================================



// =================================================================================================================================
/**
 * Clear the contents of a real page by setting its bytes to zero.
 *
 * \param real_page_addr The base address of the real page to zero.
 */
void
clear_page (vmsim_addr_t real_page_addr) {

  char zero_buffer[PAGESIZE];
  for (int i = 0; i < PAGESIZE; i += 1) {
    zero_buffer[i] = 0;
  }
  vmsim_write_real(zero_buffer, real_page_addr, PAGESIZE);  
  
} // clear_page ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Allocate a page of real memory space for a page table block.  Taken from a region of real memory reserved for this purpose.
 *
 * \return The _real_ base address of a page of memory for a page table block.
 */
vmsim_addr_t
allocate_pt () {

  vmsim_addr_t new_pt_addr = pt_free_addr;
  pt_free_addr += PAGESIZE;
  assert(IS_ALIGNED(new_pt_addr));
  assert(pt_free_addr <= PT_AREA_SIZE);
  clear_page(new_pt_addr);
  
  return new_pt_addr;
  
} // allocate_pt ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Swap out some page.
 *
 * \return the _real_ base address of the page of memory whose contents were copied to the backing store.
 */
vmsim_addr_t
swap_out () {

  vmsim_addr_t real_page_addr;

  vmsim_addr_t current_pte_addr; 

  pt_entry_t current_pte; 
  // COMPLETE ME:  Select a real page to swap out to the backing store, and then return the base address of that real page.

  while (true){ //traversing the array of pointers to lower-level page table entries and using a system where the page which has been used farther in time is repplaced. 
    
    current_pte_addr = reverse_page_map[clock_hand  % normal_real_pages]; //variables that hold the correponding addresses at each 

    if(debug) fprintf(stderr, "DEBUGGING IN SWAP_OUT: \t current_pte_addr = %d\n and clock_hand = %d\n", current_pte_addr, clock_hand);    
     
    vmsim_read_real(&current_pte, current_pte_addr, sizeof(current_pte));

    if (!IS_REFERENCED(current_pte)){
      if(debug) fprintf(stderr, "exiting the loop"); 
      break; //the victim page was found 
    }

 
    
    else {
      // an entry whose reference bit is 1 (there's a mapping between the corresponding real page and
      //simulated page).This won't be the victim page at this clock cycle. Clearing the reference bit as when the next clock cycle will reach it,
      //it will be the page that was used the least recently.)      

      pt_entry_t new_current_pte = CLEAR_REFERENCED(current_pte); //turning off the reference bit

    // update the entry in the pt.
    
    vmsim_write_real(&new_current_pte, reverse_page_map[clock_hand % normal_real_pages], sizeof(current_pte)); 

    clock_hand++; //

    }}
  

  //the victim page is found and corresponds to the page at the page address specified by the current lower_pte.

  bs_free_block++;
  
  
  real_page_addr = current_pte_addr; //the victim page is what swap_out returns. 

  bs_write(real_page_addr, bs_free_block); //copying the data from the page that is to be evicted in the backing store.

  if(debug) fprintf(stderr, "OUT OF SWAP_OUT with real_page_addr = %x\n", real_page_addr); 
   

  current_pte = bs_free_block;

 
    
  if(debug) fprintf(stderr, "ALMOST OUT OF SWAP_OUT WITH real_page_addr = %x\n, bs_free_block = %x\n, and current_pte_addr = %x\n, and current_pte = %x\n",
		    real_page_addr, bs_free_block, current_pte_addr, current_pte); 

  current_pte = (current_pte>>1);

  SET_RESIDENT(current_pte);
  
  
  
if(debug) fprintf(stderr, " The bit shifting inside pte works if  the following gives the previous current_pte value; current_pte = %x/n ",  (current_pte<<1));  


 vmsim_write_real(&current_pte, real_page_addr, sizeof(current_pte));  //my attempt at saving the specific block in the bs from which data should be retrieved.    
  

 return real_page_addr;
  
} // swap_out ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Allocate a page of real memory space for backing a simulated page.  Taken from the general pool of real memory.
 *
 * \return The _real_ base address of a page of memory.
 */
vmsim_addr_t
allocate_real_page () {

  vmsim_addr_t new_real_addr;

  // Is there real space left?
  if (real_free_addr < real_size) {

    // Yes.  Take the next free page.
    new_real_addr = real_free_addr;
    real_free_addr += PAGESIZE;
    assert(IS_ALIGNED(new_real_addr));

  } else {

    // No, so swap out a page to create a free real page.
    new_real_addr = swap_out();

    if(debug) fprintf(stderr, "just claimed a victim page at new_real_addr = %x\n ", new_real_addr); 
  }

  
  // Clear the contents of the real page.
  clear_page(new_real_addr);

  return new_real_addr;
  
} // allocate_real_page ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_init () {

  // Only initialize if it hasn't already happened.
  if (real_base == NULL) {

    // Determine the real memory size, preferrably by environment variable, otherwise use the default.
    char* real_size_envvar = getenv("VMSIM_REAL_MEM_SIZE");
    if (real_size_envvar != NULL) {
      errno = 0;
      real_size = strtoul(real_size_envvar, NULL, 10);
      assert(errno == 0);
    }
    normal_real_size  = real_size - PT_AREA_SIZE;
    normal_real_pages = normal_real_size / PAGESIZE;
    reverse_page_map  = (vmsim_addr_t*)malloc(normal_real_pages * sizeof(vmsim_addr_t));
    assert(reverse_page_map != NULL);
    
    // Map the real storage space.
    real_base = mmap(NULL, real_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    assert(real_base != NULL);
    real_limit = (void*)((intptr_t)real_base + real_size);
    upper_pt = allocate_pt();

    // Initialize the simualted space allocator.  Leave page 0 unused, start at page 1.
    sim_free_addr = PAGESIZE;

    // Initialize the supporting components.
    mmu_init(upper_pt);
    bs_init();
    
  }
  
} // vmsim_init ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Map a _simulated_ address to a _real_ one, ensuring first that the page table and real spaces are initialized.
 *
 * \param  sim_addr        The _simulated_ address to translate.
 * \param  write_operation Whether the memory access is to _read_ (`false`) or to _write_ (`true`).
 * \return the translated _real_ address.
 */ 
vmsim_addr_t
vmsim_map (vmsim_addr_t sim_addr, bool write_operation) {

  vmsim_init();

  assert(real_base != NULL);
  vmsim_addr_t real_addr = mmu_translate(sim_addr, write_operation);
  return real_addr;
  
} // vmsim_map ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Swap in a page.
 *
 * \param real_page_addr The address of the page frame in real space into which to copy a page.
 * \param pte The page table entry of the simulated page to swap in.
 */
void
swap_in (vmsim_addr_t real_page_addr, pt_entry_t pte) {
  
  
  if(debug) fprintf(stderr, " we have real_page_addr of= %x, and pte of = %x", real_page_addr, pte); 
  
  bs_read ( real_page_addr, pte<<1);  //pte is the block that data should be retrieved and real_page_addr is the destination. 

   
} // swap_in ()
// =================================================================================================================================



// =================================================================================================================================
/**
 * Called when the translation of a _simulated_ address fails.  When this function is done, a _real_ page will back the _simulated_
 * one that contains the given address, with the page tables appropriately updated.
 *
 * \param sim_addr The _simulated_ address for which address translation failed.
 // the address you currently have is the upper 20 bits of the physical address.  
*/
void
vmsim_map_fault (vmsim_addr_t sim_addr) {

  if (debug) fprintf(stderr, "checking\n");    
  
  assert(upper_pt != 0);

  // Grab the upper table's entry.
  vmsim_addr_t upper_index    = GET_UPPER_INDEX(sim_addr);
  vmsim_addr_t upper_pte_addr = upper_pt + (upper_index * sizeof(pt_entry_t));
  pt_entry_t   upper_pte;
  vmsim_read_real(&upper_pte, upper_pte_addr, sizeof(upper_pte));

  // If the lower table doesn't exist, create it and update the upper table.
  if (upper_pte == 0) {
    // upper_pte corresponds to the lower_pt_addr; and allocate_pt() just allocated a pt. 
    upper_pte = allocate_pt();
    assert(upper_pte != 0);
    vmsim_write_real(&upper_pte, upper_pte_addr, sizeof(upper_pte));
    //update the entry at tat specific section of the upper_pte_addr. In fact, when we started
    //upper_pte didn't map to any lpt, but now it does. 
  }

  // Grab the lower table's entry.
  vmsim_addr_t lower_pt       = GET_PAGE_ADDR(upper_pte);
  vmsim_addr_t lower_index    = GET_LOWER_INDEX(sim_addr);
  vmsim_addr_t lower_pte_addr = lower_pt + (lower_index * sizeof(pt_entry_t));
  pt_entry_t   lower_pte;
  vmsim_read_real(&lower_pte, lower_pte_addr, sizeof(lower_pte));

  // If there is no mapped page, create it and update the lower table //if the translation from VP to PP has failed
  if (!IS_RESIDENT(lower_pte)) {

    vmsim_addr_t real_page_addr = allocate_real_page();

    if(debug) fprintf(stderr, "from allocating a real page() = %x\n, and lower_pte equal to = %x\n", real_page_addr, lower_pte); 
      
    if (lower_pte != 0) {

      if(debug) fprintf(stderr, "called swap_in with arguments real_page_addr = %x\n, lower_pte = %x/n", real_page_addr, lower_pte); 
	
      swap_in(real_page_addr, lower_pte);

    }
    swap_in(real_page_addr, lower_pte); 
    SET_RESIDENT(lower_pte);
    vmsim_write_real(&lower_pte, lower_pte_addr, sizeof(lower_pte));

    // Add the PTE addr to the reverse map for this page.
    unsigned int real_page_number = (real_page_addr - normal_base_addr) / PAGESIZE;
    reverse_page_map[real_page_number] = lower_pte_addr;

  }
  
} // vmsim_map_fault ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_read_real (void* buffer, vmsim_addr_t real_addr, size_t size) {

  // Get a pointer into the real space and check the bounds.
  void* ptr = real_base + real_addr;
  void* end = (void*)((intptr_t)ptr + size);
  assert(end <= real_limit);

  // Copy the requested bytes from the real space.
  memcpy(buffer, ptr, size);
  
} // vmsim_read_real ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_write_real (void* buffer, vmsim_addr_t real_addr, size_t size) {

  // Get a pointer into the real space and check the bounds.
  void* ptr = real_base + real_addr;
  void* end = (void*)((intptr_t)ptr + size);
  assert(end <= real_limit);

  // Copy the requested bytes into the real space.
  memcpy(ptr, buffer, size);
  
} // vmsim_write_real ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_read (void* buffer, vmsim_addr_t addr, size_t size) {

  vmsim_addr_t real_addr = vmsim_map(addr, false);
  vmsim_read_real(buffer, real_addr, size);

} // vmsim_read ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_write (void* buffer, vmsim_addr_t addr, size_t size) {

  vmsim_addr_t real_addr = vmsim_map(addr, true);
  vmsim_write_real(buffer, real_addr, size);

} // vmsim_write ()
// =================================================================================================================================



// =================================================================================================================================
vmsim_addr_t
vmsim_alloc (size_t size) {

  vmsim_init();

  // Pointer-bumping allocator with no reclamation.
  vmsim_addr_t addr = sim_free_addr;
  sim_free_addr += size;
  return addr;
  
} // vmsim_alloc ()
// =================================================================================================================================



// =================================================================================================================================
void
vmsim_free (vmsim_addr_t ptr) {

  // No reclamation, so nothing to do.

} // vmsim_free ()
// =================================================================================================================================

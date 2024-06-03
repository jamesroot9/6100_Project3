#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <cstring>
#include <list>
#include <vector>
#include <cstdlib>

#include "procsim.hpp"





// Define PRF
    // register block
    typedef struct {
        bool ready;
        bool free;
    } reg;

    // PRF
    uint64_t total_regs;
    reg* PRF;

// Define RAT
    uint64_t* RAT;

// Define Logic Units
    // ALU unit
    typedef struct {
        bool ready;
        inst_t inst;
    } alu;

    alu* alus;
    uint8_t num_alus;

    // MUL
    typedef struct {
        bool* ready;
        inst_t* insts;
        uint64_t open_count; // determines which instruction will be returned next 
    } mul;

    mul* muls;
    uint8_t num_muls;

    // LSU
    typedef struct {
        bool ready;
        inst_t inst;
        uint64_t cycle;
    } lsu;

    lsu* lsus;
    uint8_t num_lsus;

// Define Queues
    // Define Queue Element
    struct queue_item {
        inst_t inst; // for all
        int16_t p_src1; // for ROB, SchedQu
        int16_t p_src2; // for ROB, SchedQu
        int16_t p_dest; // for ROB, SchedQu
        int16_t prev_preg;
        bool fired;
        bool complete; // for ROB
        queue_item* next;
    };

    // Define Dispatch Queue (infinite length)
    queue_item* dispatch_head;
    queue_item* dispatch_tail;
    uint64_t dispatch_entries;

    size_t fetches;

    // Define Scheduling Queue (fixed len)
    queue_item* scheduling_head;
    queue_item* scheduling_tail;
    uint64_t scheduling_length;
    uint64_t scheduling_entries;

    // Define ROB (fixed len)
    queue_item* rob_head;
    queue_item* rob_tail;
    uint64_t rob_length;
    uint64_t rob_entries;

// Define Store Buffer
    // Define store buffer Element
    struct store_item {
        inst_t inst;
        store_item* next;
    };

    store_item* store_buffer;
    store_item* store_tail;
    uint64_t store_items;

// define result busses
    // we will just store the dynamic memory instruction number here and index elsewhere
    inst_t* result_busses;
    uint64_t num_busses;
    uint64_t items_in_bus;

    //variable to track how many store retires occur in previous round to know how many items
    // can be removed from the store buffer
    uint64_t previous_retires;

//stat values to calculate avg sizes at end of program
    uint64_t total_rob, total_disp, total_sched, total_retired;


// The helper functions in this#ifdef are optional and included here for your
// convenience so you can spend more time writing your simulator logic and less
// time trying to match debug trace formatting! (If you choose to use them)
#ifdef DEBUG

static void print_operand(int8_t rx) {
    if (rx < 0) {
        printf("(none)"); //  PROVIDED
    } else {
        printf("R%" PRId8, rx); //  PROVIDED
    }
}

// Useful in the fetch and dispatch stages
static void print_instruction(const inst_t *inst) {
    if (!inst) return;
    static const char *opcode_names[] = {NULL, NULL, "ADD", "MUL", "LOAD", "STORE", "BRANCH"};

    printf("opcode=%s, dest=", opcode_names[inst->opcode]); //  PROVIDED
    print_operand(inst->dest); //  PROVIDED
    printf(", src1="); //  PROVIDED
    print_operand(inst->src1); //  PROVIDED
    printf(", src2="); //  PROVIDED
    print_operand(inst->src2); //  PROVIDED
    printf(", dyncount=%lu", inst->dyn_instruction_count); //  PROVIDED
}

// This will print out the state of the RAT
static void print_rat(void) {
    for (uint64_t regno = 0; regno < NUM_REGS; regno++) {
        if (regno == 0) {
            printf("    { R%02" PRIu64 ": P%03" PRIu64 " }", regno, RAT[regno]); 
        } else if (!(regno & 0x3)) {
            printf("\n    { R%02" PRIu64 ": P%03" PRIu64 " }", regno, RAT[regno]); 
        } else {
            printf(", { R%02" PRIu64 ": P%03" PRIu64 " }", regno, RAT[regno]); 
        }
    }
    printf("\n"); //  PROVIDED
}

// This will print out the state of the register file, where P0-P31 are architectural registers 
// and P32 is the first PREG 
static void print_prf(void) {
    for (uint64_t regno = 0; regno < total_regs; regno++) { 
        if (regno == 0) {
            printf("    { P%03" PRIu64 ": Ready: %d, Free: %d }", regno, PRF[regno].ready, PRF[regno].free);
        } else if (!(regno & 0x3)) {
            printf("\n    { P%03" PRIu64 ": Ready: %d, Free: %d }", regno, PRF[regno].ready, PRF[regno].free);
        } else {
            printf(", { P%03" PRIu64 ": Ready: %d, Free: %d }", regno, PRF[regno].ready, PRF[regno].free);
        }
    }
    printf("\n"); //  PROVIDED
}

// This will print the state of the ROB where instructions are identified by their dyn_instruction_count
static void print_rob(void) {
    size_t printed_idx = 0;
    printf("\tAllocated Entries in ROB: %lu\n", rob_entries); 
    for (queue_item* ptr = rob_head; ptr; ptr = ptr->next) { 
        if (printed_idx == 0) {
            printf("    { dyncount=%05" PRIu64 ", completed: %d, mispredict: %d }", ptr->inst.dyn_instruction_count, ptr->complete, ptr->inst.mispredict); 
        } else if (!(printed_idx & 0x3)) {
            printf("\n    { dyncount=%05" PRIu64 ", completed: %d, mispredict: %d }", ptr->inst.dyn_instruction_count, ptr->complete, ptr->inst.mispredict); 
        } else {
            printf(", { dyncount=%05" PRIu64 " completed: %d, mispredict: %d }", ptr->inst.dyn_instruction_count, ptr->complete, ptr->inst.mispredict); 
        }
        printed_idx++;
    }
    if (!printed_idx) {
        printf("    (ROB empty)"); //  PROVIDED
    }
    printf("\n"); //  PROVIDED
}
#endif




// Optional helper function which pops previously retired store buffer entries
// and pops instructions from the head of the ROB. (In a real system, the
// destination register value from the ROB would be written to the
// architectural registers, but we have no register values in this
// simulation.) This function returns the number of instructions retired.
// Immediately after retiring a mispredicting branch, this function will set
// *retired_mispredict_out = true and will not retire any more instructions. 
// Note that in this case, the mispredict must be counted as one of the retired instructions.
static uint64_t stage_state_update(procsim_stats_t *stats,
                                   bool *retired_mispredict_out) {
    // TODO: fill me in
#ifdef DEBUG
    printf("Stage Retire: \n"); //  PROVIDED
    printf("\tPopping %lu store buffer entries that retired last cycle\n", previous_retires);
#endif

    // remove the necessary number of items (# of store retires last cycle) from the store buffer
    store_item* temp;
    for (uint64_t i = 0; i < previous_retires; i++) {
        temp = store_buffer;
        store_buffer = store_buffer->next;
        store_items--;
        if (store_items == 0) {
            store_tail = nullptr;
        }
#ifdef DEBUG
        printf("\t\tPopping back of store buffer: 0x%lx\n", temp->inst.load_store_addr);
#endif
        free(temp);
    }

#ifdef DEBUG
    printf("Checking ROB: \n");
#endif

    previous_retires = 0;

    // pop as many completed operations as possible (must be connected to head)
    queue_item* ptr = rob_head;
    queue_item* rob_temp;
    uint64_t number_retired = 0;
    // iterate from head of rob and retire as many consecutively complete instructions as possible
    while (ptr && ptr->complete) {
        number_retired++;
        stats->instructions_retired++;
        rob_head = rob_head->next;

#ifdef DEBUG
        printf("\tRetiring: ");
        print_instruction(&ptr->inst);
        printf("\n");
#endif
        // check if we retire a store, increment count for next stage update
        if (ptr->inst.opcode == OPCODE_STORE) {
            previous_retires++;
#ifdef DEBUG
            printf("\t\tRetiring store, need to pop back of store buffer next cycle\n");
#endif
        }
        rob_temp = ptr;
        ptr = ptr->next;

        // free previous preg
        if(rob_temp->prev_preg >= NUM_REGS) {
#ifdef DEBUG
            printf("\t\tFreeing: P%lu for areg: R%d\n", (uint64_t) rob_temp->prev_preg, rob_temp->inst.dest);
#endif
            PRF[rob_temp->prev_preg].free = true;
        }
        rob_entries--;
        if (rob_entries == 0) {
            rob_head = nullptr;
            rob_tail = nullptr;
        }



        // check if we are retiring a mispredict, if we do, handle
        if (rob_temp->inst.mispredict) {
            *retired_mispredict_out = true;
            free(rob_temp);
            break;
        }

        free(rob_temp);

    }

#ifdef DEBUG
    if (rob_head) 
        printf("\tROB entry %ld still in flight: dyncount=%ld\n", number_retired, rob_head->inst.dyn_instruction_count);
#endif



    return number_retired;
}

// Optional helper function which is responsible for moving instructions
// through pipelined Function Units and then when instructions complete (that
// is, when instructions are in the final pipeline stage of an FU and aren't
// stalled there), setting the ready bits in the register file. This function 
// should remove an instruction from the scheduling queue when it has completed.
static void stage_exec(procsim_stats_t *stats) {
    // TODO: fill me in
#ifdef DEBUG
    printf("Stage Exec: \n"); //  PROVIDED
#endif
    items_in_bus = 0;
    // Progress ALUs
#ifdef DEBUG
    printf("Progressing ALU units\n");  // PROVIDED
#endif
    // iterate through each alu unit
    for (uint64_t i = 0; i < num_alus; i++) {
        // if not ready, then instruction has been completed in the one cycle given
        if (!alus[i].ready) {
            // instruction complete, mark unit ready for use again
            alus[i].ready = true;
            // add this item to the result bus
            result_busses[items_in_bus] = alus[i].inst;
#ifdef DEBUG
            printf("\tCompleting ALU: %ld, for dyncount=%lu\n", i, result_busses[items_in_bus].dyn_instruction_count);
#endif
            items_in_bus++;
        }
    }

    // Progress MULs
#ifdef DEBUG
    printf("Progressing MUL units\n");  // PROVIDED
#endif

    // iterate through each mul unit
    for (uint64_t i = 0; i < num_muls; i++) {
        // advance pipeline stage
        muls[i].open_count = (muls[i].open_count + 1) % 3;
        // if the instruction is slotted, then instruction has been completed in the three cycles given
        if (!muls[i].ready[muls[i].open_count % 3]) {
            // pipeline stage 1 complete, mark unit ready for use again
            muls[i].ready[muls[i].open_count % 3] = true;
            // add this item to the result bus
            result_busses[items_in_bus] = muls[i].insts[muls[i].open_count % 3];
#ifdef DEBUG
            printf("\tCompleting MUL: %ld, for dyncount=%lu\n", i, result_busses[items_in_bus].dyn_instruction_count);
#endif
            items_in_bus++;
        }
    }

// Progress LSU loads
#ifdef DEBUG
    printf("Progressing LSU units for loads\n");  // PROVIDED
#endif
    bool increment;
    for (uint64_t i = 0; i < num_lsus; i++) {
        // check if we have an item currently in the LSU... (look if count > 0)
        increment = true;


        if (lsus[i].cycle > 0) {
            // check if load or store
            if (lsus[i].inst.opcode == OPCODE_LOAD)  {
                // lsus have a variable finishing cycle, check which cycle we are on and reture if necessary
                // have a lsu being active, check if complete
                if (lsus[i].cycle == 1) {

                    stats->reads++;
                    // check if instruction found in store buffer
                    store_item* ptr = store_buffer;
                    while (ptr) {
// #ifdef DEBUG
// printf("\t\tChecking store buffer: 0x%lx == 0x%lx\n", ptr->inst.load_store_addr, lsus[i].inst.load_store_addr);
// #endif
                        if (ptr->inst.load_store_addr == lsus[i].inst.load_store_addr) {
                            // item exists in store buffer, resolve unit
// #ifdef DEBUG
// printf(" SUCCESS!\n");
// #endif
                            stats->store_buffer_read_hits++;
                            result_busses[items_in_bus] = lsus[i].inst;
#ifdef DEBUG
                            printf("\tCompleting LSU: %ld, for dyncount=%lu, Store Buffer Hit\n", i, lsus[i].inst.dyn_instruction_count);  
#endif
                            items_in_bus++;
                            increment = false;
                            break;
                        }
                        ptr = ptr->next;
                    }
                    
                } else if (lsus[i].cycle == 2) {
                    stats->dcache_reads++;
                    if (!lsus[i].inst.dcache_miss) {
                        // instruction found in cache, we can complete the instruction
                        stats->dcache_read_hits++;
#ifdef DEBUG
                        printf("\tCompleting LSU: %ld, for dyncount=%lu, Cache Hit\n", i, lsus[i].inst.dyn_instruction_count);
#endif                  
                        result_busses[items_in_bus] = lsus[i].inst;
                        items_in_bus++;
                        increment = false;
                    } else {
                        // item not in cache
                        stats->dcache_read_misses++;
                    }
                } else if (lsus[i].cycle == 2 + L1_MISS_PENALTY) {
                    // instruction found in memory
#ifdef DEBUG
                    printf("\tCompleting LSU: %ld, for dyncount=%lu, Cache Miss\n", i, lsus[i].inst.dyn_instruction_count);
#endif
                    result_busses[items_in_bus] = lsus[i].inst;
                    items_in_bus++;
                    increment = false;

                }
                if (increment)
                    lsus[i].cycle++;
                else {
                    lsus[i].cycle = 0;
                    lsus[i].ready = true;
                }
            }
        }
    }

    // Progress LSU stores
#ifdef DEBUG
    printf("Progressing LSU units for stores\n");  // PROVIDED
#endif
    for (uint64_t i = 0; i < num_lsus; i++) {
        // check if we have an item currently in the LSU... (look if count > 0)
        increment = true;
        if (lsus[i].cycle > 0) {
            // check if load or store
            if (lsus[i].inst.opcode == OPCODE_STORE) {
                // all stores complete on cycle 1, finish up instruction
                // place item in store buffer
                store_item* new_entry = (store_item*)malloc(sizeof(store_item));
                new_entry->next = nullptr;
                new_entry->inst = lsus[i].inst;
                if (store_tail) {
                    // find the tail of the queue
                    // add new entry on the end
                    store_tail->next = new_entry;
                    store_tail = store_tail->next;
                } else {
                    //no entries in queue - make object the head
                    store_buffer = new_entry;
                    store_tail = new_entry;
                }
                store_items++;
#ifdef DEBUG
                printf("\tCompleting LSU: %ld, for dyncount=%lu, adding 0x%lx to Store Buffer\n", i, lsus[i].inst.dyn_instruction_count, lsus[i].inst.load_store_addr);
#endif
                // add to result bus
                result_busses[items_in_bus] = lsus[i].inst;
                items_in_bus++;
                increment = false;

                lsus[i].cycle = 0;
                lsus[i].ready = true;

            }
        }
    }

    // Apply Result Busses
#ifdef DEBUG
    printf("Processing Result Busses\n"); // PROVIDED
#endif

    // iterate through each result and resolve
    for (uint64_t i = 0; i < items_in_bus; i++) {
        // set dest preg to ready (if it exists) 
#ifdef DEBUG
        printf("\tProcessing Result Bus for: ");
        print_instruction(&result_busses[i]);
        printf("\n");
#endif
        

        // remove this instruction from the scheduling queue
        queue_item* ptr = scheduling_head;
        queue_item* prev_ptr = nullptr;
        while (ptr) {
            if (ptr->inst.dyn_instruction_count == result_busses[i].dyn_instruction_count) {
                // we have a match, remove
                if (ptr == scheduling_tail) {
                    scheduling_tail = prev_ptr;
                }
                if (prev_ptr) {
                    // data is in the middle of the scheduling queue, remove
                    prev_ptr->next = ptr->next;
                } else {
                    // data is at the head of scheduling queue, detach
                    scheduling_head = ptr->next;
                    ptr->next = nullptr;
                }
                scheduling_entries--;
                break;
            }
            prev_ptr = ptr;
            ptr = ptr->next;
        }

        // set instruction to complete in the ROB
        queue_item* rob_ptr = rob_head;
        // iterate through rob to find instruction
        while (rob_ptr != nullptr) {
            if (rob_ptr->inst.dyn_instruction_count == ptr->inst.dyn_instruction_count) {
                // found item, mark as complete
                rob_ptr->complete = true;
                // mark preg as ready
        if (result_busses[i].dest >= 0) {
#ifdef DEBUG
                printf("\t\tMarking preg ready: P%lu\n", (uint64_t) rob_ptr->p_dest);
#endif
                PRF[rob_ptr->p_dest].ready = true;
        }
            }
            rob_ptr = rob_ptr->next;
        }
        

        // we can now free the entry from the scheduling queue
        free(ptr);

    }
}

// Optional helper function which is responsible for looking through the
// scheduling queue and firing instructions that have their source pregs
// marked as ready. Note that when multiple instructions are ready to fire
// in a given cycle, they must be fired in program order. 
// Also, load and store instructions must be fired according to the 
// memory disambiguation algorithm described in the assignment PDF. Finally,
// instructions stay in their reservation station in the scheduling queue until
// they complete (at which point stage_exec() above should free their RS).
static void stage_schedule(procsim_stats_t *stats) {
#ifdef DEBUG
    printf("Stage Schedule: \n"); //  PROVIDED
#endif

    // iterate through scheduling queue
    queue_item* ptr = scheduling_head;

    // variables for memory disambiguation
    bool can_load = true;
    bool can_store = true;

    // track if any instructions are fired this cycle
    bool anyFired = false;

    
    while (ptr) {
        if (!ptr->fired) {

            // is instruction ready to fire (not already fired, registers ready, and logic unit open?
            // check fired and registers ready
            if (((ptr->p_src1 >=0 && PRF[ptr->p_src1].ready) || ptr->p_src1 < 0) && ((ptr->p_src2 >=0 && PRF[ptr->p_src2].ready) || ptr->p_src2 < 0)) {
#ifdef DEBUG
                printf("\tAttempting to fire instruction: ");
                print_instruction(&ptr->inst);
                printf("\n");
                printf("\t\tSrc0: %d, %d; Src1: %d, %d\n", ptr->p_src1, (ptr->p_src1 >= 0 ? PRF[ptr->p_src1].ready : true), ptr->p_src2, (ptr->p_src2 >= 0 ? PRF[ptr->p_src2].ready : true));
#endif
                
                // check if logic unit is ready
                switch (ptr->inst.opcode)
                {
                case OPCODE_ADD:
                case OPCODE_BRANCH:
                    // assign to alu unit
                    for (uint64_t i = 0; i < num_alus; i++) {
                        if (alus[i].ready) {
                            // unit is open, fire away!
                            anyFired = true;
#ifdef DEBUG
                            printf("\t\tFired to ALU: %lu\n", i);
#endif
                            alus[i].inst = ptr->inst;
                            alus[i].ready = false;
                            ptr->fired = true;
                            break;
                        }
                    }
                    break;
                
                case OPCODE_LOAD:
                    // check if we can fire a load
                    if (!can_load) break;
                    // assign to lsu unit
                    for (uint64_t i = 0; i < num_lsus; i++) {
                        if (lsus[i].ready) {
                            // unit is open, fire away!
                            anyFired = true;
#ifdef DEBUG
                            printf("\t\tFired Load to LSU: %lu\n", i);
#endif                  
                            lsus[i].ready = false;
                            lsus[i].inst = ptr->inst;
                            lsus[i].cycle = 1;
                            ptr->fired = true;
                            break;
                        }
                    }
                    break;
                case OPCODE_STORE:
                    // check if we can fire a store
                    if (!can_store) break;
                    // assign to lsu unit
                    for (uint64_t i = 0; i < num_lsus; i++) {
                        if (lsus[i].ready) {
                            // unit is open, fire away!
                            anyFired = true;
#ifdef DEBUG
                            printf("\t\tFired Store to LSU: %lu\n", i);
#endif
                            lsus[i].ready = false;
                            lsus[i].inst = ptr->inst;
                            lsus[i].cycle = 1;
                            ptr->fired = true;
                            break;
                        }
                    }
                    break;
                
                default:
                    // multiply, assign to mul unit
                    for (uint64_t i = 0; i < num_muls; i++) {
                        if (muls[i].ready[muls[i].open_count % 3]) {
                            // unit is open, fire away!
                            anyFired = true;
#ifdef DEBUG
                            printf("\t\tFired to MUL: %lu\n", i);
#endif
                            muls[i].insts[muls[i].open_count % 3] = ptr->inst;
                            muls[i].ready[muls[i].open_count % 3] = false;
                            ptr->fired = true;
                            break;
                        }
                    }
                    break;
                }
            }
        }
        if (ptr->inst.opcode == OPCODE_STORE) {
            can_load = false;
            can_store = false;
        }
        if (ptr->inst.opcode == OPCODE_LOAD) {
            can_store = false;
        }
        ptr = ptr->next;
    }
    // Debug purposes, check if any instructions were fired and comment if they werent
    if (!anyFired) {
#ifdef DEBUG
        printf("\tCould not find scheduling queue entry to fire this cycle\n");
#endif
        stats->no_fire_cycles++;
    }
    
}

// Optional helper function which looks through the dispatch queue, decodes
// instructions, and inserts them into the scheduling queue. Dispatch should
// not add an instruction to the scheduling queue unless there is space for it
// in the scheduling queue and the ROB and a free preg exists if necessary; 
// effectively, dispatch allocates pregs, reservation stations and ROB space for 
// each instruction dispatched and stalls if there any are unavailable. 
// You will also need to update the RAT if need be.
// Note the scheduling queue has a configurable size and the ROB has P+32 entries.
// The PDF has details.
static void stage_dispatch(procsim_stats_t *stats) {
    
#ifdef DEBUG
    printf("Stage Dispatch: \n"); //  PROVIDED
#endif
    // continuous loop of pulling off dispatch queue until edge case is met (dispQ empty)
    while (dispatch_head && rob_entries < rob_length && scheduling_entries < scheduling_length)  { 
        // pull item off dispatch queue
        struct queue_item* element = dispatch_head;

#ifdef DEBUG
    printf("\tAttempting Dispatch for: ");
    print_instruction(&element->inst);
    printf("\n");
#endif

        // find src pregs & free dest register
        uint64_t dest_reg;
        for (dest_reg = 0; dest_reg < total_regs; dest_reg++) {
            // search until a free preg is found
            if (PRF[dest_reg].free) {
                break;
            }
        }
        // check if free preg is found, if not exit
        if (dest_reg >= total_regs && element->inst.dest >= 0) {
#ifdef DEBUG
    printf("\t\tCould not find free preg, aborting\n");
#endif
            stats->no_dispatch_pregs_cycles++;
            break;
        } else {
            // set source info
            if (element->inst.src1 < NUM_REGS && element->inst.src1 >= 0) {
#ifdef DEBUG
                printf("\t\tUsing preg: P%lu for src1: R%d\n", RAT[element->inst.src1], element->inst.src1);
#endif
                element->p_src1 = RAT[element->inst.src1];
            } else {
                element->p_src1 = -1;
            }
            if (element->inst.src2 < NUM_REGS && element->inst.src2 >=0)
            {
#ifdef DEBUG
                printf("\t\tUsing preg: P%lu for src2: R%d\n", RAT[element->inst.src2], element->inst.src2);
#endif
                element->p_src2 = RAT[element->inst.src2];
            } else {
                element->p_src2 = -1;
            }
            // we will dispatch instruction, remove from disp queue
            dispatch_head = dispatch_head->next;
            element->next = nullptr; 
            dispatch_entries--;
            if (dispatch_entries == 0) {
                dispatch_tail = nullptr;
            }
            if (element->inst.dest < 0) {
                // there is no destination register
                element->p_dest = -1;
                element->prev_preg = -1;
            } else if (RAT[element->inst.dest] != dest_reg) {
                // update RAT
#ifdef DEBUG
                printf("\t\tAllocating preg: P%lu for areg: R%d and updating RAT\n", dest_reg, element->inst.dest);
                printf("\t\tPreg to Free: P%lu\n", RAT[element->inst.dest]);
#endif
                element->prev_preg = RAT[element->inst.dest];
                RAT[element->inst.dest] = dest_reg;
                element->p_dest = dest_reg;
                PRF[RAT[element->inst.dest]].free = false;
                PRF[RAT[element->inst.dest]].ready = false;
            }
#ifdef DEBUG
            printf("\t\tDispatching instruction\n");
#endif
            // dispatch instruction



            // copy information into new entries to go in both the scheduling queue and rob
            queue_item* schedule_entry = (queue_item*)malloc(sizeof(queue_item));
            queue_item* rob_entry = (queue_item*)malloc(sizeof(queue_item));
            
            schedule_entry->inst = element->inst;
            schedule_entry->p_src1 = element->p_src1;
            schedule_entry->p_src2 = element->p_src2;
            schedule_entry->p_dest = element->p_dest;
            schedule_entry->prev_preg = element->prev_preg;
            schedule_entry->fired = false;
            schedule_entry->complete = false;
            schedule_entry->next = nullptr;

            rob_entry->inst = element->inst;
            rob_entry->p_src1 = element->p_src1;
            rob_entry->p_src2 = element->p_src2;
            rob_entry->p_dest = element->p_dest;
            rob_entry->prev_preg = element->prev_preg;
            rob_entry->fired = false;
            rob_entry->complete = false;
            rob_entry->next = nullptr;
        
            // free memory allocated for dispatch queue entry
            free(element);

            // place instruction into scheduling queue (at end)
            if (scheduling_tail) {
                // items already in queue
                scheduling_tail->next = schedule_entry;
                scheduling_tail = scheduling_tail->next;
            } else {
                // items not in queue
                scheduling_head = schedule_entry;
                scheduling_tail = schedule_entry;
            }
            scheduling_entries++;

            // allocate space in the ROB
            
            
            if (rob_tail)
            {
                rob_tail->next = rob_entry;
                rob_tail = rob_tail->next;
            }
            else
            {
                rob_head = rob_entry;
                rob_tail = rob_entry;
            }
            rob_entries++;
        }

    }
    //check if we ended because there are no more rob entries, may need more logic here
    if (rob_entries >= rob_length) {
        stats->rob_stall_cycles++;
    }

}

// Optional helper function which fetches instructions from the instruction
// cache using the provided procsim_driver_read_inst() function implemented
// in the driver and appends them to the dispatch queue. To simplify the
// project, the dispatch queue is infinite in size.
static void stage_fetch(procsim_stats_t *stats) {
    
#ifdef DEBUG
    printf("Stage Fetch: \n"); //  PROVIDED
#endif

    // fetch instructions
    for (uint64_t i = 0; i < fetches; i++) {
        const inst_t* inst = procsim_driver_read_inst();
        // fetched instruction, check if null or not
        if (inst) {
            stats->instructions_fetched++;
            // if not null, add to dispatch queue
#ifdef DEBUG
            printf("\tFetched Instruction: ");
            print_instruction(inst);
            printf("\n");
#endif
            // check for icache miss
            if (inst->icache_miss) {
                stats->icache_misses++;
#ifdef DEBUG
            printf("\t\tI-Cache miss repaired by driver\n");
#endif
            }
#ifdef DEBUG
            if (inst->mispredict)
                printf("\t\tBranch Misprediction will be handled by driver\n");
#endif
            // add instruction to the dispatch queue
            // find the tail of the queue, add to it
            if (dispatch_tail) {
                dispatch_tail->next = (queue_item *)malloc(sizeof(queue_item));
                dispatch_tail = dispatch_tail->next;
                dispatch_tail->inst = *inst;
                dispatch_tail->next = nullptr;
            } else {
                dispatch_head = (queue_item*)malloc(sizeof(queue_item));
                dispatch_head->inst = *inst;
                dispatch_head->next = nullptr;
                dispatch_tail = dispatch_head;
            }
            dispatch_entries++;
        } else {
            // if null do nothing/ print nop
#ifdef DEBUG
            printf("\tFetched NOP\n");
#endif
        }
    }



}

// Use this function to initialize all your data structures, simulator
// state, and statistics.
void procsim_init(const procsim_conf_t *sim_conf, procsim_stats_t *stats) {
    // assign the PRF
    total_regs = sim_conf->num_pregs + NUM_REGS;
    PRF = new reg[total_regs];
    for (uint64_t i = 0; i < total_regs; i++) {
        // set the value of the registers ( , ) if areg, opposite if preg
        if (i < NUM_REGS) {
            PRF[i].ready = true;
            PRF[i].free = false;
        } else {
            PRF[i].ready = false;
            PRF[i].free = true;
        }
    }

    // assign the RAT
    RAT = new uint64_t[NUM_REGS];
    for (uint64_t i = 0; i < NUM_REGS; i++) {
        // set the mapping of each areg to itself to begin
        RAT[i] = i;
    }

    // assign logic units
    // alu
    alus = new alu[sim_conf->num_alu_fus];
    num_alus = sim_conf->num_alu_fus;
    for (uint64_t i = 0; i < num_alus; i++) {
        alus[i].ready = true;
    }

    // mul
    muls = new mul[sim_conf->num_mul_fus];
    num_muls = sim_conf->num_mul_fus;
    for (uint64_t i = 0; i < num_muls; i++) {
        muls[i].ready = new bool[3];
        muls[i].ready[0] = true;
        muls[i].ready[1] = true;
        muls[i].ready[2] = true;
        muls[i].insts = new inst_t[3];
    }

    // lsu
    lsus = new lsu[sim_conf->num_lsu_fus];
    num_lsus = sim_conf->num_lsu_fus;
    for (uint64_t i = 0; i < num_lsus; i++) {
        lsus[i].ready = true;
    }

    // assign queues
    dispatch_head = nullptr;
    dispatch_tail = nullptr;

    fetches = sim_conf->fetch_width;

    scheduling_head = nullptr;
    scheduling_tail = nullptr;
    scheduling_length = (sim_conf->num_alu_fus + sim_conf->num_lsu_fus + sim_conf->num_mul_fus) * sim_conf->num_schedq_entries_per_fu;
    scheduling_entries = 0;

    rob_head = nullptr;
    rob_tail = nullptr;
    rob_length = sim_conf->num_rob_entries;
    rob_entries = 0;

    // allocate space for busses
    num_busses = (sim_conf->num_alu_fus + sim_conf->num_lsu_fus + sim_conf->num_mul_fus);
    items_in_bus = 0;
    result_busses = new inst_t[num_busses];

    previous_retires = 0;
    
    store_items = 0;

#ifdef DEBUG
    printf("\nScheduling queue capacity: %lu instructions\n", scheduling_length); 
    printf("Initial RAT state:\n"); //  PROVIDED
    print_rat();
    printf("\n"); //  PROVIDED
#endif
}

// To avoid confusion, we have provided this function for you. Notice that this
// calls the stage functions above in reverse order! This is intentional and
// allows you to avoid having to manage pipeline registers between stages by
// hand. This function returns the number of instructions retired, and also
// returns if a mispredict was retired by assigning true or false to
// *retired_mispredict_out, an output parameter.
uint64_t procsim_do_cycle(procsim_stats_t *stats,
                          bool *retired_mispredict_out) {
#ifdef DEBUG
    printf("================================ Begin cycle %" PRIu64 " ================================\n", stats->cycles); //  PROVIDED
#endif

    // stage_state_update() should set *retired_mispredict_out for us
    uint64_t retired_this_cycle = stage_state_update(stats, retired_mispredict_out);

    if (*retired_mispredict_out) {
#ifdef DEBUG
        printf("%" PRIu64 " instructions retired. Retired mispredict, so notifying driver to fetch correctly!\n", retired_this_cycle); //  PROVIDED
#endif

        // After we retire a misprediction, the other stages don't need to run
        stats->branch_mispredictions++;
    } else {
#ifdef DEBUG
        printf("%" PRIu64 " instructions retired. Did not retire mispredict, so proceeding with other pipeline stages.\n", retired_this_cycle); //  PROVIDED
#endif

        // If we didn't retire an interupt, then continue simulating the other
        // pipeline stages
        stage_exec(stats);
        stage_schedule(stats);
        stage_dispatch(stats);
        stage_fetch(stats);
    }

#ifdef DEBUG
    printf("End-of-cycle dispatch queue usage: %lu\n", dispatch_entries); 
    printf("End-of-cycle sched queue usage: %lu\n", scheduling_entries); 
    printf("End-of-cycle ROB usage: %lu\n", rob_entries); 
    printf("End-of-cycle RAT state:\n"); //  PROVIDED
    print_rat();
    printf("End-of-cycle Physical Register File state:\n"); //  PROVIDED
    print_prf();
    printf("End-of-cycle ROB state:\n"); //  PROVIDED
    print_rob();
    printf("================================ End cycle %" PRIu64 " ================================\n", stats->cycles); //  PROVIDED
    print_instruction(NULL); // this makes the compiler happy, ignore it
#endif

    // TODO: Increment max_usages and avg_usages in stats here!
    stats->cycles++;
    if (dispatch_entries > stats->dispq_max_size) {
        stats->dispq_max_size = dispatch_entries;
    }
    if (scheduling_entries > stats->schedq_max_size) {
        stats->schedq_max_size = scheduling_entries;
    }
    if (rob_entries > stats->rob_max_size) {
        stats->rob_max_size = rob_entries;
    }
    total_rob += rob_entries;
    total_disp += dispatch_entries;
    total_sched += scheduling_entries;
    total_retired += retired_this_cycle;

    // Return the number of instructions we retired this cycle (including the
    // interrupt we retired, if there was one!)
    return retired_this_cycle;
}

// Use this function to free any memory allocated for your simulator and to
// calculate some final statistics.
void procsim_finish(procsim_stats_t *stats) {
    // TODO: fill me in

    stats->store_buffer_hit_ratio = (1.0 * stats->store_buffer_read_hits) / (1.0 * stats->reads);
    stats->dcache_read_miss_ratio = (1.0 * stats->dcache_read_misses) / (1.0 * stats->dcache_reads);
    stats->dcache_ratio = (1.0 * stats->dcache_reads) / (1.0 * stats->reads);

    stats->dcache_read_aat = 1.0 * L1_HIT_TIME + stats->dcache_read_miss_ratio * (1.0 * L1_MISS_PENALTY);
    stats->read_aat = 1.0 * stats->store_buffer_hit_ratio + stats->dcache_ratio * stats->dcache_read_aat;

    stats->dispq_avg_size = total_disp / (1.0 * stats->cycles);
    stats->schedq_avg_size = total_sched / (1.0 * stats->cycles);
    stats->rob_avg_size = total_rob / (1.0 * stats->cycles);

    stats->ipc = total_retired / (1.0 * stats->cycles);
}

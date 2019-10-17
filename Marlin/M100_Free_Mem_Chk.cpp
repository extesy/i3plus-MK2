#define M100_FREE_MEMORY_DUMPER     
#define M100_FREE_MEMORY_CORRUPTOR  
#include "MarlinConfig.h"
#if ENABLED(M100_FREE_MEMORY_WATCHER)
#define TEST_BYTE ((char) 0xE5)
extern char command_queue[BUFSIZE][MAX_CMD_SIZE];
extern char* __brkval;
extern size_t  __heap_start, __heap_end, __flp;
extern char __bss_end;
#include "Marlin.h"
#include "gcode.h"
#include "hex_print_routines.h"
#define END_OF_HEAP() (__brkval ? __brkval : &__bss_end)
int check_for_free_memory_corruption(const char * const title);
char* top_of_stack() {
  char x;
  return &x + 1; 
}
int16_t count_test_bytes(const char * const ptr) {
  for (uint16_t i = 0; i < 32000; i++)
    if (((char) ptr[i]) != TEST_BYTE)
      return i - 1;
  return -1;
}
#if ENABLED(M100_FREE_MEMORY_DUMPER)
  void dump_free_memory(const char *ptr, const char *sp) {
    ptr = (char *)((uint16_t)ptr & 0xFFF0); 
    sp  = (char *)((uint16_t)sp  | 0x000F); 
    while (ptr < sp) {
      print_hex_word((uint16_t)ptr);      
      SERIAL_CHAR(':');
      for (uint8_t i = 0; i < 16; i++) {  
        if (i == 8) SERIAL_CHAR('-');
        print_hex_byte(ptr[i]);
        SERIAL_CHAR(' ');
      }
      safe_delay(25);
      SERIAL_CHAR('|');                   
      for (uint8_t i = 0; i < 16; i++) {
        char ccc = (char)ptr[i]; 
        if (&ptr[i] >= (const char*)command_queue && &ptr[i] < (const char*)(command_queue + sizeof(command_queue))) { 
          if (!WITHIN(ccc, ' ', 0x7E)) ccc = ' ';
        }
        else { 
          ccc = (ccc == TEST_BYTE) ? ' ' : '?';
        }
        SERIAL_CHAR(ccc);
      }
      SERIAL_EOL();
      ptr += 16;
      safe_delay(25);
      idle();
    }
  }
void M100_dump_routine(const char * const title, const char *start, const char *end) {
  SERIAL_ECHOLN(title);
  start = (char*)((uint16_t) start & 0xFFF0);
  end   = (char*)((uint16_t) end   | 0x000F);
  dump_free_memory(start, end);
}
#endif 
void free_memory_pool_report(char * const ptr, const int16_t size) {
  int16_t max_cnt = -1, block_cnt = 0;
  char *max_addr = NULL;
  for (int16_t i = 0; i < size; i++) {
    char *addr = ptr + i;
    if (*addr == TEST_BYTE) {
      const int16_t j = count_test_bytes(addr);
      if (j > 8) {
        SERIAL_ECHOPAIR("Found ", j);
        SERIAL_ECHOLNPAIR(" bytes free at ", hex_address(addr));
        if (j > max_cnt) {
          max_cnt  = j;
          max_addr = addr;
        }
        i += j;
        block_cnt++;
      }
    }
  }
  if (block_cnt > 1) {
    SERIAL_ECHOLNPGM("\nMemory Corruption detected in free memory area.");
    SERIAL_ECHOPAIR("\nLargest free block is ", max_cnt);
    SERIAL_ECHOLNPAIR(" bytes at ", hex_address(max_addr));
  }
  SERIAL_ECHOLNPAIR("check_for_free_memory_corruption() = ", check_for_free_memory_corruption("M100 F "));
}
#if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
  void corrupt_free_memory(char *ptr, const uint16_t size) {
    ptr += 8;
    const uint16_t near_top = top_of_stack() - ptr - 250, 
                   j = near_top / (size + 1);
    SERIAL_ECHOLNPGM("Corrupting free memory block.\n");
    for (uint16_t i = 1; i <= size; i++) {
      char * const addr = ptr + i * j;
      *addr = i;
      SERIAL_ECHOPAIR("\nCorrupting address: ", hex_address(addr));
    }
    SERIAL_EOL();
  }
#endif 
void init_free_memory(char *ptr, int16_t size) {
  SERIAL_ECHOLNPGM("Initializing free memory block.\n\n");
  size -= 250;    
  if (size < 0) {
    SERIAL_ECHOLNPGM("Unable to initialize.\n");
    return;
  }
  ptr += 8;       
  memset(ptr, TEST_BYTE, size);
  SERIAL_ECHO(size);
  SERIAL_ECHOLNPGM(" bytes of memory initialized.\n");
  for (int16_t i = 0; i < size; i++) {
    if (ptr[i] != TEST_BYTE) {
      SERIAL_ECHOPAIR("? address : ", hex_address(ptr + i));
      SERIAL_ECHOLNPAIR("=", hex_byte(ptr[i]));
      SERIAL_EOL();
    }
  }
}
void gcode_M100() {
  SERIAL_ECHOPAIR("\n__brkval : ", hex_address(__brkval));
  SERIAL_ECHOPAIR("\n__bss_end : ", hex_address(&__bss_end));
  char *ptr = END_OF_HEAP(), *sp = top_of_stack();
  SERIAL_ECHOPAIR("\nstart of free space : ", hex_address(ptr));
  SERIAL_ECHOLNPAIR("\nStack Pointer : ", hex_address(sp));
  static bool m100_not_initialized = true;
  if (m100_not_initialized || parser.seen('I')) {
    m100_not_initialized = false;
    init_free_memory(ptr, sp - ptr);
  }
  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    if (parser.seen('D'))
      return dump_free_memory(ptr, sp);
  #endif
  if (parser.seen('F'))
    return free_memory_pool_report(ptr, sp - ptr);
  #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
    if (parser.seen('C'))
      return corrupt_free_memory(ptr, parser.value_int());
  #endif
}
int check_for_free_memory_corruption(const char * const title) {
  SERIAL_ECHO(title);
  char *ptr = END_OF_HEAP(), *sp = top_of_stack();
  int n = sp - ptr;
  SERIAL_ECHOPAIR("\nfmc() n=", n);
  SERIAL_ECHOPAIR("\n&__brkval: ", hex_address(&__brkval));
  SERIAL_ECHOPAIR("=",             hex_address(__brkval));
  SERIAL_ECHOPAIR("\n__bss_end: ", hex_address(&__bss_end));
  SERIAL_ECHOPAIR(" sp=",          hex_address(sp));
  if (sp < ptr)  {
    SERIAL_ECHOPGM(" sp < Heap ");
    safe_delay(20);
    #ifdef M100_FREE_MEMORY_DUMPER
      M100_dump_routine("   Memory corruption detected with sp<Heap\n", (char*)0x1B80, (char*)0x21FF);
    #endif
  }
  int block_cnt = 0;
  for (int i = 0; i < n; i++) {
    if (ptr[i] == TEST_BYTE) {
      int16_t j = count_test_bytes(ptr + i);
      if (j > 8) {
        i += j;
        block_cnt++;
        SERIAL_ECHOPAIR(" (", block_cnt);
        SERIAL_ECHOPAIR(") found=", j);
        SERIAL_ECHOPGM("   ");
      }
    }
  }
  SERIAL_ECHOPAIR("  block_found=", block_cnt);
  if (block_cnt != 1 || __brkval != 0x0000)
    SERIAL_ECHOLNPGM("\nMemory Corruption detected in free memory area.");
  if (block_cnt == 0)       
    block_cnt = -1;         
  SERIAL_ECHOPGM(" return=");
  if (block_cnt == 1) {
    SERIAL_CHAR('0');       
    SERIAL_EOL();             
    return 0;
  }
  SERIAL_ECHOLNPGM("true");
  return block_cnt;
}
#endif 

// esp32 Forth, based on Version 6.3

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#ifdef esp32
#  include "freertos/FreeRTOS.h"
#  include "freertos/task.h"
#  include "esp_system.h"
#  include "driver/gpio.h"
#  include "driver/ledc.h"
#  include "esp_spi_flash.h"
#  include "esp_err.h"
#  include "esp_vfs_dev.h"
#  include "driver/uart.h"
#else
#  include <termios.h>
#  include <unistd.h>
#endif

#ifdef esp32
esp_err_t example_configure_stdin_stdout(void)
{
    // Initialize VFS & UART so we can use std::cout/cin
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install( (uart_port_t)CONFIG_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    return ESP_OK;
}
#else
static struct termios terminalOld;
static void RestoreTerminal(void) {
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalOld);
}
static void SetupTerminal(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  tcgetattr(STDIN_FILENO, &terminalOld);
  atexit(RestoreTerminal);
  struct termios t = terminalOld;
  t.c_lflag &= ~ECHO;
  t.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &t);
}
#endif

#define DEBUG_COREWORDS 0

typedef intptr_t cell_t;
typedef uintptr_t ucell_t;
#if __SIZEOF_POINTER__ == 8
typedef __int128_t dcell_t;
typedef __uint128_t udcell_t;
# define UPPER_MASK 0x5f5f5f5f5f5f5f5f
#elif __SIZEOF_POINTER__ == 4
typedef int64_t dcell_t;
typedef uint64_t udcell_t;
# define UPPER_MASK 0x5f5f5f5f
#else
# error "unsupported cell size"
#endif

#define CELL_BITS (sizeof(cell_t)*8)
#define PRIxCELL PRIxPTR
#define CELL_MASK (sizeof(cell_t)-1)

# define  FALSE 0
# define  TRUE  -1
# define  LOGICAL ? TRUE : FALSE
# define  LOWER(x,y) ((ucell_t)(x)<(ucell_t)(y))
# define  pop top = stack[(unsigned char)S--]
# define  push stack[(unsigned char)++S] = top; top =
# define  popR rack[(unsigned char)R--]
# define  pushR rack[(unsigned char)++R]

cell_t rack[256] = {0};
cell_t stack[256] = {0};
unsigned char R=0, S=0;
cell_t* Pointer ;
cell_t  P, IP, WP, top, links, len ;
uint8_t* cData ;
dcell_t d, n, m ;

#define PRIMITIVE_LIST \
  X("NOP", NOP, next()) \
  X("ACCEPT", ACCEPT, len = duplexread(cData, top); top = len) \
  X("?KEY", QKEY, push fgetc(stdin); push -1) \
  X("EMIT", EMIT, char c=top; fputc(c, stdout); pop) \
  X("DOCON", DOCON, push data[WP/sizeof(cell_t)]) \
  X("DOLIT", DOLIT, push data[IP/sizeof(cell_t)]; \
    IP += sizeof(cell_t); next()) \
  X("DOLIST", DOLIST, rack[(unsigned char)++R] = IP; IP = WP; next()) \
  X("EXIT", EXIT, IP = (cell_t) rack[(unsigned char)R--]; next()) \
  X("EXECUTE", EXECUTE, P = top; WP = P + sizeof(cell_t); pop) \
  X("DONEXT", DONEXT, if(rack[(unsigned char)R]) \
    (rack[(unsigned char)R] -= 1, IP = data[IP/sizeof(cell_t)]); \
    else (IP += sizeof(cell_t), R--); next()) \
  X("QBRANCH", QBRANCH, if(top == 0) IP = data[IP/sizeof(cell_t)]; \
    else IP += sizeof(cell_t); pop; next()) \
  X("BRANCH", BRANCH, IP = data[IP/sizeof(cell_t)]; next()) \
  X("!", STORE, data[top/sizeof(cell_t)] = stack[(unsigned char)S--]; pop) \
  X("@", AT, top = data[top/sizeof(cell_t)]) \
  X("C!", CSTORE, cData[top] = (unsigned char) stack[(unsigned char)S--]; pop) \
  X("C@", CAT, top = (cell_t) cData[top]) \
  X("RPAT", RPAT, ) \
  X("RPSTO", RPSTO, ) \
  X("R>", RFROM, push rack[(unsigned char)R--]) \
  X("R@", RAT, push rack[(unsigned char)R]) \
  X(">R", TOR, rack[(unsigned char)++R] = top; pop) \
  X("SPAT", SPAT, ) \
  X("SPSTO", SPSTO, ) \
  X("DROP", DROP, pop) \
  X("DUP", DUP, stack[(unsigned char)++S] = top) \
  X("SWAP", SWAP, WP = top; top = stack[(unsigned char)S]; \
    stack[(unsigned char)S] = WP) \
  X("OVER", OVER, push stack[(unsigned char)(S-1)] ) \
  X("0<", ZLESS, top = (top < 0) LOGICAL) \
  X("AND", AND, top &= stack[(unsigned char)S--]) \
  X("OR", OR, top |= stack[(unsigned char)S--]) \
  X("XOR", XOR, top ^= stack[(unsigned char)S--]) \
  X("U+", UPLUS, stack[(unsigned char)S] += top; \
    top = LOWER(stack[(unsigned char)S], top)) \
  X("NEXT", NEXTT, next()) \
  X("?DUP", QDUP, if(top) stack[(unsigned char)++S] = top) \
  X("ROT", ROT, WP = stack[(unsigned char)(S-1)]; \
    stack[(unsigned char)(S-1)] = stack[(unsigned char)S]; \
    stack[(unsigned char)S] = top; top = WP) \
  X("2DROP", DDROP, fun_DROP(); fun_DROP()) \
  X("2DUP", DDUP, fun_OVER(); fun_OVER()) \
  X("+", PLUS, top += stack[(unsigned char)S--]) \
  X("INVERSE", INVERSE, top = -top-1) \
  X("NEGATE", NEGATE, top = 0 - top) \
  X("DNEGATE", DNEGATE, fun_INVERSE(); \
    fun_TOR(); fun_INVERSE(); push 1; fun_UPLUS(); \
    fun_RFROM(); fun_PLUS()) \
  X("-", SUB, top = stack[(unsigned char)S--] - top) \
  X("ABS", ABS, if(top < 0) top = -top) \
  X("=", EQUAL, top = (stack[(unsigned char)S--] == top) LOGICAL) \
  X("U<", ULESS, top = LOWER(stack[(unsigned char)S], top) LOGICAL; S--) \
  X("<", LESS, top = (stack[(unsigned char)S--] < top) LOGICAL) \
  X("UM/MOD", UMMOD, d = (udcell_t)((ucell_t)top); \
    m = (udcell_t)((ucell_t)stack[(unsigned char) S]); \
    n = (udcell_t)((ucell_t)stack[(unsigned char) (S - 1)]); \
    n += m << CELL_BITS; \
    pop; \
    if (d == 0) (top = 0, stack[S] = 0); \
    else (top = (ucell_t)(n / d), \
    stack[(unsigned char) S] = (ucell_t)(n%d))) \
  X("M/MOD", MSMOD, d = (dcell_t)((cell_t)top); \
    m = (dcell_t)((cell_t)stack[(unsigned char) S]); \
    n = (dcell_t)((cell_t)stack[(unsigned char) S - 1]); \
    n += m << CELL_BITS; \
    pop; \
    if (d == 0) (top = 0, stack[S] = 0); \
    else (top = (cell_t)(n / d), \
    stack[(unsigned char) S] = (cell_t)(n%d))) \
  X("/MOD", SLMOD, if (top != 0) \
    (WP = stack[(unsigned char) S] / top, \
    stack[(unsigned char) S] %= top, \
    top = WP)) \
  X("MOD", MOD, top = (top) ? stack[(unsigned char) S--] % top \
    : stack[(unsigned char) S--]) \
  X("/", SLASH, top = (top) ? stack[(unsigned char) S--] / top : (S--, 0)) \
  X("UM*", UMSTA, d = (udcell_t)top; \
    m = (udcell_t)stack[(unsigned char) S]; \
    m *= d; \
    top = (ucell_t)(m >> CELL_BITS); \
    stack[(unsigned char) S] = (ucell_t)m) \
  X("*", STAR, top *= stack[(unsigned char) S--]) \
  X("M*", MSTAR, d = (dcell_t)top; \
    m = (dcell_t)stack[(unsigned char) S]; \
    m *= d; \
    top = (cell_t)(m >> CELL_BITS); \
    stack[(unsigned char) S] = (cell_t)m) \
  X("*/MOD", SSMOD, d = (dcell_t)top; \
    m = (dcell_t)stack[(unsigned char) S]; \
    n = (dcell_t)stack[(unsigned char) (S - 1)]; \
    n *= m; \
    pop; \
    top = (cell_t)(n / d); \
    stack[(unsigned char) S] = (cell_t)(n%d)) \
  X("*/", STASL, d = (dcell_t)top; \
    m = (dcell_t)stack[(unsigned char) S]; \
    n = (dcell_t)stack[(unsigned char) (S - 1)]; \
    n *= m; \
    pop; pop; \
    top = (cell_t)(n / d)) \
  X("PICK", PICK, top = stack[(unsigned char)(S-top)]) \
  X("+!", PSTORE, data[top/sizeof(cell_t)] += stack[(unsigned char)S--]; pop) \
  X("2!", DSTORE, data[(top/sizeof(cell_t))+1] = stack[(unsigned char)S--]; \
    data[top/sizeof(cell_t)] = stack[(unsigned char)S--]; pop) \
  X("2@", DAT, push data[top/sizeof(cell_t)]; \
    top = data[(top/sizeof(cell_t))+1]) \
  X("COUNT", COUNT, stack[(unsigned char)++S] = top + 1; top = cData[top]) \
  X("DOVAR", DOVAR, push WP) \
  X("MAX", MAX, if (top < stack[(unsigned char)S]) pop; else S--) \
  X("MIN", MIN, if (top < stack[(unsigned char)S]) S--; else pop) \
  X("TONE", TONE, WP=top; pop; /* ledcWriteTone(WP,top); */ pop) \
  X("sendPacket", sendPacket, ) \
  X("POKE", POKE, Pointer = (cell_t*)top; \
    *Pointer = stack[(unsigned char)S--]; pop) \
  X("PEEK", PEEK, Pointer = (cell_t*)top; top = *Pointer) \
  X("ADC", ADC, /* top= (cell_t) analogRead(top); */ top = (cell_t) 0) \
  X("PIN", PIN, WP = top; pop; setpin(WP, top); pop) \
  X("DUTY", DUTY, WP = top; pop; /* ledcAnalogWrite(WP,top,255); */ pop) \
  X("FREQ", FREQ, WP = top; pop; /* ledcSetup(WP,top,13); */ pop) \
  X("MS", MS, WP = top; pop; mspause(WP)) \
  X("TERMINATE", TERMINATE, exit(top))

static int ABORQP=0, DOTQP=0, STRQP = 0;
#define X(sname, name, code) static int name = 0;
  PRIMITIVE_LIST
#undef X

cell_t data[16000] = {};
int IMEDD=0x80;
int COMPO=0x40;

static void HEADER_WITH_FLAGS(int flags, const char *name) {
  P=IP/sizeof(cell_t);
  int i;
  int len = strlen(name) & 0x1f;
  data[P++] = links;
  IP=P*sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, links);
  for (i=links/sizeof(cell_t);i<P;i++) {
    printf(" ");
    printf("%" PRIxCELL, data[i]);
  }
#endif
  links=IP;
  cData[IP++]=len | flags;
  for (i=0;i<len;i++) {
    cData[IP++] = name[i];
  }
  while (IP&CELL_MASK) {
    cData[IP++]=0;
  }
#if DEBUG_COREWORDS
  printf("\n");
  printf("%s", name);
  printf(" ");
  printf("%" PRIxCELL, IP);
#endif
}

static void HEADER(const char *name) {
  HEADER_WITH_FLAGS(0, name);
}

static void HEADER_IMMEDIATE(const char *name) {
  HEADER_WITH_FLAGS(IMEDD, name);
}

static int WithPadding(int sz) {
  return (sz + CELL_MASK) & ~CELL_MASK;
}

static void Comma(cell_t n) {
  assert(IP % sizeof(cell_t) == 0);
  P=IP/sizeof(cell_t);
  data[P++] = n;
  IP=P*sizeof(cell_t);
}

static int LABEL(int len, ... ) {
  int addr=IP;
  P=IP/sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
#endif
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
}

#define STR_LIKE(op) \
  str = va_arg(argList, const char*); \
  int len = strlen(str); \
  data[P++] = op; \
  IP=P*sizeof(cell_t); \
  cData[IP++]=len; \
  for (i=0;i<len;i++) cData[IP++]=str[i]; \
  while (IP & CELL_MASK) cData[IP++]=0; \
  P=IP/sizeof(cell_t);

#define MACRO_LIST \
  X(BEGIN, pushR=P) \
  X(AGAIN, data[P++]=BRANCH; data[P++]=popR*sizeof(cell_t)) \
  X(UNTIL, data[P++]=QBRANCH; data[P++]=popR*sizeof(cell_t)) \
  X(WHILE, data[P++]=QBRANCH; data[P++]=0; k=popR; pushR=(P-1); pushR=k) \
  X(REPEAT, data[P++]=BRANCH; data[P++]=popR*sizeof(cell_t); \
    data[popR]=P*sizeof(cell_t)) \
  X(IF, data[P++]=QBRANCH; pushR=P; data[P++]=0) \
  X(ELSE, data[P++]=BRANCH; data[P++]=0; \
    data[popR]=P*sizeof(cell_t); pushR=P-1) \
  X(THEN, data[popR]=P*sizeof(cell_t)) \
  X(FOR, data[P++]=TOR; pushR=P) \
  X(NEXT, data[P++]=DONEXT; data[P++]=popR*sizeof(cell_t)) \
  X(AFT, data[P++]=BRANCH; data[P++]=0; k=popR; pushR=P; pushR=P-1) \
  X(DOTQ, STR_LIKE(DOTQP)) \
  X(STRQ, STR_LIKE(STRQP)) \
  X(ABORTQ, STR_LIKE(ABORQP))

enum {
  UNUSED_MACRO=0x80000000,
#define X(name, code) name,
MACRO_LIST
#undef X
};

static void CheckSum() {
  int i;
  unsigned char sum=0;
  printf("\n");
  printf("%04" PRIxCELL " ",IP);
  for (i=0;i<32;i++) {
    sum += cData[IP];
    printf("%02x",cData[IP++]);
  }
  printf(" %02x",sum);
}

/******************************************************************************/
/* PRIMITIVES                                                                 */
/******************************************************************************/

static void next(void) {
  P = data[IP/sizeof(cell_t)];
  IP += sizeof(cell_t);
  WP = P+sizeof(cell_t);
}

static int duplexread(unsigned char* dst, int sz) {
  int len = 0;
  while (sz > 0) {
    int ch = fgetc(stdin);
    if (ch == 127 || ch == 8) {
      if (len > 0) {
        fputc(8, stdout);
        fputc(' ', stdout);
        fputc(8, stdout);
        --len;
      }
      continue;
    }
    if (ch == '\n') {
      fputc('\r', stdout);
    }
    fputc(ch, stdout);
    if (ch == '\n' || ch < 0) {
      break;
    }
    dst[len++] = ch;
  }
  return len;
}

static void setpin(int p, int level) {
#ifdef esp32
   gpio_pad_select_gpio(p);
   gpio_set_direction(p, GPIO_MODE_OUTPUT);
   gpio_set_level(p, top);
#endif
}

static void mspause(cell_t ms) {
#ifdef esp32
  vTaskDelay(ms / portTICK_PERIOD_MS);
#endif
}

#define X(sname, name, code) static void fun_ ## name(void) { code; }
  PRIMITIVE_LIST
#undef X

static void (*primitives[])(void) = {
#define X(sname, name, code) fun_ ## name,
  PRIMITIVE_LIST
#undef X
};

enum {
  as_UNKNOWN = -1,
#define X(sname, name, code) as_ ## name,
  PRIMITIVE_LIST
#undef X
};

int CODE(const char *name, ... ) {
  HEADER(name);
  int addr=IP;
  cell_t s;
  va_list argList;
  va_start(argList, name);
  do {
    s = va_arg(argList, cell_t);
    cData[IP++] = s;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%" PRIxCELL, s);
#endif
  } while(s != as_NEXTT);
  while (IP & CELL_MASK) {
    cData[IP++]=0;
  }
  va_end(argList);
  return addr;
}

int COLON_WITH_FLAGS(int flags, const char *name, ...) {
  HEADER_WITH_FLAGS(flags, name);
  int addr=IP;
  P=IP/sizeof(cell_t);
  data[P++] = as_DOLIST;
  va_list argList;
  va_start(argList, name);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
  printf(" ");
  printf("6");
#endif
  int last=0, word=0, i, k;
  const char *str;
  do {
    last = word;
    word = va_arg(argList, int);
    switch (word) {
#define X(name, code) case name: { code; } break;
      MACRO_LIST
#undef X
      default:
        data[P++] = word;
#if DEBUG_COREWORDS
        printf(" ");
        printf("%" PRIxCELL, data[P-1]);
#endif
       break;
    }
  } while (word != EXIT || R > 0 || last == DOLIT);
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
}

#define COLON(...) COLON_WITH_FLAGS(0, __VA_ARGS__)
#define COLON_IMMEDIATE(...) COLON_WITH_FLAGS(IMEDD, __VA_ARGS__)

static int CONSTANT(const char *name, cell_t n) {
  int ret = CODE(name, as_DOCON, as_NEXTT);
  Comma(n);
  return ret;
}

void evaluate()
{
  for(;;) {
    unsigned char bytecode = cData[P++];
#if 0
    printf("[%d]%d: [%d] %d %d %d %d %d | [%d] %d %d %d %d %d\n",
      (int)P, bytecode,
      (int)S,
      (int)stack[0], (int)stack[1], (int)stack[2], (int)stack[3], (int)stack[4],
      (int)R,
      (int)rack[0], (int)rack[1], (int)rack[2], (int)rack[3], (int)rack[4]);
#endif
    if (bytecode) primitives[bytecode]();
    else break;
  }
}

static void run() {
  printf("AIBOT\n");
  // TODO: Find better way to start in decimal.
  strcpy((char*) cData, "decimal");
  len = strlen((char*) cData);
  for (;;) {
    data[0x66] = 0;                   // >IN
    data[0x67] = len;                 // #TIB
    data[0x68] = 0;                   // 'TIB
    P = 0x60 * sizeof(cell_t);        // EVAL
    WP = P + sizeof(cell_t);
    evaluate();
    len = duplexread(cData, 255);
  }
}

#ifdef esp32
void app_main(void) {
#else
int main(void) {
#endif
#ifdef esp32
  example_configure_stdin_stdout();
#else
  SetupTerminal();
#endif
  P = 0x60 * sizeof(cell_t);
  WP = P + sizeof(cell_t);
  IP = 0;
  S = 0;
  R = 0;
  top = 0;
  cData = (uint8_t *) data;

  IP=128 * sizeof(cell_t);
  cell_t datap = 0x64 * sizeof(cell_t) - sizeof(cell_t);
  R=0;
  int HLD=CONSTANT("HLD", datap += sizeof(cell_t));
  int SPAN=CONSTANT("SPAN", datap += sizeof(cell_t));
  int INN=CONSTANT(">IN", datap += sizeof(cell_t));
  int NTIB=CONSTANT("#TIB", datap += sizeof(cell_t));
  int TTIB=CONSTANT("'TIB", datap += sizeof(cell_t));
  int BASE=CONSTANT("BASE", datap += sizeof(cell_t));
  int CNTXT=CONSTANT("CONTEXT", datap += sizeof(cell_t));
  int CP=CONSTANT("CP", datap += sizeof(cell_t));
  int LAST=CONSTANT("LAST", datap += sizeof(cell_t));
  int TEVAL=CONSTANT("'EVAL", datap += sizeof(cell_t));
  int TABRT=CONSTANT("'ABORT", datap += sizeof(cell_t));
  int TEMP=CONSTANT("tmp", datap += sizeof(cell_t));
  int Z=CONSTANT("Z", 0);
  int PPQN=CONSTANT("ppqn", datap += sizeof(cell_t));
  int CHANN=CONSTANT("channel", datap += sizeof(cell_t));

#define X(sname, name, code) name = CODE(sname, as_ ## name, as_NEXTT);
  PRIMITIVE_LIST
#undef X

  int BLANK=CONSTANT("BL", 32);
  int CELL=CONSTANT("CELL", sizeof(cell_t));
  int CELLP=CODE("CELL+", as_DOCON, as_PLUS, as_NEXTT); Comma(sizeof(cell_t));
  int CELLM=CODE("CELL-", as_DOCON, as_SUB, as_NEXTT); Comma(sizeof(cell_t));
  int CELLS=CODE("CELLS", as_DOCON, as_STAR, as_NEXTT); Comma(sizeof(cell_t));
  int CELLD=CODE("CELL/", as_DOCON, as_SLASH, as_NEXTT); Comma(sizeof(cell_t));
  int ONEP=CODE("1+", as_DOCON, as_PLUS, as_NEXTT); Comma(1);
  int ONEM=CODE("1-", as_DOCON, as_SUB, as_NEXTT); Comma(1);
  int TWOP=CODE("2+", as_DOCON, as_PLUS, as_NEXTT); Comma(2);
  int TWOM=CODE("2-", as_DOCON, as_SUB, as_NEXTT); Comma(2);
  int TWOST=CODE("2*", as_DOCON, as_STAR, as_NEXTT); Comma(2);
  int TWOS=CODE("2/", as_DOCON, as_SLASH, as_NEXTT); Comma(2);

  int BYE=COLON("BYE", DOLIT,0,TERMINATE,EXIT);
  int KEY=COLON("KEY", BEGIN,QKEY,UNTIL,EXIT);
  int WITHI=COLON("WITHIN", OVER,SUB,TOR,SUB,RFROM,ULESS,EXIT);
  int TCHAR=COLON(">CHAR", DOLIT,0x7F,AND,DUP,DOLIT,127,BLANK,WITHI,
                  IF, DROP,DOLIT,'_', THEN, EXIT);
  int ALIGN=COLON("ALIGNED", DOLIT,CELL_MASK,PLUS, DOLIT,~CELL_MASK,AND,EXIT);
  int HERE=COLON("HERE", CP,AT,EXIT);
  int PAD=COLON("PAD", HERE,DOLIT,80,PLUS,EXIT);
  int TIB=COLON("TIB", TTIB,AT,EXIT);
  int ATEXE=COLON("@EXECUTE", AT,QDUP,IF,EXECUTE,THEN,EXIT);
  int CMOVEE=COLON("CMOVE", FOR,AFT,OVER,CAT,OVER,CSTORE,TOR,ONEP,RFROM,ONEP,
                            THEN,NEXT,DDROP,EXIT);
  int MOVE=COLON("MOVE", CELLD,FOR,AFT,OVER,AT,OVER,STORE,TOR,CELLP,RFROM,CELLP,
                         THEN,NEXT,DDROP,EXIT);
  int FILL=COLON("FILL", SWAP,FOR,SWAP,AFT,DDUP,CSTORE,ONEP,THEN,NEXT,
                         DDROP,EXIT);
  int DIGIT=COLON("DIGIT", DOLIT,9,OVER,LESS,DOLIT,7,AND,PLUS,
                           DOLIT,'0',PLUS,EXIT);
  int EXTRC=COLON("EXTRACT", DOLIT,0,SWAP,UMMOD,SWAP,DIGIT,EXIT);
  int BDIGS=COLON("<#", PAD,HLD,STORE,EXIT);
  int HOLD=COLON("HOLD", HLD,AT,ONEM,DUP,HLD,STORE,CSTORE,EXIT);
  int DIG=COLON("#", BASE,AT,EXTRC,HOLD,EXIT);
  int DIGS=COLON("#S", BEGIN,DIG,DUP,WHILE,REPEAT,EXIT);
  int SIGN=COLON("SIGN", ZLESS,IF,DOLIT,'-',HOLD,THEN,EXIT);
  int EDIGS=COLON("#>", DROP,HLD,AT,PAD,OVER,SUB,EXIT);
  int STRR=COLON("str", DUP,TOR,ABS,BDIGS,DIGS,RFROM,SIGN,EDIGS,EXIT);
  int HEXX=COLON("HEX", DOLIT,16,BASE,STORE,EXIT);
  int DECIM=COLON("DECIMAL", DOLIT,10,BASE,STORE,EXIT);
  int UPPER=COLON("wupper", DOLIT,UPPER_MASK,AND,EXIT);
  int TOUPP=COLON(">upper", DUP,DOLIT,'a',DOLIT,'{',WITHI,IF,
                            DOLIT,0x5F,AND,THEN,EXIT);
  int DIGTQ=COLON("DIGIT?", TOR,TOUPP,DOLIT,'0',SUB,DOLIT,9,OVER,LESS,
                            IF,DOLIT,7,SUB,DUP,DOLIT,10,LESS,OR,THEN,
                            DUP,RFROM,ULESS,EXIT);
  int NUMBQ=COLON("NUMBER?", BASE,AT,TOR,DOLIT,0,OVER,COUNT,OVER,CAT,
                             DOLIT,'$',EQUAL,IF,HEXX,SWAP,ONEP,SWAP,ONEM,THEN,
                             OVER,CAT,DOLIT,'-',EQUAL,TOR,SWAP,RAT,SUB,
                             SWAP,RAT,PLUS,QDUP,IF,ONEM,
                             FOR,DUP,TOR,CAT,BASE,AT,DIGTQ,
                             WHILE,SWAP,BASE,AT,STAR,PLUS,RFROM,ONEP,NEXT,
                             DROP,RAT,IF,NEGATE,THEN,SWAP,
                             ELSE,RFROM,RFROM,DDROP,DDROP,DOLIT,0,THEN,
                             DUP,THEN,RFROM,DDROP,RFROM,BASE,STORE,EXIT);
  int SPACE=COLON("SPACE", BLANK,EMIT,EXIT);
  int CHARS=COLON("CHARS", SWAP,DOLIT,0,MAX,FOR,AFT,DUP,EMIT,THEN,NEXT,
                           DROP,EXIT);
  int SPACS=COLON("SPACES", BLANK,CHARS,EXIT);
  int TYPES=COLON("TYPE",
    FOR,AFT,DUP,CAT,TCHAR,EMIT,ONEP,THEN,NEXT,DROP,EXIT);
  int CR=COLON("CR", DOLIT,'\n',DOLIT,'\r',EMIT,EMIT,EXIT);
  int DOSTR=COLON("do$", RFROM,RAT,RFROM,COUNT,PLUS,ALIGN,TOR,SWAP,TOR,EXIT);
  STRQP=COLON("$\"|", DOSTR,EXIT);
  DOTQP=COLON(".\"|", DOSTR,COUNT,TYPES,EXIT);
  int DOTR=COLON(".R", TOR,STRR,RFROM,OVER,SUB,SPACS,TYPES,EXIT);
  int UDOTR=COLON("U.R",
    TOR,BDIGS,DIGS,EDIGS,RFROM,OVER,SUB,SPACS,TYPES,EXIT);
  int UDOT=COLON("U.", BDIGS,DIGS,EDIGS,SPACE,TYPES,EXIT);
  int DOT=COLON(".", BASE,AT,DOLIT,10,XOR,IF,UDOT,EXIT,THEN,
                     STRR,SPACE,TYPES,EXIT);
  int QUEST=COLON("?", AT,DOT,EXIT);
  int PARS=COLON("(parse)", TEMP,CSTORE,OVER,TOR,DUP,IF,
                              ONEM,TEMP,CAT,BLANK,EQUAL,IF,
                                FOR,BLANK,OVER,CAT,SUB,ZLESS,INVERSE,
                                  WHILE,ONEP,
                                NEXT,
                                RFROM,DROP,DOLIT,0,DUP,EXIT,
                              THEN,RFROM,
                            THEN,OVER,SWAP,
                            FOR,TEMP,CAT,OVER,CAT,SUB,TEMP,CAT,BLANK,EQUAL,
                              IF,ZLESS,THEN,
                              WHILE,ONEP,
                            NEXT,DUP,TOR,
                            ELSE,RFROM,DROP,DUP,ONEP,TOR,
                            THEN,OVER,SUB,RFROM,RFROM,SUB,EXIT,
                            THEN,OVER,RFROM,SUB,EXIT);
  int PACKS=COLON("PACK$", DUP,TOR,DDUP,PLUS,DOLIT,
    ~CELL_MASK,AND,DOLIT,0,SWAP,STORE,DDUP,CSTORE,ONEP,SWAP,CMOVEE,RFROM,EXIT);
  int PARSE=COLON("PARSE", TOR,TIB,INN,AT,PLUS,NTIB,AT,INN,AT,SUB,RFROM,
    PARS,INN,PSTORE,EXIT);
  int TOKEN=COLON("TOKEN", BLANK,PARSE,DOLIT,0x1F,MIN,HERE,CELLP,PACKS,EXIT);
  int WORDD=COLON("WORD", PARSE,HERE,CELLP,PACKS,EXIT);
  int NAMET=COLON("NAME>", COUNT,DOLIT,0x1F,AND,PLUS,ALIGN,EXIT);
  int SAMEQ=COLON("SAME?", DOLIT,0x1F,AND,CELLD,FOR,AFT,
                             OVER,RAT,CELLS,PLUS,AT,UPPER,OVER,RAT,
                             CELLS,PLUS,AT,UPPER,SUB,QDUP,IF,
                               RFROM,DROP,EXIT,THEN,
                           THEN,NEXT,DOLIT,0,EXIT);
  int FIND=COLON("find", SWAP,DUP,AT,TEMP,STORE,DUP,AT,TOR,CELLP,SWAP,
                         BEGIN,AT,DUP,IF,
                           DUP,AT,DOLIT,~0xC0,AND,UPPER,RAT,UPPER,XOR,
                           IF,CELLP,DOLIT,-1,ELSE,CELLP,TEMP,AT,SAMEQ,THEN,
                         ELSE,RFROM,DROP,SWAP,CELLM,SWAP,EXIT,THEN,
                         WHILE,CELLM,CELLM,REPEAT,
                         RFROM,DROP,SWAP,DROP,CELLM,DUP,NAMET,SWAP,EXIT);
  int NAMEQ=COLON("NAME?", CNTXT,FIND,EXIT);
  int EXPEC=COLON("EXPECT", ACCEPT,SPAN,STORE,DROP,EXIT);
  int QUERY=COLON("QUERY", TIB,DOLIT,0x100,ACCEPT,NTIB,STORE,DROP,
    DOLIT,0,INN,STORE,EXIT);
  int ABORT=COLON("ABORT", NOP,TABRT,ATEXE,EXIT);
  ABORQP=COLON("abort\"", IF,DOSTR,COUNT,TYPES,ABORT,THEN,
    DOSTR,DROP,EXIT);
  int ERRORR=COLON("ERROR", SPACE,COUNT,TYPES,DOLIT,'?',EMIT,CR,ABORT,EXIT);
  int INTER=COLON("$INTERPRET", NAMEQ,QDUP,IF,CAT,DOLIT,COMPO,AND,
                  ABORTQ," compile only",EXECUTE,EXIT,THEN,NUMBQ,IF,
                  EXIT,THEN,ERRORR,EXIT);
  int LBRAC=COLON_IMMEDIATE("[", DOLIT,INTER,TEVAL,STORE,EXIT);
  int DOTOK=COLON(".OK", CR,DOLIT,INTER,TEVAL,AT,EQUAL,IF,
                    TOR,TOR,TOR,DUP,DOT,RFROM,DUP,DOT,RFROM,DUP,DOT,
                    RFROM,DUP,DOT,DOTQ," ok>",THEN,EXIT);
  int EVAL=COLON("EVAL", LBRAC,BEGIN,TOKEN,DUP,AT,WHILE,TEVAL,ATEXE,
                 REPEAT,DROP,DOTOK,NOP,EXIT);
  int QUITT=COLON("QUIT", LBRAC,BEGIN,QUERY,EVAL,AGAIN,EXIT);
  int LOAD=COLON("LOAD", NTIB,STORE,TTIB,STORE,DOLIT,0,INN,STORE,EVAL,EXIT);
  int COMMA=COLON(",", HERE,DUP,CELLP,CP,STORE,STORE,EXIT);
  int LITER=COLON_IMMEDIATE("LITERAL", DOLIT,DOLIT,COMMA,COMMA,EXIT);
  int ALLOT=COLON("ALLOT", ALIGN,CP,PSTORE,EXIT);
  int STRCQ=COLON("$,\"", DOLIT,'"',WORDD,COUNT,PLUS,ALIGN,CP,STORE,EXIT);
  int UNIQU=COLON("?UNIQUE", DUP,NAMEQ,QDUP,IF,
                   COUNT,DOLIT,0x1F,AND,SPACE,TYPES,DOTQ," reDef",
                  THEN,DROP,EXIT);
  int SNAME=COLON("$,n", DUP,AT,IF,UNIQU,DUP,NAMET,CP,STORE,DUP,
                  LAST,STORE,CELLM,CNTXT,AT,SWAP,STORE,EXIT,THEN,
                  ERRORR,EXIT);
  int TICK=COLON("'", TOKEN,NAMEQ,IF,EXIT,THEN,ERRORR,EXIT);
  int BCOMP=COLON_IMMEDIATE("[COMPILE]", TICK,COMMA,EXIT);
  int COMPI=COLON("COMPILE", RFROM,DUP,AT,COMMA,CELLP,TOR,EXIT);
  int SCOMP=COLON("$COMPILE", NAMEQ,QDUP,IF,AT,DOLIT,IMEDD,AND,IF,EXECUTE,
                  ELSE,COMMA,THEN,EXIT,THEN,NUMBQ,IF,LITER,EXIT,THEN,
                  ERRORR,EXIT);
  int OVERT=COLON("OVERT", LAST,AT,CNTXT,STORE,EXIT);
  int RBRAC=COLON("]", DOLIT,SCOMP,TEVAL,STORE,EXIT);
  int COLN=COLON(":", TOKEN,SNAME,RBRAC,DOLIT,as_DOLIST,COMMA,EXIT);
  int SEMIS=COLON_IMMEDIATE(";", DOLIT,EXIT,COMMA,LBRAC,OVERT,EXIT);
  int DMP=COLON("dm+", OVER,DOLIT,6,UDOTR,FOR,AFT,DUP,AT,DOLIT,9,UDOTR,CELLP,
    THEN,NEXT,EXIT);
  int DUMP=COLON("DUMP", BASE,AT,TOR,HEXX,DOLIT,0x1F,PLUS,DOLIT,0x20,SLASH,
    FOR,AFT,CR,DOLIT,8,DDUP,DMP,TOR,SPACE,CELLS,TYPES,RFROM,THEN,NEXT,
    DROP,RFROM,BASE,STORE,EXIT);
  int TNAME=COLON(">NAME", CNTXT,BEGIN,AT,DUP,WHILE,DDUP,NAMET,XOR,
    IF,ONEM,ELSE,SWAP,DROP,EXIT,THEN,REPEAT,SWAP,DROP,EXIT);
  int DOTID=COLON(".ID", COUNT,DOLIT,0x1F,AND,TYPES,SPACE,EXIT);
  int WORDS=COLON("WORDS", CR,CNTXT,DOLIT,0,TEMP,STORE,BEGIN,AT,QDUP,
    WHILE,DUP,SPACE,DOTID,CELLM,TEMP,AT,DOLIT,8,LESS,
    IF,DOLIT,1,TEMP,PSTORE,ELSE,CR,DOLIT,0,TEMP,STORE,THEN,
    REPEAT,EXIT);
  int FORGT=COLON("FORGET", TOKEN,NAMEQ,QDUP,IF,
    CELLM,DUP,CP,STORE,AT,DUP,CNTXT,STORE,LAST,STORE,DROP,EXIT,THEN,
    ERRORR,EXIT);
  int COLD=COLON("COLD",CR,DOTQ,"esp32forth V6.3, 2019 ",CR,EXIT);
  int LINE=COLON("LINE",
    DOLIT,0x7,FOR,DUP,PEEK,DOLIT,0x9,UDOTR,CELLP,NEXT,EXIT);
  int PP=COLON("PP",
    FOR,AFT,CR,DUP,DOLIT,0x9,UDOTR,SPACE,LINE,THEN,NEXT,EXIT);
  int P0=COLON("P0", DOLIT,0x3FF44004,POKE,EXIT);
  int P0S=COLON("P0S", DOLIT,0x3FF44008,POKE,EXIT);
  int P0C=COLON("P0C", DOLIT,0x3FF4400C,POKE,EXIT);
  int P1=COLON("P1", DOLIT,0x3FF44010,POKE,EXIT);
  int P1S=COLON("P1S", DOLIT,0x3FF44014,POKE,EXIT);
  int P1C=COLON("P1C", DOLIT,0x3FF44018,POKE,EXIT);
  int P0EN=COLON("P0EN", DOLIT,0x3FF44020,POKE,EXIT);
  int P0ENS=COLON("P0ENS", DOLIT,0x3FF44024,POKE,EXIT);
  int P0ENC=COLON("P0ENC", DOLIT,0x3FF44028,POKE,EXIT);
  int P1EN=COLON("P1EN", DOLIT,0x3FF4402C,POKE,EXIT);
  int P1ENS=COLON("P1ENS", DOLIT,0x3FF44030,POKE,EXIT);
  int P1ENC=COLON("P1ENC", DOLIT,0x3FF44034,POKE,EXIT);
  int P0IN=COLON("P0IN", DOLIT,0x3FF4403C,PEEK,DOT,EXIT);
  int P1IN=COLON("P1IN", DOLIT,0x3FF44040,PEEK,DOT,EXIT);
  int PPP=COLON("PPP", DOLIT,0x3FF44000,DOLIT,3,PP,DROP,EXIT);
  int EMITT=COLON("EMITT", DOLIT,0x3,FOR,DOLIT,0,DOLIT,0x100,MSMOD,SWAP,
    TCHAR,EMIT,NEXT,DROP,EXIT);
  int TYPEE=COLON("TYPEE", SPACE,DOLIT,0x7,FOR,DUP,PEEK,EMITT,CELLP,NEXT,
    DROP,EXIT);
  int PPPP=COLON("PPPP", FOR,AFT,CR,DUP,DUP,DOLIT,0x9,UDOTR,SPACE,LINE,
    SWAP,TYPEE,THEN,NEXT,EXIT);
  int KKK=COLON("KKK", DOLIT,0x3FF59000,DOLIT,0x10,PP,DROP,EXIT);
  int THENN=COLON_IMMEDIATE("THEN", HERE,SWAP,STORE,EXIT);
  int FOR=COLON_IMMEDIATE("FOR", COMPI,TOR,HERE,EXIT);
  int BEGIN=COLON_IMMEDIATE("BEGIN", HERE,EXIT);
  int NEXT=COLON_IMMEDIATE("NEXT", COMPI,DONEXT,COMMA,EXIT);
  int UNTIL=COLON_IMMEDIATE("UNTIL", COMPI,QBRANCH,COMMA,EXIT);
  int AGAIN=COLON_IMMEDIATE("AGAIN", COMPI,BRANCH,COMMA,EXIT);
  int IFF=COLON_IMMEDIATE("IF", COMPI,QBRANCH,HERE,DOLIT,0,COMMA,EXIT);
  int AHEAD=COLON_IMMEDIATE("AHEAD", COMPI,BRANCH,HERE,DOLIT,0,COMMA,EXIT);
  int REPEA=COLON_IMMEDIATE("REPEAT", AGAIN,THENN,EXIT);
  int AFT=COLON_IMMEDIATE("AFT", DROP,AHEAD,HERE,SWAP,EXIT);
  int ELSEE=COLON_IMMEDIATE("ELSE", AHEAD,SWAP,THENN,EXIT);
  int WHILEE=COLON_IMMEDIATE("WHILE", IFF,SWAP,EXIT);
  int ABRTQ=COLON_IMMEDIATE("ABORT\"", DOLIT,ABORQP,HERE,STORE,STRCQ,EXIT);
  int STRQ=COLON_IMMEDIATE("$\"", DOLIT,STRQP,HERE,STORE,STRCQ,EXIT);
  int DOTQQ=COLON_IMMEDIATE(".\"", DOLIT,DOTQP,HERE,STORE,STRCQ,EXIT);
  int CODE=COLON("CODE", TOKEN,SNAME,OVERT,HERE,ALIGN,CP,STORE,EXIT);
  int CREAT=COLON("CREATE", CODE,DOLIT,as_DOVAR + (as_NEXTT << 8),COMMA,EXIT);
  int VARIA=COLON("VARIABLE", CREAT,DOLIT,0,COMMA,EXIT);
  int CONST=COLON("CONSTANT", CODE,DOLIT,as_DOCON + (as_NEXTT << 8),
    COMMA,COMMA,EXIT);
  int DOTPR=COLON_IMMEDIATE(".(", DOLIT,')',PARSE,TYPES,EXIT);
  int BKSLA=COLON_IMMEDIATE("\\", DOLIT,'\n',WORDD,DROP,EXIT);
  int PAREN=COLON_IMMEDIATE("(", DOLIT,')',PARSE,DDROP,EXIT);
  int ONLY=COLON_IMMEDIATE("COMPILE-ONLY", DOLIT,COMPO,LAST,AT,PSTORE,EXIT);
  int IMMED=COLON("IMMEDIATE", DOLIT,IMEDD,LAST,AT,PSTORE,EXIT);
  int ENDD=IP;
#if DEBUG_COREWORDS
  printf("\n");
  printf("IP=");
  printf("%" PRIxCELL, IP);
  printf(" R-stack= ");
  printf("%" PRIxCELL, popR*sizeof(cell_t));
#endif
  IP = 0x60 * sizeof(cell_t);
  int USER=LABEL(18,
                 as_DOLIST,EVAL,0,0,
                 0,  // HLD
                 0,  // SPAN
                 0,  // >IN
                 0,  // #TIB
                 0,  // 'TIB
                 0x10,  // BASE
                 IMMED-WithPadding(10), // CONTEXT
                 ENDD,  // CP
                 IMMED-WithPadding(10), // LAST
                 INTER,  // 'EVAL
                 EVAL,  // 'ABORT
                 0,  // tmp
                 0,  // ppqn
                 0);  // channel

#if DEBUG_COREWORDS
  // dump dictionary
  IP=0;
  for (len=0;len<0x48 * sizeof(cell_t);len++) {
    CheckSum();
  }
  printf("\n");
#endif

  setpin(13, 0);
  run();
}


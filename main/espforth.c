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
unsigned char R, S, bytecode ;
cell_t* Pointer ;
cell_t  P, IP, WP, top, links, len ;
uint8_t* cData ;
dcell_t d, n, m ;

int BRAN=0,QBRAN=0,DONXT=0,DOTQP=0,STRQP=0,TOR=0,ABORQP=0;

//#include "rom_54.h" /* load dictionary */
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

int CODE(int len, ... ) {
  int total = WithPadding(len);
  int padding = total - len;
  int addr=IP;
  int s;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    s= va_arg(argList, int);
    cData[IP++]=s;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", s);
#endif
  }
  for (; padding;padding--) {
    cData[IP++]=0;
  }
  va_end(argList);
  return addr;
  }
static void Comma(cell_t n) {
  assert(IP % sizeof(cell_t) == 0);
  P=IP/sizeof(cell_t);
  data[P++] = n;
  IP=P*sizeof(cell_t);
}
int COLON(int len, ... ) {
  int addr=IP;
  P=IP/sizeof(cell_t);
  data[P++]=6; // dolist
  va_list argList;
  va_start(argList, len);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
  printf(" ");
  printf("6");
#endif
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%" PRIxCELL, data[P-1]);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
  }
int LABEL(int len, ... ) {
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
void BEGIN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" BEGIN ");
#endif
  pushR=P;
  va_list argList;
  va_start(argList, len);
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
}
void AGAIN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" AGAIN ");
#endif
  data[P++]=BRAN;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
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
  }
void UNTIL(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" UNTIL ");
#endif
  data[P++]=QBRAN;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
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
  }
void WHILE(int len, ... ) {
  P=IP/sizeof(cell_t);
  int k;
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" WHILE ");
#endif
  data[P++]=QBRAN;
  data[P++]=0;
  k=popR;
  pushR=(P-1);
  pushR=k;
  va_list argList;
  va_start(argList, len);
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
  }
void REPEAT(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" REPEAT ");
#endif
  data[P++]=BRAN;
  data[P++]=popR*sizeof(cell_t);
  data[popR]=P*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
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
  }
void IF(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" IF ");
#endif
  data[P++]=QBRAN;
  pushR=P;
  data[P++]=0;
  va_list argList;
  va_start(argList, len);
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
  }
void ELSE(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ELSE ");
#endif
  data[P++]=BRAN;
  data[P++]=0;
  data[popR]=P*sizeof(cell_t);
  pushR=P-1;
  va_list argList;
  va_start(argList, len);
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
  }
void THEN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" THEN ");
#endif
  data[popR]=P*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
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
  }
void FOR(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" FOR ");
#endif
  data[P++]=TOR;
  pushR=P;
  va_list argList;
  va_start(argList, len);
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
  }
void NEXT(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" NEXT ");
#endif
  data[P++]=DONXT;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
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
  }
void AFT(int len, ... ) {
  P=IP/sizeof(cell_t);
  int k;
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" AFT ");
#endif
  data[P++]=BRAN;
  data[P++]=0;
  k=popR;
  (void)k;
  pushR=P;
  pushR=P-1;
  va_list argList;
  va_start(argList, len);
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
  }
void DOTQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=DOTQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}
void STRQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=STRQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}
void ABORQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=ABORQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}

void CheckSum() {
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
/* ledc                                                                       */
/******************************************************************************/
/* LEDC Software Fade */
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            5
int brightness = 255;    // how bright the LED is

/******************************************************************************/
/* PRIMITIVES                                                                 */
/******************************************************************************/

void next(void)
{ P = data[IP/sizeof(cell_t)];
  IP += sizeof(cell_t);
  WP = P+sizeof(cell_t);  }

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

void accep()
/* WiFiClient */
{
  len = duplexread(cData, top);
  top = len;
}
void qrx(void)
  { push fgetc(stdin);
    push -1; }

void txsto(void)
{  char c=top;
   fputc(c, stdout);
   pop;
}

void docon(void)
{  push data[WP/sizeof(cell_t)]; }

void dolit(void)
{   push data[IP/sizeof(cell_t)];
  IP += sizeof(cell_t);
  next(); }

void dolist(void)
{   rack[(unsigned char)++R] = IP;
  IP = WP;
  next(); }

void exitt(void)
{   IP = (cell_t) rack[(unsigned char)R--];
  next(); }

void execu(void)
{  P = top;
  WP = P + sizeof(cell_t);
  pop; }

void donext(void)
{   if(rack[(unsigned char)R]) {
    rack[(unsigned char)R] -= 1 ;
    IP = data[IP/sizeof(cell_t)];
  } else { IP += sizeof(cell_t);  R-- ;  }
  next(); }

void qbran(void)
{   if(top == 0) IP = data[IP/sizeof(cell_t)];
  else IP += sizeof(cell_t);  pop;
  next(); }

void bran(void)
{   IP = data[IP/sizeof(cell_t)];
  next(); }

void store(void)
{   data[top/sizeof(cell_t)] = stack[(unsigned char)S--];
  pop;  }

void at(void)
{   top = data[top/sizeof(cell_t)];  }

void cstor(void)
{   cData[top] = (unsigned char) stack[(unsigned char)S--];
  pop;  }

void cat(void)
{   top = (cell_t) cData[top];  }

void rpat(void) {}
void rpsto(void) {}

void rfrom(void)
{   push rack[(unsigned char)R--];  }

void rat(void)
{   push rack[(unsigned char)R];  }

void tor(void)
{   rack[(unsigned char)++R] = top;  pop;  }

void spat(void) {}
void spsto(void) {}

void drop(void)
{   pop;  }

void dup_(void)
{   stack[(unsigned char)++S] = top;  }

void swap(void)
{   WP = top;
  top = stack[(unsigned char)S];
  stack[(unsigned char)S] = WP;  }

void over(void)
{  push stack[(unsigned char)(S-1)];  }

void zless(void)
{   top = (top < 0) LOGICAL;  }

void andd(void)
{   top &= stack[(unsigned char)S--];  }

void orr(void)
{   top |= stack[(unsigned char)S--];  }

void xorr(void)
{   top ^= stack[(unsigned char)S--];  }

void uplus(void)
{   stack[(unsigned char)S] += top;
  top = LOWER(stack[(unsigned char)S], top);  }

void nop(void)
{   next(); }

void qdup(void)
{   if(top) stack[(unsigned char)++S] = top ;  }

void rot(void)
{   WP = stack[(unsigned char)(S-1)];
  stack[(unsigned char)(S-1)] = stack[(unsigned char)S];
  stack[(unsigned char)S] = top;
  top = WP;  }

void ddrop(void)
{   drop(); drop();  }

void ddup(void)
{   over(); over();  }

void plus(void)
{   top += stack[(unsigned char)S--];  }

void inver(void)
{   top = -top-1;  }

void negat(void)
{   top = 0 - top;  }

void dnega(void)
{   inver();
  tor();
  inver();
  push 1;
  uplus();
  rfrom();
  plus(); }

void subb(void)
{   top = stack[(unsigned char)S--] - top;  }

void abss(void)
{   if(top < 0)
    top = -top;  }

void great(void)
{   top = (stack[(unsigned char)S--] > top) LOGICAL;  }

void less(void)
{   top = (stack[(unsigned char)S--] < top) LOGICAL;  }

void equal(void)
{   top = (stack[(unsigned char)S--] == top) LOGICAL;  }

void uless(void)
{   top = LOWER(stack[(unsigned char)S], top) LOGICAL; S--;  }

void ummod(void)
{  d = (udcell_t)((ucell_t)top);
  m = (udcell_t)((ucell_t)stack[(unsigned char) S]);
  n = (udcell_t)((ucell_t)stack[(unsigned char) (S - 1)]);
  n += m << CELL_BITS;
  pop;
  if (d == 0) {
    top = 0;
    stack[S] = 0;
    return;
  }
  top = (ucell_t)(n / d);
  stack[(unsigned char) S] = (ucell_t)(n%d); }
void msmod(void)
{ d = (dcell_t)((cell_t)top);
  m = (dcell_t)((cell_t)stack[(unsigned char) S]);
  n = (dcell_t)((cell_t)stack[(unsigned char) S - 1]);
  n += m << CELL_BITS;
  pop;
  if (d == 0) {
    top = 0;
    stack[S] = 0;
    return;
  }
  top = (cell_t)(n / d);
  stack[(unsigned char) S] = (cell_t)(n%d); }
void slmod(void)
{ if (top != 0) {
    WP = stack[(unsigned char) S] / top;
    stack[(unsigned char) S] %= top;
    top = WP;
  } }
void mod(void)
{ top = (top) ? stack[(unsigned char) S--] % top : stack[(unsigned char) S--]; }
void slash(void)
{ top = (top) ? stack[(unsigned char) S--] / top : (S--, 0); }
void umsta(void)
{ d = (udcell_t)top;
  m = (udcell_t)stack[(unsigned char) S];
  m *= d;
  top = (ucell_t)(m >> CELL_BITS);
  stack[(unsigned char) S] = (ucell_t)m; }
void star(void)
{ top *= stack[(unsigned char) S--]; }
void mstar(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  m *= d;
  top = (cell_t)(m >> CELL_BITS);
  stack[(unsigned char) S] = (cell_t)m; }
void ssmod(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  n = (dcell_t)stack[(unsigned char) (S - 1)];
  n *= m;
  pop;
  top = (cell_t)(n / d);
  stack[(unsigned char) S] = (cell_t)(n%d); }
void stasl(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  n = (dcell_t)stack[(unsigned char) (S - 1)];
  n *= m;
  pop; pop;
  top = (cell_t)(n / d); }

void pick(void)
{   top = stack[(unsigned char)(S-top)];  }

void pstor(void)
{   data[top/sizeof(cell_t)] += stack[(unsigned char)S--], pop;  }

void dstor(void)
{   data[(top/sizeof(cell_t))+1] = stack[(unsigned char)S--];
  data[top/sizeof(cell_t)] = stack[(unsigned char)S--];
  pop;  }

void dat(void)
{   push data[top/sizeof(cell_t)];
  top = data[(top/sizeof(cell_t))+1];  }

void count(void)
{   stack[(unsigned char)++S] = top + 1;
  top = cData[top]; }

void dovar(void)
{   push WP; }

void maxx(void)
{   if (top < stack[(unsigned char)S]) pop;
  else S--; }

void minn(void)
{   if (top < stack[(unsigned char)S]) S--;
  else pop; }

void audio(void)
{  WP=top; pop;
   // ledcWriteTone(WP,top);
   pop;
}

void sendPacket(void)
{}

void poke(void)
{   Pointer = (cell_t*)top; *Pointer = stack[(unsigned char)S--];
    pop;  }

void peeek(void)
{   Pointer = (cell_t*)top; top = *Pointer;  }

void adc(void) {
  //top= (cell_t) analogRead(top);
  top= (cell_t) 0;
}

static void setpin(int p, int level) {
#ifdef esp32
   gpio_pad_select_gpio(p);
   gpio_set_direction(p, GPIO_MODE_OUTPUT);
   gpio_set_level(p, top);
#endif
}

void pin(void)
{  WP=top; pop;
   //ledcAttachPin(top,WP);
   setpin(WP, top);
   pop;
}

void ms(void) {
  WP = top; pop;
#ifdef esp32
  vTaskDelay(WP / portTICK_PERIOD_MS);
#endif
}

void duty(void)
{  WP=top; pop;
   //ledcAnalogWrite(WP,top,255);
   pop;
}

void freq(void)
{  WP=top; pop;
   //ledcSetup(WP,top,13);
   pop;
}

void (*primitives[73])(void) = {
    /* case 0 */ nop,
    /* case 1 */ accep,
    /* case 2 */ qrx,
    /* case 3 */ txsto,
    /* case 4 */ docon,
    /* case 5 */ dolit,
    /* case 6 */ dolist,
    /* case 7 */ exitt,
    /* case 8 */ execu,
    /* case 9 */ donext,
    /* case 10 */ qbran,
    /* case 11 */ bran,
    /* case 12 */ store,
    /* case 13 */ at,
    /* case 14 */ cstor,
    /* case 15 */ cat,
    /* case 16 */ nop,
    /* case 17 */ nop,
    /* case 18 */ rfrom,
    /* case 19 */ rat,
    /* case 20 */ tor,
    /* case 21 */ nop,
    /* case 22 */ nop,
    /* case 23 */ drop,
    /* case 24 */ dup_,
    /* case 25 */ swap,
    /* case 26 */ over,
    /* case 27 */ zless,
    /* case 28 */ andd,
    /* case 29 */ orr,
    /* case 30 */ xorr,
    /* case 31 */ uplus,
    /* case 32 */ next,
    /* case 33 */ qdup,
    /* case 34 */ rot,
    /* case 35 */ ddrop,
    /* case 36 */ ddup,
    /* case 37 */ plus,
    /* case 38 */ inver,
    /* case 39 */ negat,
    /* case 40 */ dnega,
    /* case 41 */ subb,
    /* case 42 */ abss,
    /* case 43 */ equal,
    /* case 44 */ uless,
    /* case 45 */ less,
    /* case 46 */ ummod,
    /* case 47 */ msmod,
    /* case 48 */ slmod,
    /* case 49 */ mod,
    /* case 50 */ slash,
    /* case 51 */ umsta,
    /* case 52 */ star,
    /* case 53 */ mstar,
    /* case 54 */ ssmod,
    /* case 55 */ stasl,
    /* case 56 */ pick,
    /* case 57 */ pstor,
    /* case 58 */ dstor,
    /* case 59 */ dat,
    /* case 60 */ count,
    /* case 61 */ dovar,
    /* case 62 */ maxx,
    /* case 63 */ minn,
    /* case 64 */ audio,
    /* case 65 */ sendPacket,
    /* case 66 */ poke,
    /* case 67 */ peeek,
    /* case 68 */ adc,
    /* case 69 */ pin,
    /* case 70 */ duty,
    /* case 71 */ freq,
    /* case 72 */ ms };

int as_nop=0;
int as_accept=1;
int as_qrx=2;
int as_txsto=3;
int as_docon=4;
int as_dolit=5;
int as_dolist=6;
int as_exit=7;
int as_execu=8;
int as_donext=9;
int as_qbran=10;
int as_bran=11;
int as_store=12;
int as_at=13;
int as_cstor=14;
int as_cat=15;
int as_rpat=16;
int as_rpsto=17;
int as_rfrom=18;
int as_rat=19;
int as_tor=20;
int as_spat=21;
int as_spsto=22;
int as_drop=23;
int as_dup=24;
int as_swap=25;
int as_over=26;
int as_zless=27;
int as_andd=28;
int as_orr=29;
int as_xorr=30;
int as_uplus=31;
int as_next=32;
int as_qdup=33;
int as_rot=34;
int as_ddrop=35;
int as_ddup=36;
int as_plus=37;
int as_inver=38;
int as_negat=39;
int as_dnega=40;
int as_subb=41;
int as_abss=42;
int as_equal=43;
int as_uless=44;
int as_less=45;
int as_ummod=46;
int as_msmod=47;
int as_slmod=48;
int as_mod=49;
int as_slash=50;
int as_umsta=51;
int as_star=52;
int as_mstar=53;
int as_ssmod=54;
int as_stasl=55;
int as_pick=56;
int as_pstor=57;
int as_dstor=58;
int as_dat=59;
int as_count=60;
int as_dovar=61;
int as_max=62;
int as_min=63;
int as_tone=64;
int as_sendPacket=65;
int as_poke=66;
int as_peek=67;
int as_adc=68;
int as_pin=69;
int as_duty=70;
int as_freq=71;
int as_ms=72;

static int CONSTANT(cell_t n) {
  int ret = CODE(2, as_docon, as_next);
  Comma(n);
  return ret;
}

void evaluate()
{
  for(;;) {
    bytecode=(unsigned char)cData[P++];
    //printf("%d ", bytecode);
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
#if 0
  printf("booting...\n");
#endif
  P = 0x60 * sizeof(cell_t);
  WP = P + sizeof(cell_t);
  IP = 0;
  S = 0;
  R = 0;
  top = 0;
  cData = (uint8_t *) data;

#if 0
// Setup timer and attach timer to a led pin
  ledcSetup(0, 100, LEDC_TIMER_13_BIT);
  ledcAttachPin(5, 0);
  ledcAnalogWrite(0, 250, brightness);
  pinMode(2,OUTPUT);
  digitalWrite(2, HIGH);   // turn the LED2 on
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);   // motor1 forward
  pinMode(17,OUTPUT);
  digitalWrite(17, LOW);   // motor1 backward
  pinMode(18,OUTPUT);
  digitalWrite(18, LOW);   // motor2 forward
  pinMode(19,OUTPUT);
  digitalWrite(19, LOW);   // motor2 bacward
#endif

  IP=128 * sizeof(cell_t);
  cell_t datap = 0x64 * sizeof(cell_t) - sizeof(cell_t);
  R=0;
  HEADER("HLD");
  int HLD=CONSTANT(datap += sizeof(cell_t));
  HEADER("SPAN");
  int SPAN=CONSTANT(datap += sizeof(cell_t));
  HEADER(">IN");
  int INN=CONSTANT(datap += sizeof(cell_t));
  HEADER("#TIB");
  int NTIB=CONSTANT(datap += sizeof(cell_t));
  HEADER("'TIB");
  int TTIB=CONSTANT(datap += sizeof(cell_t));
  HEADER("BASE");
  int BASE=CONSTANT(datap += sizeof(cell_t));
  HEADER("CONTEXT");
  int CNTXT=CONSTANT(datap += sizeof(cell_t));
  HEADER("CP");
  int CP=CONSTANT(datap += sizeof(cell_t));
  HEADER("LAST");
  int LAST=CONSTANT(datap += sizeof(cell_t));
  HEADER("'EVAL");
  int TEVAL=CONSTANT(datap += sizeof(cell_t));
  HEADER("'ABORT");
  int TABRT=CONSTANT(datap += sizeof(cell_t));
  HEADER("tmp");
  int TEMP=CONSTANT(datap += sizeof(cell_t));
  HEADER("Z");
  int Z=CONSTANT(0);
  HEADER("ppqn");
  int PPQN=CONSTANT(datap += sizeof(cell_t));
  HEADER("channel");
  int CHANN=CONSTANT(datap += sizeof(cell_t));

  HEADER("NOP");
  int NOP=CODE(4,as_nop,as_next,0,0);
  HEADER("ACCEPT");
  int ACCEP=CODE(4,as_accept,as_next,0,0);
  HEADER("?KEY");
  int QKEY=CODE(4,as_qrx,as_next,0,0);
  HEADER("EMIT");
  int EMIT=CODE(4,as_txsto,as_next,0,0);
  HEADER("DOLIT");
  int DOLIT=CODE(4,as_dolit,as_next,0,0);
  HEADER("DOLIST");
  int DOLST=CODE(4,as_dolist,as_next,0,0);
  HEADER("EXIT");
  int EXITT=CODE(4,as_exit,as_next,0,0);
  HEADER("EXECUTE");
  int EXECU=CODE(4,as_execu,as_next,0,0);
  HEADER("DONEXT");
  DONXT=CODE(4,as_donext,as_next,0,0);
  HEADER("QBRANCH");
  QBRAN=CODE(4,as_qbran,as_next,0,0);
  HEADER("BRANCH");
  BRAN=CODE(4,as_bran,as_next,0,0);
  HEADER("!");
  int STORE=CODE(4,as_store,as_next,0,0);
  HEADER("@");
  int AT=CODE(4,as_at,as_next,0,0);
  HEADER("C!");
  int CSTOR=CODE(4,as_cstor,as_next,0,0);
  HEADER("C@");
  int CAT=CODE(4,as_cat,as_next,0,0);
  HEADER("R>");
  int RFROM=CODE(4,as_rfrom,as_next,0,0);
  HEADER("R@");
  int RAT=CODE(4,as_rat,as_next,0,0);
  HEADER(">R");
  TOR=CODE(4,as_tor,as_next,0,0);
  HEADER("DROP");
  int DROP=CODE(4,as_drop,as_next,0,0);
  HEADER("DUP");
  int DUPP=CODE(4,as_dup,as_next,0,0);
  HEADER("SWAP");
  int SWAP=CODE(4,as_swap,as_next,0,0);
  HEADER("OVER");
  int OVER=CODE(4,as_over,as_next,0,0);
  HEADER("0<");
  int ZLESS=CODE(4,as_zless,as_next,0,0);
  HEADER("AND");
  int ANDD=CODE(4,as_andd,as_next,0,0);
  HEADER("OR");
  int ORR=CODE(4,as_orr,as_next,0,0);
  HEADER("XOR");
  int XORR=CODE(4,as_xorr,as_next,0,0);
  HEADER("UM+");
  int UPLUS=CODE(4,as_uplus,as_next,0,0);
  HEADER("?DUP");
  int QDUP=CODE(4,as_qdup,as_next,0,0);
  HEADER("ROT");
  int ROT=CODE(4,as_rot,as_next,0,0);
  HEADER("2DROP");
  int DDROP=CODE(4,as_ddrop,as_next,0,0);
  HEADER("2DUP");
  int DDUP=CODE(4,as_ddup,as_next,0,0);
  HEADER("+");
  int PLUS=CODE(4,as_plus,as_next,0,0);
  HEADER("NOT");
  int INVER=CODE(4,as_inver,as_next,0,0);
  HEADER("NEGATE");
  int NEGAT=CODE(4,as_negat,as_next,0,0);
  HEADER("DNEGATE");
  int DNEGA=CODE(4,as_dnega,as_next,0,0);
  HEADER("-");
  int SUBBB=CODE(4,as_subb,as_next,0,0);
  HEADER("ABS");
  int ABSS=CODE(4,as_abss,as_next,0,0);
  HEADER("=");
  int EQUAL=CODE(4,as_equal,as_next,0,0);
  HEADER("U<");
  int ULESS=CODE(4,as_uless,as_next,0,0);
  HEADER("<");
  int LESS=CODE(4,as_less,as_next,0,0);
  HEADER("UM/MOD");
  int UMMOD=CODE(4,as_ummod,as_next,0,0);
  HEADER("M/MOD");
  int MSMOD=CODE(4,as_msmod,as_next,0,0);
  HEADER("/MOD");
  int SLMOD=CODE(4,as_slmod,as_next,0,0);
  HEADER("MOD");
  int MODD=CODE(4,as_mod,as_next,0,0);
  HEADER("/");
  int SLASH=CODE(4,as_slash,as_next,0,0);
  HEADER("UM*");
  int UMSTA=CODE(4,as_umsta,as_next,0,0);
  HEADER("*");
  int STAR=CODE(4,as_star,as_next,0,0);
  HEADER("M*");
  int MSTAR=CODE(4,as_mstar,as_next,0,0);
  HEADER("*/MOD");
  int SSMOD=CODE(4,as_ssmod,as_next,0,0);
  HEADER("*/");
  int STASL=CODE(4,as_stasl,as_next,0,0);
  HEADER("PICK");
  int PICK=CODE(4,as_pick,as_next,0,0);
  HEADER("+!");
  int PSTOR=CODE(4,as_pstor,as_next,0,0);
  HEADER("2!");
  int DSTOR=CODE(4,as_dstor,as_next,0,0);
  HEADER("2@");
  int DAT=CODE(4,as_dat,as_next,0,0);
  HEADER("COUNT");
  int COUNT=CODE(4,as_count,as_next,0,0);
  HEADER("MAX");
  int MAX=CODE(4,as_max,as_next,0,0);
  HEADER("MIN");
  int MIN=CODE(4,as_min,as_next,0,0);
  HEADER("BL");
  int BLANK=CONSTANT(32);
  HEADER("CELL");
  int CELL=CONSTANT(sizeof(cell_t));
  HEADER("CELL+");
  int CELLP=CODE(3,as_docon,as_plus,as_next); Comma(sizeof(cell_t));
  HEADER("CELL-");
  int CELLM=CODE(3,as_docon,as_subb,as_next); Comma(sizeof(cell_t));
  HEADER("CELLS");
  int CELLS=CODE(3,as_docon,as_star,as_next); Comma(sizeof(cell_t));
  HEADER("CELL/");
  int CELLD=CODE(3,as_docon,as_slash,as_next); Comma(sizeof(cell_t));
  HEADER("1+");
  int ONEP=CODE(3,as_docon,as_plus,as_next); Comma(1);
  HEADER("1-");
  int ONEM=CODE(3,as_docon,as_subb,as_next); Comma(1);
  HEADER("2+");
  int TWOP=CODE(3,as_docon,as_plus,as_next); Comma(2);
  HEADER("2-");
  int TWOM=CODE(3,as_docon,as_subb,as_next); Comma(2);
  HEADER("2*");
  int TWOST=CODE(3,as_docon,as_star,as_next); Comma(2);
  HEADER("2/");
  int TWOS=CODE(3,as_docon,as_slash,as_next); Comma(2);
  HEADER("sendPacket");
  int SENDP=CODE(4,as_sendPacket,as_next,0,0);
  HEADER("POKE");
  int POKE=CODE(4,as_poke,as_next,0,0);
  HEADER("PEEK");
  int PEEK=CODE(4,as_peek,as_next,0,0);
  HEADER("ADC");
  int ADC=CODE(4,as_adc,as_next,0,0);
  HEADER("PIN");
  int PIN=CODE(4,as_pin,as_next,0,0);
  HEADER("TONE");
  int TONE=CODE(4,as_tone,as_next,0,0);
  HEADER("DUTY");
  int DUTY=CODE(4,as_duty,as_next,0,0);
  HEADER("FREQ");
  int FREQ=CODE(4,as_freq,as_next,0,0);
  HEADER("MS");
  int MS=CODE(4,as_ms,as_next,0,0);

  HEADER("KEY");
  int KEY=COLON(0);
  BEGIN(1,QKEY);
  UNTIL(1,EXITT);
  HEADER("WITHIN");
  int WITHI=COLON(7,OVER,SUBBB,TOR,SUBBB,RFROM,ULESS,EXITT);
  HEADER(">CHAR");
  int TCHAR=COLON(8,DOLIT,0x7F,ANDD,DUPP,DOLIT,127,BLANK,WITHI);
  IF(3,DROP,DOLIT,'_');
  THEN(1,EXITT);
  HEADER("ALIGNED");
  int ALIGN=COLON(7,DOLIT,CELL_MASK,PLUS,
                  DOLIT,~CELL_MASK,ANDD,EXITT);
  HEADER("HERE");
  int HERE=COLON(3,CP,AT,EXITT);
  HEADER("PAD");
  int PAD=COLON(5,HERE,DOLIT,80,PLUS,EXITT);
  HEADER("TIB");
  int TIB=COLON(3,TTIB,AT,EXITT);
  HEADER("@EXECUTE");
  int ATEXE=COLON(2,AT,QDUP);
  IF(1,EXECU);
  THEN(1,EXITT);
  HEADER("CMOVE");
  int CMOVEE=COLON(0);
  FOR(0);
  AFT(8,OVER,CAT,OVER,CSTOR,TOR,ONEP,RFROM,ONEP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  HEADER("MOVE");
  int MOVE=COLON(1,CELLD);
  FOR(0);
  AFT(8,OVER,AT,OVER,STORE,TOR,CELLP,RFROM,CELLP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  HEADER("FILL");
  int FILL=COLON(1,SWAP);
  FOR(1,SWAP);
  AFT(3,DDUP,CSTOR,ONEP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  HEADER("DIGIT");
  int DIGIT=COLON(12,DOLIT,9,OVER,LESS,DOLIT,7,ANDD,PLUS,DOLIT,'0',PLUS,EXITT);
  HEADER("EXTRACT");
  int EXTRC=COLON(7,DOLIT,0,SWAP,UMMOD,SWAP,DIGIT,EXITT);
  HEADER("<#");
  int BDIGS=COLON(4,PAD,HLD,STORE,EXITT);
  HEADER("HOLD");
  int HOLD=COLON(8,HLD,AT,ONEM,DUPP,HLD,STORE,CSTOR,EXITT);
  HEADER("#");
  int DIG=COLON(5,BASE,AT,EXTRC,HOLD,EXITT);
  HEADER("#S");
  int DIGS=COLON(0);
  BEGIN(2,DIG,DUPP);
  WHILE(0);
  REPEAT(1,EXITT);
  HEADER("SIGN");
  int SIGN=COLON(1,ZLESS);
  IF(3,DOLIT,'-',HOLD);
  THEN(1,EXITT);
  HEADER("#>");
  int EDIGS=COLON(7,DROP,HLD,AT,PAD,OVER,SUBBB,EXITT);
  HEADER("str");
  int STRR=COLON(9,DUPP,TOR,ABSS,BDIGS,DIGS,RFROM,SIGN,EDIGS,EXITT);
  HEADER("HEX");
  int HEXX=COLON(5,DOLIT,16,BASE,STORE,EXITT);
  HEADER("DECIMAL");
  int DECIM=COLON(5,DOLIT,10,BASE,STORE,EXITT);
  HEADER("wupper");
  int UPPER=COLON(1,DOLIT); Comma(UPPER_MASK); Comma(ANDD); Comma(EXITT);
  HEADER(">upper");
  int TOUPP=COLON(6,DUPP,DOLIT,'a',DOLIT,'{',WITHI);
  IF(3,DOLIT,0x5F,ANDD);
  THEN(1,EXITT);
  HEADER("DIGIT?");
  int DIGTQ=COLON(9,TOR,TOUPP,DOLIT,'0',SUBBB,DOLIT,9,OVER,LESS);
  IF(8,DOLIT,7,SUBBB,DUPP,DOLIT,10,LESS,ORR);
  THEN(4,DUPP,RFROM,ULESS,EXITT);
  HEADER("NUMBER?");
  int NUMBQ=COLON(12,BASE,AT,TOR,DOLIT,0,OVER,COUNT,OVER,CAT,DOLIT,'$',EQUAL);
  IF(5,HEXX,SWAP,ONEP,SWAP,ONEM);
  THEN(13,OVER,CAT,DOLIT,'-',EQUAL,TOR,SWAP,RAT,SUBBB,SWAP,RAT,PLUS,QDUP);
  IF(1,ONEM);
  FOR(6,DUPP,TOR,CAT,BASE,AT,DIGTQ);
  WHILE(7,SWAP,BASE,AT,STAR,PLUS,RFROM,ONEP);
  NEXT(2,DROP,RAT);
  IF(1,NEGAT);
  THEN(1,SWAP);
  ELSE(6,RFROM,RFROM,DDROP,DDROP,DOLIT,0);
  THEN(1,DUPP);
  THEN(6,RFROM,DDROP,RFROM,BASE,STORE,EXITT);
  HEADER("SPACE");
  int SPACE=COLON(3,BLANK,EMIT,EXITT);
  HEADER("CHARS");
  int CHARS=COLON(4,SWAP,DOLIT,0,MAX);
  FOR(0);
  AFT(2,DUPP,EMIT);
  THEN(0);
  NEXT(2,DROP,EXITT);
  HEADER("SPACES");
  int SPACS=COLON(3,BLANK,CHARS,EXITT);
  HEADER("TYPE");
  int TYPES=COLON(0);
  FOR(0);
  AFT(5,DUPP,CAT,TCHAR,EMIT,ONEP);
  THEN(0);
  NEXT(2,DROP,EXITT);
  HEADER("CR");
  int CR=COLON(7,DOLIT,'\n',DOLIT,'\r',EMIT,EMIT,EXITT);
  HEADER("do$");
  int DOSTR=COLON(10,RFROM,RAT,RFROM,COUNT,PLUS,ALIGN,TOR,SWAP,TOR,EXITT);
  HEADER("$\"|");
  int STRQP=COLON(2,DOSTR,EXITT);
  HEADER(".\"|");
  DOTQP=COLON(4,DOSTR,COUNT,TYPES,EXITT);
  HEADER(".R");
  int DOTR=COLON(8,TOR,STRR,RFROM,OVER,SUBBB,SPACS,TYPES,EXITT);
  HEADER("U.R");
  int UDOTR=COLON(10,TOR,BDIGS,DIGS,EDIGS,RFROM,OVER,SUBBB,SPACS,TYPES,EXITT);
  HEADER("U.");
  int UDOT=COLON(6,BDIGS,DIGS,EDIGS,SPACE,TYPES,EXITT);
  HEADER(".");
  int DOT=COLON(5,BASE,AT,DOLIT,10,XORR);
  IF(3,UDOT,EXITT);
  THEN(4,STRR,SPACE,TYPES,EXITT);
  HEADER("?");
  int QUEST=COLON(3,AT,DOT,EXITT);
  HEADER("(parse)");
  int PARS=COLON(5,TEMP,CSTOR,OVER,TOR,DUPP);
  IF(5,ONEM,TEMP,CAT,BLANK,EQUAL);
  IF(0);
  FOR(6,BLANK,OVER,CAT,SUBBB,ZLESS,INVER);
  WHILE(1,ONEP);
  NEXT(6,RFROM,DROP,DOLIT,0,DUPP,EXITT);
  THEN(1,RFROM);
  THEN(2,OVER,SWAP);
  FOR(9,TEMP,CAT,OVER,CAT,SUBBB,TEMP,CAT,BLANK,EQUAL);
  IF(1,ZLESS);
  THEN(0);
  WHILE(1,ONEP);
  NEXT(2,DUPP,TOR);
  ELSE(5,RFROM,DROP,DUPP,ONEP,TOR);
  THEN(6,OVER,SUBBB,RFROM,RFROM,SUBBB,EXITT);
  THEN(4,OVER,RFROM,SUBBB,EXITT);
  HEADER("PACK$");
  int PACKS=COLON(18,DUPP,TOR,DDUP,PLUS,DOLIT,~CELL_MASK,ANDD,DOLIT,0,SWAP,STORE,DDUP,CSTOR,ONEP,SWAP,CMOVEE,RFROM,EXITT);
  HEADER("PARSE");
  int PARSE=COLON(15,TOR,TIB,INN,AT,PLUS,NTIB,AT,INN,AT,SUBBB,RFROM,PARS,INN,PSTOR,EXITT);
  HEADER("TOKEN");
  int TOKEN=COLON(9,BLANK,PARSE,DOLIT,0x1F,MIN,HERE,CELLP,PACKS,EXITT);
  HEADER("WORD");
  int WORDD=COLON(5,PARSE,HERE,CELLP,PACKS,EXITT);
  HEADER("NAME>");
  int NAMET=COLON(7,COUNT,DOLIT,0x1F,ANDD,PLUS,ALIGN,EXITT);
  HEADER("SAME?");
  int SAMEQ=COLON(4,DOLIT,0x1F,ANDD,CELLD);
  FOR(0);
  AFT(14,OVER,RAT,CELLS,PLUS,AT,UPPER,OVER,RAT,CELLS,PLUS,AT,UPPER,SUBBB,QDUP);
  IF(3,RFROM,DROP,EXITT);
  THEN(0);
  THEN(0);
  NEXT(3,DOLIT,0,EXITT);
  HEADER("find");
  int FIND=COLON(10,SWAP,DUPP,AT,TEMP,STORE,DUPP,AT,TOR,CELLP,SWAP);
  BEGIN(2,AT,DUPP);
  IF(9,DUPP,AT,DOLIT,~0xC0,ANDD,UPPER,RAT,UPPER,XORR);
  IF(3,CELLP,DOLIT,-1);
  ELSE(4,CELLP,TEMP,AT,SAMEQ);
  THEN(0);
  ELSE(6,RFROM,DROP,SWAP,CELLM,SWAP,EXITT);
  THEN(0);
  WHILE(2,CELLM,CELLM);
  REPEAT(9,RFROM,DROP,SWAP,DROP,CELLM,DUPP,NAMET,SWAP,EXITT);
  HEADER("NAME?");
  int NAMEQ=COLON(3,CNTXT,FIND,EXITT);
  HEADER("EXPECT");
  int EXPEC=COLON(5,ACCEP,SPAN,STORE,DROP,EXITT);
  HEADER("QUERY");
  int QUERY=COLON(12,TIB,DOLIT,0x100,ACCEP,NTIB,STORE,DROP,DOLIT,0,INN,STORE,EXITT);
  HEADER("ABORT");
  int ABORT=COLON(4,NOP,TABRT,ATEXE,EXITT);
  HEADER("abort\"");
  ABORQP=COLON(0);
  IF(4,DOSTR,COUNT,TYPES,ABORT);
  THEN(3,DOSTR,DROP,EXITT);
  HEADER("ERROR");
  int ERRORR=COLON(8,SPACE,COUNT,TYPES,DOLIT,'?',EMIT,CR,ABORT);
  HEADER("$INTERPRET");
  int INTER=COLON(2,NAMEQ,QDUP);
  IF(4,CAT,DOLIT,COMPO,ANDD);
  ABORQ(" compile only");
  int INTER0=LABEL(2,EXECU,EXITT);
  THEN(1,NUMBQ);
  IF(1,EXITT);
  THEN(1,ERRORR);
  HEADER_IMMEDIATE("[");
  int LBRAC=COLON(5,DOLIT,INTER,TEVAL,STORE,EXITT);
  HEADER(".OK");
  int DOTOK=COLON(6,CR,DOLIT,INTER,TEVAL,AT,EQUAL);
  IF(14,TOR,TOR,TOR,DUPP,DOT,RFROM,DUPP,DOT,RFROM,DUPP,DOT,RFROM,DUPP,DOT);
  DOTQ(" ok>");
  THEN(1,EXITT);
  HEADER("EVAL");
  int EVAL=COLON(1,LBRAC);
  BEGIN(3,TOKEN,DUPP,AT);
  WHILE(2,TEVAL,ATEXE);
  REPEAT(4,DROP,DOTOK,NOP,EXITT);
  HEADER("QUIT");
  int QUITT=COLON(1,LBRAC);
  BEGIN(2,QUERY,EVAL);
  AGAIN(0);
  HEADER("LOAD");
  int LOAD=COLON(10,NTIB,STORE,TTIB,STORE,DOLIT,0,INN,STORE,EVAL,EXITT);
  HEADER(",");
  int COMMA=COLON(7,HERE,DUPP,CELLP,CP,STORE,STORE,EXITT);
  HEADER_IMMEDIATE("LITERAL");
  int LITER=COLON(5,DOLIT,DOLIT,COMMA,COMMA,EXITT);
  HEADER("ALLOT");
  int ALLOT=COLON(4,ALIGN,CP,PSTOR,EXITT);
  HEADER("$,\"");
  int STRCQ=COLON(9,DOLIT,'"',WORDD,COUNT,PLUS,ALIGN,CP,STORE,EXITT);
  HEADER("?UNIQUE");
  int UNIQU=COLON(3,DUPP,NAMEQ,QDUP);
  IF(6,COUNT,DOLIT,0x1F,ANDD,SPACE,TYPES);
  DOTQ(" reDef");
  THEN(2,DROP,EXITT);
  HEADER("$,n");
  int SNAME=COLON(2,DUPP,AT);
  IF(14,UNIQU,DUPP,NAMET,CP,STORE,DUPP,LAST,STORE,CELLM,CNTXT,AT,SWAP,STORE,EXITT);
  THEN(1,ERRORR);
  HEADER("'");
  int TICK=COLON(2,TOKEN,NAMEQ);
  IF(1,EXITT);
  THEN(1,ERRORR);
  HEADER_IMMEDIATE("[COMPILE]");
  int BCOMP=COLON(3,TICK,COMMA,EXITT);
  HEADER("COMPILE");
  int COMPI=COLON(7,RFROM,DUPP,AT,COMMA,CELLP,TOR,EXITT);
  HEADER("$COMPILE");
  int SCOMP=COLON(2,NAMEQ,QDUP);
  IF(4,AT,DOLIT,IMEDD,ANDD);
  IF(1,EXECU);
  ELSE(1,COMMA);
  THEN(1,EXITT);
  THEN(1,NUMBQ);
  IF(2,LITER,EXITT);
  THEN(1,ERRORR);
  HEADER("OVERT");
  int OVERT=COLON(5,LAST,AT,CNTXT,STORE,EXITT);
  HEADER("]");
  int RBRAC=COLON(5,DOLIT,SCOMP,TEVAL,STORE,EXITT);
  HEADER(":");
  int COLN=COLON(7,TOKEN,SNAME,RBRAC,DOLIT,as_dolist,COMMA,EXITT);
  HEADER_IMMEDIATE(";");
  int SEMIS=COLON(6,DOLIT,EXITT,COMMA,LBRAC,OVERT,EXITT);
  HEADER("dm+");
  int DMP=COLON(4,OVER,DOLIT,6,UDOTR);
  FOR(0);
  AFT(6,DUPP,AT,DOLIT,9,UDOTR,CELLP);
  THEN(0);
  NEXT(1,EXITT);
  HEADER("DUMP");
  int DUMP=COLON(10,BASE,AT,TOR,HEXX,DOLIT,0x1F,PLUS,DOLIT,0x20,SLASH);
  FOR(0);
  AFT(10,CR,DOLIT,8,DDUP,DMP,TOR,SPACE,CELLS,TYPES,RFROM);
  THEN(0);
  NEXT(5,DROP,RFROM,BASE,STORE,EXITT);
  HEADER(">NAME");
  int TNAME=COLON(1,CNTXT);
  BEGIN(2,AT,DUPP);
  WHILE(3,DDUP,NAMET,XORR);
  IF(1,ONEM);
  ELSE(3,SWAP,DROP,EXITT);
  THEN(0);
  REPEAT(3,SWAP,DROP,EXITT);
  HEADER(".ID");
  int DOTID=COLON(7,COUNT,DOLIT,0x1F,ANDD,TYPES,SPACE,EXITT);
  HEADER("WORDS");
  int WORDS=COLON(6,CR,CNTXT,DOLIT,0,TEMP,STORE);
  BEGIN(2,AT,QDUP);
  WHILE(9,DUPP,SPACE,DOTID,CELLM,TEMP,AT,DOLIT,0x10,LESS);
  IF(4,DOLIT,1,TEMP,PSTOR);
  ELSE(5,CR,DOLIT,0,TEMP,STORE);
  THEN(0);
  REPEAT(1,EXITT);
  HEADER("FORGET");
  int FORGT=COLON(3,TOKEN,NAMEQ,QDUP);
  IF(12,CELLM,DUPP,CP,STORE,AT,DUPP,CNTXT,STORE,LAST,STORE,DROP,EXITT);
  THEN(1,ERRORR);
  HEADER("COLD");
  int COLD=COLON(1,CR);
  DOTQ("esp32forth V6.3, 2019 ");
  int DOTQ1=LABEL(2,CR,EXITT);
  HEADER("LINE");
  int LINE=COLON(2,DOLIT,0x7);
  FOR(6,DUPP,PEEK,DOLIT,0x9,UDOTR,CELLP);
  NEXT(1,EXITT);
  HEADER("PP");
  int PP=COLON(0);
  FOR(0);
  AFT(7,CR,DUPP,DOLIT,0x9,UDOTR,SPACE,LINE);
  THEN(0);
  NEXT(1,EXITT);
  HEADER("P0");
  int P0=COLON(4,DOLIT,0x3FF44004,POKE,EXITT);
  HEADER("P0S");
  int P0S=COLON(4,DOLIT,0x3FF44008,POKE,EXITT);
  HEADER("P0C");
  int P0C=COLON(4,DOLIT,0x3FF4400C,POKE,EXITT);
  HEADER("P1");
  int P1=COLON(4,DOLIT,0x3FF44010,POKE,EXITT);
  HEADER("P1S");
  int P1S=COLON(4,DOLIT,0x3FF44014,POKE,EXITT);
  HEADER("P1C");
  int P1C=COLON(4,DOLIT,0x3FF44018,POKE,EXITT);
  HEADER("P0EN");
  int P0EN=COLON(4,DOLIT,0x3FF44020,POKE,EXITT);
  HEADER("P0ENS");
  int P0ENS=COLON(4,DOLIT,0x3FF44024,POKE,EXITT);
  HEADER("P0ENC");
  int P0ENC=COLON(4,DOLIT,0x3FF44028,POKE,EXITT);
  HEADER("P1EN");
  int P1EN=COLON(4,DOLIT,0x3FF4402C,POKE,EXITT);
  HEADER("P1ENS");
  int P1ENS=COLON(4,DOLIT,0x3FF44030,POKE,EXITT);
  HEADER("P1ENC");
  int P1ENC=COLON(4,DOLIT,0x3FF44034,POKE,EXITT);
  HEADER("P0IN");
  int P0IN=COLON(5,DOLIT,0x3FF4403C,PEEK,DOT,EXITT);
  HEADER("P1IN");
  int P1IN=COLON(5,DOLIT,0x3FF44040,PEEK,DOT,EXITT);
  HEADER("PPP");
  int PPP=COLON(7,DOLIT,0x3FF44000,DOLIT,3,PP,DROP,EXITT);
  HEADER("EMITT");
  int EMITT=COLON(2,DOLIT,0x3);
  FOR(8,DOLIT,0,DOLIT,0x100,MSMOD,SWAP,TCHAR,EMIT);
  NEXT(2,DROP,EXITT);
  HEADER("TYPEE");
  int TYPEE=COLON(3,SPACE,DOLIT,0x7);
  FOR(4,DUPP,PEEK,EMITT,CELLP);
  NEXT(2,DROP,EXITT);
  HEADER("PPPP");
  int PPPP=COLON(0);
  FOR(0);
  AFT(10,CR,DUPP,DUPP,DOLIT,0x9,UDOTR,SPACE,LINE,SWAP,TYPEE);
  THEN(0);
  NEXT(1,EXITT);
  HEADER("KKK");
  int KKK=COLON(7,DOLIT,0x3FF59000,DOLIT,0x10,PP,DROP,EXITT);
  HEADER_IMMEDIATE("THEN");
  int THENN=COLON(4,HERE,SWAP,STORE,EXITT);
  HEADER_IMMEDIATE("FOR");
  int FORR=COLON(4,COMPI,TOR,HERE,EXITT);
  HEADER_IMMEDIATE("BEGIN");
  int BEGIN=COLON(2,HERE,EXITT);
  HEADER_IMMEDIATE("NEXT");
  int NEXT=COLON(4,COMPI,DONXT,COMMA,EXITT);
  HEADER_IMMEDIATE("UNTIL");
  int UNTIL=COLON(4,COMPI,QBRAN,COMMA,EXITT);
  HEADER_IMMEDIATE("AGAIN");
  int AGAIN=COLON(4,COMPI,BRAN,COMMA,EXITT);
  HEADER_IMMEDIATE("IF");
  int IFF=COLON(7,COMPI,QBRAN,HERE,DOLIT,0,COMMA,EXITT);
  HEADER_IMMEDIATE("AHEAD");
  int AHEAD=COLON(7,COMPI,BRAN,HERE,DOLIT,0,COMMA,EXITT);
  HEADER_IMMEDIATE("REPEAT");
  int REPEA=COLON(3,AGAIN,THENN,EXITT);
  HEADER_IMMEDIATE("AFT");
  int AFT=COLON(5,DROP,AHEAD,HERE,SWAP,EXITT);
  HEADER_IMMEDIATE("ELSE");
  int ELSEE=COLON(4,AHEAD,SWAP,THENN,EXITT);
  HEADER_IMMEDIATE("WHILE");
  int WHILEE=COLON(3,IFF,SWAP,EXITT);
  HEADER_IMMEDIATE("ABORT\"");
  int ABRTQ=COLON(6,DOLIT,ABORQP,HERE,STORE,STRCQ,EXITT);
  HEADER_IMMEDIATE("$\"");
  int STRQ=COLON(6,DOLIT,STRQP,HERE,STORE,STRCQ,EXITT);
  HEADER_IMMEDIATE(".\"");
  int DOTQQ=COLON(6,DOLIT,DOTQP,HERE,STORE,STRCQ,EXITT);
  HEADER("CODE");
  int CODE=COLON(8,TOKEN,SNAME,OVERT,HERE,ALIGN,CP,STORE,EXITT);
  HEADER("CREATE");
  int CREAT=COLON(5,CODE,DOLIT,as_dovar + (as_next << 8),COMMA,EXITT);
  HEADER("VARIABLE");
  int VARIA=COLON(5,CREAT,DOLIT,0,COMMA,EXITT);
  HEADER("CONSTANT");
  int CONST=COLON(6,CODE,DOLIT,as_docon + (as_next << 8),COMMA,COMMA,EXITT);
  HEADER_IMMEDIATE(".(");
  int DOTPR=COLON(5,DOLIT,')',PARSE,TYPES,EXITT);
  HEADER_IMMEDIATE("\\");
  int BKSLA=COLON(5,DOLIT,'\n',WORDD,DROP,EXITT);
  HEADER_IMMEDIATE("(");
  int PAREN=COLON(5,DOLIT,')',PARSE,DDROP,EXITT);
  HEADER("COMPILE-ONLY");
  int ONLY=COLON(6,DOLIT,COMPO,LAST,AT,PSTOR,EXITT);
  HEADER("IMMEDIATE");
  int IMMED=COLON(6,DOLIT,IMEDD,LAST,AT,PSTOR,EXITT);
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
                 as_dolist,EVAL,0,0,
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


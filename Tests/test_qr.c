#include "app/qr_app.h"
#include "app/app_runtime.h"
#include "usart/bsp_qr_uart.h"
#include "usart/bsp_qr_trigger.h"
#include "usart/bsp_motor_uart.h"
#include "motor/motor_test_config.h"
#include "gyro/gyro_config.h"
#include "lcd/bsp_lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
void GyroTest_Fail(unsigned value);
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static const char sample[] = "156+123+516+231";
static const char other[] = "123+312+321+123";
static void receive(QrService *s, const char *t, uint32_t at)
{ QrService_Receive(s, (const uint8_t *)t, strlen(t), at); }
static void test_validation(void)
{
    QrTaskCode c, saved;
    const char *bad[] = {"", "156+123+516+23", "156+123+516+231x", "x156+123+516+231",
        "156-123+516+231", "056+123+516+231", "176+123+516+231", "116+123+516+231",
        "156+113+516+231", "156+124+516+231", "156+123+516+230", "156+123+512+231",
        "156+123+566+231", "156+123+516+223", " 156+123+516+231", "156+123+516+231 "};
    uint8_t nonascii[15];
    size_t i;
    CHECK(QrTaskCode_Parse((const uint8_t *)sample, 15u, &c) == QR_CODE_OK);
    CHECK(!strcmp(c.text, sample) && c.colors[0][0] == 1u && c.colors[0][1] == 5u &&
          c.colors[0][2] == 6u && c.colors[1][0] == 5u && c.colors[1][1] == 1u && c.colors[1][2] == 6u);
    CHECK(c.positions[0][0] == 1u && c.positions[0][1] == 2u && c.positions[0][2] == 3u &&
          c.positions[1][0] == 2u && c.positions[1][1] == 3u && c.positions[1][2] == 1u);
    saved = c;
    for (i=0u; i<sizeof(bad)/sizeof(bad[0]); ++i) {
        CHECK(QrTaskCode_Parse((const uint8_t *)bad[i], strlen(bad[i]), &c) != QR_CODE_OK);
        CHECK(!strcmp(c.text, saved.text));
    }
    memcpy(nonascii, sample, 15u); nonascii[0] = 0xB1u;
    CHECK(QrTaskCode_Parse(nonascii, 15u, &c) != QR_CODE_OK);
    nonascii[0] = 0u;
    CHECK(QrTaskCode_Parse(nonascii, 15u, &c) != QR_CODE_OK);
    CHECK(QrTaskCode_Parse(0, 15u, &c) == QR_CODE_LENGTH);
    CHECK(QrTaskCode_Parse((const uint8_t *)other, 15u, &c) == QR_CODE_OK);
    /* Exhaust all distinct color triples and all permutations of each batch. */
    {
        unsigned a,b,d,p,q,r;
        char t[16];
        for(a=1;a<=6;++a) for(b=1;b<=6;++b) for(d=1;d<=6;++d) {
            if(a==b || a==d || b==d) continue;
            for(p=1;p<=3;++p) for(q=1;q<=3;++q) for(r=1;r<=3;++r) {
                if(p==q || p==r || q==r) continue;
                snprintf(t,sizeof(t),"%u%u%u+%u%u%u+%u%u%u+321",a,b,d,p,q,r,d,a,b);
                CHECK(QrTaskCode_Parse((const uint8_t *)t,15u,&c)==QR_CODE_OK);
            }
        }
    }
}
static void test_stream_sessions(void)
{
    QrService s;
    QrTaskCode event;
    size_t split;
    unsigned i;
    uint8_t ack[] = {2,0,0,1,0,0x33,0x31,'\r'};
    const char *endings[] = {"\r","\n","\r\n"};
    for(i=0;i<3;++i) for(split=0;split<=15u;++split) {
        QrService_Init(&s,1u,0u);
        QrService_Receive(&s,(const uint8_t *)sample,split,10u);
        QrService_Receive(&s,(const uint8_t *)sample+split,15u-split,30u);
        CHECK(!s.data.valid); /* 15 bytes and IDLE alone do not accept a prefix. */
        receive(&s,endings[i],40u);
        CHECK(s.data.valid && s.data.records==1u && s.data.accepted_seq==1u);
        CHECK(s.data.accepted_ms==40u && QrService_TakeAccepted(&s,&event));
        CHECK(!QrService_TakeAccepted(&s,0) && !strcmp(event.text,sample));
    }
    QrService_Init(&s,1u,0u);
    receive(&s,"156+123+516+231\r\n156+123+516+231\r123+312+321+123\n",1u);
    CHECK(s.data.records==3u && s.data.duplicates==1u && s.data.ignored==1u);
    CHECK(s.data.accepted_seq==1u && !strcmp(s.data.current.text,sample));
    receive(&s,"junk\r",2u); CHECK(s.data.valid && s.data.invalid_records==1u);
    CHECK(s.data.last_error==QR_ERROR_INVALID && !strcmp(s.data.history.text,sample));
    receive(&s,"old partial",3u);
    QrService_BeginSession(&s,4u);
    CHECK(!s.data.valid && !QrService_TakeAccepted(&s,0) && s.data.history_valid);
    receive(&s,"156+123+516+231\r",5u); CHECK(!s.data.valid); /* no new boundary yet */
    receive(&s,"156+123+516+231\r",6u);
    CHECK(s.data.valid && s.data.session==2u && s.data.accepted_seq==2u);
    QrService_BeginSession(&s,7u); QrService_Process(&s,257u);
    receive(&s,"156+123+516+231\r",258u); CHECK(s.data.valid && s.data.accepted_seq==3u);
    QrService_BeginSession(&s,300u); QrService_Process(&s,10300u);
    CHECK(!strcmp(QrService_StateName(&s),"WAIT TIMEOUT"));
    receive(&s,"INVALID\r",10301u); CHECK(!s.data.valid);
    receive(&s,"156+123+516+231\n",10302u); CHECK(s.data.valid);
    QrService_Init(&s,1u,UINT32_MAX-5000u); QrService_Process(&s,4999u);
    CHECK(s.data.timed_out); receive(&s,"156+123+516+231\r",5000u); CHECK(s.data.valid);
    QrService_Init(&s,1u,0u);
    receive(&s,"156+123",10u); QrService_Process(&s,260u);
    CHECK(s.data.frame_timeouts==1u && !s.data.valid);
    receive(&s,"+516+231\r156+123+516+231\r",300u); CHECK(s.data.valid && s.data.records==1u);
    QrService_Init(&s,1u,0u);
    for(i=0;i<100u;++i) receive(&s,"x",i);
    receive(&s,"156+123+516+231\r",101u);
    CHECK(!s.data.valid && s.data.overflows==1u);
    receive(&s,"156+123+516+231\r",102u); CHECK(s.data.valid);
    QrService_Init(&s,1u,0u);
    receive(&s,"noise156+123+516+231\n156+123+516+231junk\n",1u);
    QrService_Receive(&s,ack,sizeof(ack),2u); CHECK(!s.data.valid && s.data.invalid_records==3u);
    receive(&s,"156+123+516+231MORE\r",3u); CHECK(!s.data.valid);
    receive(&s,"156+123+516+231",10u);
    QrService_TransportError(&s,QR_ERROR_UART,2u);
    receive(&s,"\r156+123+516+231\r",11u); CHECK(s.data.valid && s.data.uart_errors==2u);
    QrService_TransportError(&s,QR_ERROR_OVERFLOW,1u);
    CHECK(s.data.valid && !strcmp(s.data.current.text,sample));
    QrService_Init(&s,0u,0u); receive(&s,"156+123+516+231\r",1u);
    CHECK(!s.data.valid && !strcmp(QrService_StateName(&s),"UART ERROR"));
}

/* Real BSP and callback dispatcher run against a deterministic HAL/DMA substitute. */
USART_TypeDef fake_usart1, fake_usart3, fake_uart5;
DMA_Stream_TypeDef fake_stream1;
GPIO_TypeDef fake_gpiob;
static UART_HandleTypeDef *uart;
static uint8_t *dma_memory;
static uint16_t dma_size;
static uint32_t tick, irq_mask;
static int init_fail, dma_init_fail, receive_fail, dma_start_fail;
static unsigned dma_event, motor_rx_events, motor_errors, motor_tx_events, motor_pumps, aborts;
static unsigned gpio_checked, priority_checked, trig_gpio_inits, trig_lows, trig_highs;
static uint32_t trig_level = GPIO_PIN_SET;
static unsigned trigger_measure, trigger_have;
static uint32_t trigger_at, trigger_max_jitter;
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask=1u; }
void __set_PRIMASK(uint32_t mask) { irq_mask=mask; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u)
{ uart=u; BspQrUart_MspInit(u); return init_fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *d)
{
    CHECK(d->Instance==DMA1_Stream1 && d->Init.Channel==DMA_CHANNEL_4 && d->Init.Mode==DMA_CIRCULAR);
    d->State=HAL_DMA_STATE_READY; return dma_init_fail ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    if(receive_fail) return HAL_ERROR;
    dma_memory=b; dma_size=n; u->hdmarx->Instance->count=n;
    if(!dma_start_fail) u->hdmarx->State=HAL_DMA_STATE_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *u)
{ ++aborts; if(u->hdmarx) u->hdmarx->State=HAL_DMA_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *d) { d->State=HAL_DMA_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *d) { d->State=HAL_DMA_STATE_READY; return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *g)
{
    if (g->Pin == (GPIO_PIN_10 | GPIO_PIN_11)) {
        CHECK(port==GPIOB && g->Mode==GPIO_MODE_AF_PP && g->Alternate==GPIO_AF7_USART3);
        ++gpio_checked;
    } else {
        CHECK(port==GPIOB && g->Pin==QR_TRIG_PIN && g->Mode==GPIO_MODE_OUTPUT_PP);
        ++trig_gpio_inits;
    }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    /* Called every trigger phase; validate without incrementing the check count. */
    if (port != GPIOB || pin != QR_TRIG_PIN) { fprintf(stderr,"FAIL: trigger GPIO write\n"); exit(1); }
    if(trigger_measure && state!=trig_level) {
        uint32_t expected=trig_level==GPIO_PIN_RESET ? QR_TRIG_LOW_MS : QR_TRIG_PERIOD_MS-QR_TRIG_LOW_MS;
        if(trigger_have) {
            uint32_t dt=tick-trigger_at;
            CHECK(dt>=expected);
            if(dt-expected>trigger_max_jitter) trigger_max_jitter=dt-expected;
        }
        trigger_at=tick; trigger_have=1u;
    }
    if (state == GPIO_PIN_RESET) ++trig_lows; else ++trig_highs;
    trig_level = state;
}
void HAL_NVIC_SetPriority(unsigned irq, unsigned preempt, unsigned sub)
{ CHECK((irq==USART3_IRQn || irq==DMA1_Stream1_IRQn) && preempt==2u && sub==0u); ++priority_checked; }
void HAL_NVIC_EnableIRQ(unsigned irq) { (void)irq; }
void HAL_NVIC_DisableIRQ(unsigned irq) { (void)irq; }
void HAL_NVIC_ClearPendingIRQ(unsigned irq) { (void)irq; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *u)
{ if(u->Instance->SR & 15u) { HAL_UART_ErrorCallback(u); u->Instance->SR &= ~15u; } }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d)
{
    if(dma_event==1u) HAL_UART_RxHalfCpltCallback((UART_HandleTypeDef *)d->Parent);
    else HAL_UART_RxCpltCallback((UART_HandleTypeDef *)d->Parent);
}
void BspMotorUart_RxEvent(void) { ++motor_rx_events; }
void BspMotorUart_Error(void) { ++motor_errors; }
void BspMotorUart_TxComplete(void) { ++motor_tx_events; }
static void (*motor_pump_hook)(MotorBus *,uint32_t);
void BspMotorUart_PumpBus(MotorBus *s, uint32_t now)
{ ++motor_pumps; if(motor_pump_hook) motor_pump_hook(s,now); }
extern void USART3_IRQHandler(void);
extern void DMA1_Stream1_IRQHandler(void);
static void inject(const char *b, uint32_t at, int idle)
{
    size_t i;
    tick=at;
    for(i=0;i<strlen(b);++i) {
        dma_memory[dma_size-fake_stream1.count]=(uint8_t)b[i];
        --fake_stream1.count;
        if(fake_stream1.count==dma_size/2u) { dma_event=1u; DMA1_Stream1_IRQHandler(); }
        if(!fake_stream1.count) { fake_stream1.count=dma_size; dma_event=2u; DMA1_Stream1_IRQHandler(); }
    }
    if(idle) { fake_usart3.SR |= UART_FLAG_IDLE; USART3_IRQHandler(); }
}

/* Pixel/text LCD substitute: validates bounds and saves actual font renderings. */
static uint8_t pixels[272][480];
static sFONT *font;
static uint32_t foreground, background;
static unsigned draws, clears, bad_layout;
static char lines[9][64];
/* Same monotonic clock as HAL/gyro. Costs are injected, never board timings. */
static unsigned lcd_timed, lcd_overrun, lcd_max_step;
static uint32_t lcd_clear_ms=12u, lcd_fill_ms=1u, lcd_text_ms=1u, lcd_setup_ms;
static const unsigned ys[9]={8,32,60,92,114,136,158,180,202};
void LCD_SetFont(sFONT *f) { if(lcd_timed) tick+=lcd_setup_ms; font=f; }
void LCD_SetBackColor(uint32_t c) { if(lcd_timed) tick+=lcd_setup_ms; background=c; }
void LCD_SetTextColor(uint32_t c) { if(lcd_timed) tick+=lcd_setup_ms; foreground=c; }
void LCD_Clear(uint32_t c) { if(lcd_timed) tick+=lcd_clear_ms; ++clears; memset(pixels,c?255:0,sizeof(pixels)); memset(lines,0,sizeof(lines)); }
void LCD_FillRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h)
{
    unsigned i,j;
    if(lcd_timed) tick+=lcd_overrun ? 12u : (lcd_fill_ms+((unsigned)w*h)/2000u);
    if((unsigned)x+w>480u || (unsigned)y+h>272u) { ++bad_layout; return; }
    if(x==8u) for(i=0;i<9u;++i) if(y==ys[i]) lines[i][0]='\0';
    for(i=y;i<(unsigned)y+h;++i) for(j=x;j<(unsigned)x+w;++j) pixels[i][j]=foreground?255u:0u;
}
void LCD_DisplayStringAt(uint16_t x,uint16_t y,uint8_t *text,unsigned mode)
{
    unsigned row,i,j,k,byte_width,offset;
    uint32_t bits;
    (void)mode; ++draws;
    if(lcd_timed) tick+=lcd_text_ms+(unsigned)strlen((char *)text)/2u;
    if(!font || (unsigned)x+strlen((char *)text)*font->Width>480u || (unsigned)y+font->Height>272u) {
        ++bad_layout; return;
    }
    for(row=0;row<9u;++row) if(y==ys[row] && x>=8u) {
        unsigned col=(x-8u)/font->Width;
        if(col+strlen((char *)text)<sizeof(lines[row])) strcpy(lines[row]+col,(char *)text);
    }
    byte_width=(font->Width+7u)/8u;
    for(k=0;text[k];++k) for(i=0;i<font->Height;++i) {
        /* bsp_lcd.c DrawChar first paints 3 background rows, then glyph rows 0..H-4. */
        offset=((unsigned)text[k]-32u)*font->Height*byte_width+(i<3u?0u:i-3u)*byte_width;
        bits=0u;
        if(i>=3u) for(j=0;j<byte_width;++j) bits=(bits<<8)|font->table[offset+j];
        for(j=0;j<font->Width;++j) pixels[y+i][x+k*font->Width+j]=
            (bits & (1u << (byte_width*8u-1u-j))) ? (foreground?255u:0u) : (background?255u:0u);
    }
}
static void save_image(const char *name)
{
    FILE *f=fopen(name,"wb"); CHECK(f!=0);
    fprintf(f,"P5\n480 272\n255\n"); fwrite(pixels,1,sizeof(pixels),f); fclose(f);
}
static void test_bsp(void)
{
    QrService s;
    UART_HandleTypeDef m={0},debug={0};
    unsigned i;
    uint32_t seq;
    CHECK(BspQrUart_Init());
    CHECK(uart->Init.BaudRate==QR_UART_BAUDRATE && gpio_checked==1u && priority_checked==2u);
    /* MCU trigger pin: init high, then 800 ms low + 200 ms high every 1000 ms. */
    BspQrTrigger_Init();
    CHECK(trig_gpio_inits==1u && trig_level==GPIO_PIN_SET);
    BspQrTrigger_Poll(1000u); CHECK(trig_level==GPIO_PIN_RESET && trig_lows==1u);
    BspQrTrigger_Poll(1799u); CHECK(trig_level==GPIO_PIN_RESET);
    BspQrTrigger_Poll(1800u); CHECK(trig_level==GPIO_PIN_SET);
    BspQrTrigger_Poll(1999u); CHECK(trig_level==GPIO_PIN_SET);
    BspQrTrigger_Poll(2000u); CHECK(trig_level==GPIO_PIN_RESET && trig_lows==2u);
    m.Instance=UART5; debug.Instance=USART1;
    HAL_UART_RxHalfCpltCallback(&m); HAL_UART_RxCpltCallback(&m);
    HAL_UART_ErrorCallback(&m); HAL_UART_TxCpltCallback(&m);
    HAL_UART_RxHalfCpltCallback(&debug); HAL_UART_RxCpltCallback(&debug);
    HAL_UART_ErrorCallback(&debug); HAL_UART_TxCpltCallback(uart);
    CHECK(motor_rx_events==2u && motor_errors==1u && motor_tx_events==1u);
    QrService_Init(&s,1u,0u);
    inject("156+123",10u,1); BspQrUart_Pump(&s,10u); CHECK(!s.data.valid);
    inject("+516+231\r\n",20u,1); BspQrUart_Pump(&s,20u); CHECK(s.data.valid && s.data.records==1u);
    /* Cross half/full DMA callbacks repeatedly without IDLE. */
    for(i=0;i<100u;++i) inject("156+123+516+231\r\n",30u,0);
    inject("",31u,1); BspQrUart_Pump(&s,31u);
    CHECK(s.data.overflows==1u && s.data.valid); /* poisoned SW queue never publishes */
    BspQrUart_BeginSession(&s,40u);
    inject("\r156+123+516+231\r",41u,1); BspQrUart_Pump(&s,41u); CHECK(s.data.valid);
    seq=s.data.accepted_seq;
    BspQrUart_BeginSession(&s,50u);
    inject("156+123+516+231",400u,1); BspQrUart_Pump(&s,400u); CHECK(!s.data.valid);
    fake_usart3.SR=UART_FLAG_ORE|UART_FLAG_IDLE;
    tick=401u; USART3_IRQHandler(); BspQrUart_Pump(&s,401u);
    CHECK(s.data.uart_errors==1u && !s.data.valid && aborts==1u);
    inject("516+231\r156+123+516+231\r",402u,1); BspQrUart_Pump(&s,402u);
    CHECK(s.data.valid && s.data.accepted_seq==seq+1u);
    /* Discard an already complete result and a partial DMA record on rescan. */
    inject("123+312+321+123\r156+123",500u,1);
    BspQrUart_BeginSession(&s,501u); BspQrUart_Pump(&s,501u);
    CHECK(!s.data.valid && !QrService_TakeAccepted(&s,0));
    inject("+516+231\r156+123+516+231\r",502u,1); BspQrUart_Pump(&s,502u); CHECK(s.data.valid);
    /* Arrival times survive main-loop delay: a pre-gap partial cannot join a later tail. */
    BspQrUart_BeginSession(&s,600u); inject("\r156+123",601u,1);
    inject("+516+231\r",1000u,1); BspQrUart_Pump(&s,1001u);
    CHECK(!s.data.valid && s.data.frame_timeouts==1u);
    for(i=0;i<40u;++i) {
        inject("156+123+516+231\r\n",1100u+i,1); BspQrUart_Pump(&s,1100u+i);
    }
    CHECK(s.data.valid && s.data.records>=40u && s.data.overflows==1u);
    seq=s.data.accepted_seq;
    fake_usart3.SR=UART_FLAG_FE|UART_FLAG_IDLE; USART3_IRQHandler();
    receive_fail=1; BspQrUart_Pump(&s,1200u);
    CHECK(!s.data.uart_ready && s.data.valid && s.data.accepted_seq==seq);
    receive_fail=0; BspQrUart_Pump(&s,1300u);
    CHECK(s.data.uart_ready && s.data.valid);
    /* Long uptime and a retry deadline spanning SysTick wrap. */
    tick=UINT32_MAX-50u; fake_usart3.SR=UART_FLAG_NE; USART3_IRQHandler();
    receive_fail=1; BspQrUart_Pump(&s,tick); CHECK(!s.data.uart_ready);
    i=aborts;
    tick=25u; BspQrUart_Pump(&s,tick); CHECK(aborts==i);
    receive_fail=0; tick=50u; BspQrUart_Pump(&s,tick);
    CHECK(s.data.uart_ready && s.data.valid && aborts==i+1u);
}

static unsigned motor_sends, motion_sends;
static unsigned app_commands[256];
static uint8_t app_last[128], app_length;
static int motor_send(void *ctx,const uint8_t *b,uint8_t n)
{
    (void)ctx; ++motor_sends;
    ++app_commands[b[1]]; memcpy(app_last,b,n); app_length=n;
    if(b[1]!=EMM_POSITION && b[1]!=EMM_SPEED && b[1]!=EMM_STATUS) ++motion_sends;
    return 1;
}
static int motor_busy(void *ctx) { (void)ctx; return 0; }
static int32_t touch_x,touch_y;
static uint8_t touch_down;
static unsigned touch_polls;
void GTP_TouchProcess(void) { ++touch_polls; }
/* Untimed legacy pixel fixtures explicitly finish the incremental job. The
   review timing tests below use exactly one real QrApp step, with no drain. */
static void finish_ui(QrUi *u,const QrService *q,const MotorBus *m,const FlowSnapshot *f,const GyroSnapshot *g,uint32_t now)
{
    unsigned budget=2048u;
    do { QrUi_Process(u,q,m,f,g,now); } while(u->drawing && --budget);
    CHECK(budget);
}
static void app_step(QrApp *a,uint32_t now)
{
    tick=now; AppTouch_Report(0,touch_x,touch_y,touch_down); QrApp_Process(a,now);
    if(!lcd_timed) {
        FlowSnapshot f; GyroSnapshot g;
        Flow_GetSnapshot(&a->flow,tick,&f); Gyro_GetSnapshot(&a->gyro,tick,&g);
        finish_ui(&a->ui,&a->qr,&a->motor,&f,&g,tick);
    }
}
static void click(QrApp *a,int32_t x,int32_t y,uint32_t now)
{ touch_x=x; touch_y=y; touch_down=1u; app_step(a,now); touch_down=0u; app_step(a,now+1u); }
static void test_app_ui(void)
{
    QrApp a;
    MotorTransport t={0,motor_send,motor_busy};
    uint32_t session,seq,now;
    unsigned previous_draws,previous_clears;
    QrApp_Init(&a,t,1u,2000u); app_step(&a,2000u);
    CHECK(!a.ui.motor_page && a.qr.data.session==1u && !strcmp(lines[2],"--"));
    CHECK(strstr(lines[1],"WAIT CODE")!=0); save_image("qr_wait.pgm");
    previous_draws=draws; app_step(&a,2050u); app_step(&a,2100u);
    CHECK(draws==previous_draws); /* unchanged rows do not redraw */
    app_step(&a,12000u); CHECK(strstr(lines[1],"WAIT TIMEOUT")!=0);
    inject("156+123+516+231\r\n",12001u,1); app_step(&a,12001u); app_step(&a,12101u);
    CHECK(a.qr.data.valid && !strcmp(lines[2],sample)); save_image("qr_locked.pgm");
    session=a.qr.data.session; seq=a.qr.data.accepted_seq;
    click(&a,300,245,12200u); CHECK(a.ui.motor_page && a.qr.data.session==session);
    CHECK(strstr(lines[0],"CHASSIS")!=0); save_image("qr_motor.pgm");
    inject("123+312+321+123\r",12202u,1); app_step(&a,12202u);
    CHECK(a.qr.data.ignored==1u && a.qr.data.accepted_seq==seq);
    click(&a,300,245,12300u); CHECK(!a.ui.motor_page && !strcmp(lines[2],sample));
    touch_x=20; touch_y=245; touch_down=1u;
    app_step(&a,12400u); CHECK(!a.qr.data.valid && !strcmp(lines[2],"--"));
    for(now=12401u;now<13000u;now+=20u) app_step(&a,now);
    CHECK(a.qr.data.session==session+1u); /* a long press only rescans once */
    AppTouch_Report(1,300,245,1u);
    touch_down=0u; app_step(&a,13000u);
    CHECK(a.qr.data.session==session+1u && a.ui.was_down); /* another finger holds the same gesture */
    AppTouch_Report(1,300,245,0u); app_step(&a,13000u);
    inject("156+123+516+231\r",13001u,1); app_step(&a,13001u); CHECK(a.qr.data.accepted_seq==seq+1u);
    inject("bad\r",13002u,1); app_step(&a,13102u);
    CHECK(a.qr.data.valid && strstr(lines[4],"INVALID CODE") && !strcmp(lines[2],sample));
    save_image("qr_invalid_locked.pgm");
    previous_clears=clears;
    /* Press outside, drag into RESCAN, and keep held: never creates a new click. */
    touch_x=0; touch_y=0; touch_down=1u; app_step(&a,13200u);
    touch_x=20; touch_y=245; app_step(&a,13201u);
    CHECK(a.qr.data.session==session+1u && clears==previous_clears);
    touch_down=0; app_step(&a,13202u);
    for(now=14000u;now<20000u;++now) app_step(&a,now);
    CHECK(motor_pumps>1000u && motor_sends>3u && motion_sends==0u && touch_polls>10u);
    a.qr.data.session=a.qr.data.raw_bytes=a.qr.data.records=a.qr.data.invalid_records=UINT32_MAX;
    a.qr.data.overflows=a.qr.data.uart_errors=a.qr.data.duplicates=a.qr.data.ignored=UINT32_MAX;
    a.qr.data.accepted_seq=a.qr.data.accepted_ms=a.qr.data.frame_timeouts=UINT32_MAX;
    a.ui.redraw=1u; app_step(&a,20000u);
    CHECK(bad_layout==0u); /* include maximum 32-bit counters and both font sizes */
    QrApp_Init(&a,t,0u,21000u); finish_ui(&a.ui,&a.qr,&a.motor,0,0,21000u);
    CHECK(strstr(lines[1],"UART ERROR")!=0 && !strcmp(lines[2],"--"));
    save_image("qr_uart_error.pgm");
    QrApp_Init(&a,t,1u,22000u); CHECK(!a.qr.data.valid && a.qr.data.session==1u);
    click(&a,300,245,22001u); CHECK(a.ui.motor_page);
    inject("156+123+516+231\r",22003u,1); app_step(&a,22003u);
    CHECK(a.qr.data.valid && a.qr.data.session==1u);
    click(&a,300,245,22100u); CHECK(!a.ui.motor_page && !strcmp(lines[2],sample));
    CHECK(trig_lows >= 5u); /* QrApp_Process keeps the trigger waveform running */
}
static void test_seven_ui(void)
{
    QrApp a; MotorTransport t={0,motor_send,motor_busy};
    unsigned i, before=motion_sends;
    QrApp_Init(&a,t,1u,30000u);
    click(&a,300,245,30000u);
    CHECK(a.ui.motor_page && !a.motor.manual);
    CHECK(!strcmp(lines[0],a.ui.detail ? "MOTOR READ ONLY" : "CHASSIS [tap] / MOTOR"));
    for(i=0;i<7u;++i) CHECK(strstr(lines[i+1u],MotorBus_Role((uint8_t)(i+1u))) && strstr(lines[i+1u],"--"));
    save_image("motor_seven_wait.pgm");
    click(&a,20,140,30002u); CHECK(a.ui.detail && a.ui.selected==4u);
    CHECK(strstr(lines[2],"--"));
    click(&a,20,245,30004u); CHECK(a.motor.manual && strstr(lines[0],"MANUAL"));
    CHECK(!strcmp(lines[0],"MOTOR MANUAL / STOP READY"));
    click(&a,20,140,30006u); CHECK(a.ui.test_action==1u);
    click(&a,20,184,30008u);
    CHECK(motion_sends==before);
#if MOTOR_TEST_CONFIGURED
    CHECK(strstr(a.motor.message,"NEED"));
#elif MOTOR_BENCH_JOG_ENABLED
    /* Bench jog reaches Submit but still requires healthy feedback. */
    CHECK(strstr(a.motor.message,"NEED"));
#else
    CHECK(strstr(a.motor.message,"CONFIGURE"));
#endif
    /* UI edge detection applies equally to one-shot position and group actions. */
    a.ui.test_action=2u;
    CHECK(QrUi_Touch(&a.ui,20,184,1u)==QR_UI_RUN);
    CHECK(QrUi_Touch(&a.ui,20,184,1u)==QR_UI_NONE);
    CHECK(QrUi_Touch(&a.ui,20,184,0u)==QR_UI_NONE);
    CHECK(QrUi_Touch(&a.ui,170,245,1u)==QR_UI_STOP_ALL);
    (void)QrUi_Touch(&a.ui,170,245,0u);
    CHECK(QrUi_Touch(&a.ui,300,184,1u)==QR_UI_STOP_AXIS);
    (void)QrUi_Touch(&a.ui,300,184,0u);
    a.motor.nodes[4].data.valid=7u; a.motor.nodes[4].data.online=1u;
    a.motor.nodes[4].data.position_counts=-INT64_C(4294967295);
    a.motor.nodes[4].target_counts=INT64_C(4294967295); a.motor.nodes[4].target_known=1u;
    a.motor.nodes[4].data.speed_rpm=INT32_MIN;
    a.ui.redraw=1u; finish_ui(&a.ui,&a.qr,&a.motor,0,0,UINT32_MAX);
    CHECK(!bad_layout && strstr(lines[2],"-4294967295")); save_image("motor_detail_stale.pgm");
    a.ui.detail=0; a.ui.redraw=1u;
    for(i=0;i<7u;++i) {
        MotorSnapshot *d=&a.motor.nodes[i].data;
        d->valid=7u; d->online=1u; d->speed_rpm=(int32_t)i*10;
        d->status_flags=3u; d->speed_ms=d->status_ms=31000u;
    }
    finish_ui(&a.ui,&a.qr,&a.motor,0,0,31000u); save_image("motor_seven_online.pgm");
    click(&a,20,245,31002u); CHECK(!a.motor.manual && a.motor.stop_mask);
    {
        unsigned x,y;
        CHECK(!strcmp(lines[0],a.ui.detail ? "MOTOR READ ONLY" : "CHASSIS [tap] / MOTOR"));
        for(y=8u;y<24u;++y) for(x=8u+(unsigned)strlen(lines[0])*Font16.Width;x<312u;++x) CHECK(!pixels[y][x]);
    }
    CHECK(!bad_layout);
}
#if MOTOR_TEST_CONFIGURED
static void manual_fixture(QrApp *a,uint32_t now,uint8_t action,uint8_t selected)
{
    MotorTransport t={0,motor_send,motor_busy}; unsigned i,j;
    QrApp_Init(a,t,1u,now); a->ui.motor_page=a->ui.detail=1u;
    a->ui.test_action=action; a->ui.selected=selected;
    MotorBus_SetManual(&a->motor,1u,now);
    for(i=0;i<7u;++i) {
        MotorSnapshot *d=&a->motor.nodes[i].data;
        d->valid=7u; d->online=1u; d->status_flags=3u; d->rx_count=1u;
        d->position_ms=d->speed_ms=d->status_ms=d->last_rx_ms=now;
        for(j=0;j<3u;++j) a->motor.nodes[i].poll_due[j]=now+10000u;
    }
}
static void test_configured_actions(void)
{
    QrApp a; unsigned before; uint32_t now;
    manual_fixture(&a,40000u,2u,4u); before=app_commands[EMM_MOVE];
    touch_x=20; touch_y=184; touch_down=1u;
    for(now=40000u;now<40040u;++now) app_step(&a,now);
    CHECK(app_commands[EMM_MOVE]==before+1u && app_last[0]==5u && app_last[10]==2u);
    touch_down=0u; app_step(&a,40040u);
    manual_fixture(&a,41000u,1u,6u); before=app_commands[EMM_VELOCITY];
    click(&a,20,184,41000u);
    CHECK(app_commands[EMM_VELOCITY]==before+1u && app_last[0]==7u && app_length==8u);
    manual_fixture(&a,42000u,3u,0u); before=app_commands[EMM_MULTI];
    click(&a,20,184,42000u);
    CHECK(app_commands[EMM_MULTI]==before+1u && app_length==37u && app_last[3]==37u);
    manual_fixture(&a,43000u,4u,0u); before=app_commands[EMM_SYNC];
    click(&a,20,184,43000u); CHECK(app_commands[EMM_SYNC]==before && strstr(a.motor.message,"LOCKED"));
    CHECK(!bad_layout);
}
#endif
#if !MOTOR_TEST_CONFIGURED && MOTOR_BENCH_JOG_ENABLED
static void bench_jog_fixture(QrApp *a,uint32_t now,uint8_t action,uint8_t selected)
{
    MotorTransport t={0,motor_send,motor_busy}; unsigned i,j;
    QrApp_Init(a,t,1u,now); a->ui.motor_page=a->ui.detail=1u;
    a->ui.test_action=action; a->ui.selected=selected;
    MotorBus_SetManual(&a->motor,1u,now);
    for(i=0;i<7u;++i) {
        MotorSnapshot *d=&a->motor.nodes[i].data;
        d->valid=7u; d->online=1u; d->status_flags=3u; d->rx_count=1u;
        d->position_ms=d->speed_ms=d->status_ms=d->last_rx_ms=now;
        for(j=0;j<3u;++j) a->motor.nodes[i].poll_due[j]=now+10000u;
    }
}
/* MOTOR_TEST_CONFIGURED=0 keeps POSITION/WHEELS/SYNC gated but allows an
   explicit MANUAL + RUN ONCE single-axis SPEED jog. */
static void test_bench_jog(void)
{
    QrApp a; unsigned before;
    bench_jog_fixture(&a,50000u,1u,0u); before=app_commands[EMM_VELOCITY];
    click(&a,20,184,50000u);
    CHECK(app_commands[EMM_VELOCITY]==before+1u && app_last[0]==1u && app_length==8u &&
        app_last[2]==MOTOR_TEST_DIRECTION && app_last[5]==MOTOR_TEST_ACCELERATION);
    bench_jog_fixture(&a,51000u,2u,4u); before=app_commands[EMM_MOVE];
    click(&a,20,184,51000u);
    CHECK(app_commands[EMM_MOVE]==before && strstr(a.motor.message,"CONFIGURE"));
    bench_jog_fixture(&a,52000u,3u,0u); before=app_commands[EMM_MULTI];
    click(&a,20,184,52000u);
    CHECK(app_commands[EMM_MULTI]==before && strstr(a.motor.message,"CONFIGURE"));
    bench_jog_fixture(&a,53000u,4u,0u); before=app_commands[EMM_SYNC];
    click(&a,20,184,53000u);
    CHECK(app_commands[EMM_SYNC]==before && strstr(a.motor.message,"CONFIGURE"));
    CHECK(!bad_layout);
}
#endif
void FlowTest_Fail(unsigned value);
unsigned FlowTest_Bursts(void);

/* Application fixture uses the real QrApp/MotorBus while simulating four motors. */
static struct {
    uint8_t bytes[128], pending, online, flags[7], reject_enable, drop_velocity;
    int32_t rpm[7];
    /* Persistent STOP retries for absent axes are now intentionally retained. */
    uint8_t code[256], address[256]; uint32_t at[256]; unsigned count, groups;
} app_motor;
static int app_send(void *ctx,const uint8_t *b,uint8_t length)
{
    (void)ctx;
    CHECK(!app_motor.pending && length<=sizeof(app_motor.bytes));
    memcpy(app_motor.bytes,b,length); app_motor.pending=1u;
    if(b[1]==EMM_ENABLE || b[1]==EMM_VELOCITY || b[1]==EMM_STOP) {
        CHECK(app_motor.count<256u);
        app_motor.code[app_motor.count]=b[1]; app_motor.address[app_motor.count]=b[0];
        app_motor.at[app_motor.count++]=tick;
    }
    return 1;
}
static void app_reply(MotorBus *bus,uint32_t now)
{
    uint8_t r[8]={0}, *b=app_motor.bytes, length=4u;
    unsigned i;
    if(!app_motor.pending) return;
    app_motor.pending=0u;
    if (b[1]==EMM_MULTI) {
        ++app_motor.groups;
        for (i=0u;i<4u;++i) {
            const uint8_t *m=b+4u+i*8u;
            app_motor.rpm[i]=((int32_t)m[3]<<8)|m[4];
            if (m[2]) app_motor.rpm[i]=-app_motor.rpm[i];
        }
        if (app_motor.drop_velocity) return;
        r[0]=1u; r[1]=EMM_VELOCITY; r[2]=2u; r[3]=0x6Bu;
        MotorBus_Receive(bus,r,4u,now); return;
    }
    i=b[0]-1u;
    if(!(app_motor.online & (1u<<i))) return;
    r[0]=b[0]; r[1]=b[1]; r[2]=2u;
    if(b[1]==EMM_ENABLE) {
        if(b[0]==app_motor.reject_enable) r[2]=0xE2u;
        else app_motor.flags[i]=(uint8_t)(2u|b[3]);
    } else if(b[1]==EMM_VELOCITY) {
        app_motor.rpm[i]=((int32_t)b[3]<<8)|b[4];
        if(b[2]) app_motor.rpm[i]=-app_motor.rpm[i];
        if(app_motor.drop_velocity) return;
    } else if(b[1]==EMM_STOP) app_motor.rpm[i]=0;
    else if(b[1]==EMM_SPEED) {
        length=6u; r[2]=(uint8_t)(app_motor.rpm[i]<0);
        r[4]=(uint8_t)(r[2] ? -app_motor.rpm[i] : app_motor.rpm[i]);
    } else if(b[1]==EMM_POSITION) { length=8u; r[2]=0u; }
    else if(b[1]==EMM_STATUS) r[2]=app_motor.flags[i];
    r[length-1u]=0x6Bu;
    MotorBus_Receive(bus,r,length,now);
}
static void app_fixture(QrApp *a,uint32_t now)
{
    MotorTransport t={0,app_send,motor_busy};
    memset(&app_motor,0,sizeof(app_motor)); app_motor.online=0x0Fu;
    memset(app_motor.flags,2,sizeof(app_motor.flags)); motor_pump_hook=app_reply;
    touch_down=0u; tick=now; BspQrTrigger_Init();
    QrApp_Init(a,t,1u,now);
    CHECK(a->chassis.data.state==CHASSIS_DISARMED && !a->motor.manual);
}
static void test_chassis_screen(void)
{
    QrApp a; uint32_t now, id;
    CHECK(BspQrUart_Init()); app_fixture(&a,0u);
    for(now=0u;now<8000u;++now) app_step(&a,now);
    CHECK(!app_motor.count && a.chassis.data.state==CHASSIS_DISARMED);
    /* Exercise explicit unconfigured state even when the local vehicle config
       has been filled in by the user. Do not rewrite the production header. */
    a.chassis.config.mechanical_confirmed=a.chassis.config.feedback_confirmed=0u;
    a.chassis.config.wheel_diameter_mm=a.chassis.config.length_mm=a.chassis.config.width_mm=0.0f;
    a.chassis.config.gear_ratio=0.0f; memset(a.chassis.config.motor_sign,0,sizeof(a.chassis.config.motor_sign));
    click(&a,300,245,now); click(&a,20,14,now+2u);
    CHECK(a.ui.chassis_page && strstr(lines[0],"CHASSIS"));
    id=a.chassis.data.request_id;
    click(&a,20,245,now+4u);
    CHECK(a.chassis.data.rejection==CHASSIS_CONFIG && a.chassis.data.request_id==id);
    for(now+=6u;now<10000u;++now) app_step(&a,now);
    CHECK(!app_motor.count && strstr(lines[8],"CONFIG REQUIRED"));
    save_image("chassis_config.pgm");
    click(&a,420,14,now); CHECK(a.ui.flow_page);
    click(&a,300,245,now+2u); CHECK(a.ui.chassis_page && !a.ui.flow_page);
    click(&a,20,14,now+4u); CHECK(a.ui.chassis_detail);
    save_image("chassis_wheels.pgm");
    click(&a,170,245,now+6u);
    for(now+=8u;now<12000u;++now) app_step(&a,now);
    CHECK(app_motor.count && app_motor.code[0]==EMM_STOP);
    CHECK(!a.motor.wheels_pending && !a.chassis.data.sequence_active);
    CHECK(!bad_layout);
    /* Render latched failure beside CURRENT healthy sensor indicators. The
       historical cause must remain readable after motor position feedback has recovered. */
    a.chassis.config.mechanical_confirmed=a.chassis.config.feedback_confirmed=1u;
    a.chassis.config.wheel_diameter_mm=100; a.chassis.config.length_mm=200; a.chassis.config.width_mm=180;
    a.chassis.config.gear_ratio=1;
    { unsigned i; for(i=0u;i<4u;++i) a.chassis.config.motor_sign[i]=1; }
    a.chassis.data.state=CHASSIS_FAULT; a.chassis.data.result=CHASSIS_FAILED;
    a.chassis.data.reason=CHASSIS_ODOMETRY_INVALID; a.chassis.data.stage=1u;
    a.chassis.data.odometry_valid=a.chassis.data.gyro_valid=1u; a.chassis.data.stop_pending=0u;
    a.chassis.data.fault.valid=1u; a.chassis.data.fault.issue=CHASSIS_ODOM_SYNC_TIMEOUT;
    a.chassis.data.fault.wheel=2u; a.chassis.data.fault.sample_age_ms=105u;
    a.ui.chassis_detail=0u; a.ui.redraw=1u;
    finish_ui(&a.ui,&a.qr,&a.motor,0,0,now);
    CHECK(!strcmp(lines[8],"ODOM INVALID: TIME ALIGN") && strstr(lines[7],"AT WHEEL:2"));
    CHECK(!bad_layout); save_image("chassis_motor_fault.pgm");
    puts("PASS: no power-on motion, chassis screen/config/edge/back/global STOP");
}

static void test_flow_ui(void)
{
    QrApp a; MotorTransport t={0,motor_send,motor_busy};
    FlowSnapshot d; uint32_t now; unsigned before=motion_sends, n; uint32_t session;
    QrApp_Init(&a,t,1u,80000u); click(&a,420,14,80000u);
    CHECK(a.ui.flow_page && !a.ui.motor_page && strstr(lines[1],"INIT")); save_image("flow_init.pgm");
    for (now=80002u;now<81000u;++now) app_step(&a,now);
    CHECK(a.flow.data.valid && strstr(lines[1],"READY") && !a.motor.manual && motion_sends==before);
    save_image("flow_ready.pgm");
    inject("156+123+516+231\r",81000u,1); app_step(&a,81000u);
    CHECK(a.qr.data.valid); session=a.qr.data.session;
    touch_x=20; touch_y=245; touch_down=1u;
    for(now=81001u;now<81500u;++now) app_step(&a,now);
    CHECK(a.flow.data.clears==1u && a.flow.data.zero_sequence>0u && a.qr.data.session==session);
    CHECK(motion_sends==before && !a.motor.manual);
    touch_down=0; app_step(&a,81500u);
    click(&a,20,184,81501u); CHECK(motion_sends==before && !a.ui.detail); /* no motor row hit */
    click(&a,420,14,81503u); CHECK(a.ui.flow_detail); save_image("flow_diagnostics.pgm");
    FlowTest_Fail(1u);
    for(now=81505u;now<81700u;++now) app_step(&a,now);
    CHECK(a.flow.data.state==FLOW_ERROR && a.qr.data.valid && motion_sends==before);
    save_image("flow_error.pgm");
    click(&a,300,245,81700u); CHECK(!a.ui.flow_page && !a.ui.motor_page && !strcmp(lines[2],sample));
    FlowTest_Fail(0u);
    for(now=81702u;now<83500u;++now) app_step(&a,now);
    CHECK(a.flow.data.valid && a.flow.data.segments==2u && a.flow.data.gaps && a.qr.data.valid);
    n=FlowTest_Bursts(); click(&a,300,245,83500u); CHECK(a.ui.motor_page);
    click(&a,420,14,83502u); CHECK(a.ui.flow_page && a.ui.motor_page);
    CHECK(QrUi_Touch(&a.ui,20,184,1u)==QR_UI_NONE); (void)QrUi_Touch(&a.ui,20,184,0u);
    for(now=83504u;now<83600u;++now) app_step(&a,now);
    CHECK(FlowTest_Bursts()>n && !a.motor.manual);
    /* Full 64-bit totals and maximum counters fit, including every diagnostic row. */
    Flow_GetSnapshot(&a.flow,83600u,&d);
    d.total_x=INT64_MIN; d.total_y=INT64_MAX;
    d.accepted=d.rejected=d.errors=d.segments=d.gaps=d.missed=UINT32_MAX;
    d.sequence=d.zero_sequence=d.max_process_us=d.max_interval_ms=d.clears=UINT32_MAX;
    a.ui.redraw=1u; finish_ui(&a.ui,&a.qr,&a.motor,&d,0,83600u);
    CHECK(strstr(lines[3],"-9223372036854775808") && !bad_layout); save_image("flow_limits.pgm");
    a.ui.flow_detail=1u; a.ui.redraw=1u; finish_ui(&a.ui,&a.qr,&a.motor,&d,0,83600u); CHECK(!bad_layout);
    click(&a,170,245,83601u); CHECK(a.motor.stop_mask && !a.motor.manual); /* global STOP on FLOW */
    click(&a,300,245,83603u); CHECK(!a.ui.flow_page && a.ui.motor_page);
    CHECK(a.qr.data.valid && a.qr.data.session==session);
    CHECK(!bad_layout);
    printf("FLOW concurrent UI: interval max %lu ms, missed %lu, process max %lu us (simulated)\n",
        (unsigned long)a.flow.data.max_interval_ms,(unsigned long)a.flow.data.missed,(unsigned long)a.flow.data.max_process_us);
}

static void test_gyro_ui(void)
{
    QrApp a; MotorTransport t={0,motor_send,motor_busy}; uint32_t now,seq,session;
    unsigned before=motion_sends,requests,cleared; GyroSnapshot d;
    QrApp_Init(&a,t,1u,110000u); click(&a,340,14,110000u);
    CHECK(a.ui.gyro_page && !a.ui.motor_page && strstr(lines[1],"INIT")); save_image("gyro_init.pgm");
    for(now=110002u;now<111000u;++now) app_step(&a,now);
    CHECK(a.gyro.data.raw_valid && !a.gyro.data.calibrated && strstr(lines[1],"UNCAL"));
    CHECK(strstr(lines[2],"--") && strstr(lines[3],"--")); save_image("gyro_uncal.pgm");
    inject("156+123+516+231\r",111000u,1); app_step(&a,111000u);
    CHECK(a.qr.data.valid); session=a.qr.data.session; cleared=a.flow.data.clears;
    click(&a,108,245,111001u); CHECK(!a.gyro.data.zeros); /* ZERO rejected before CAL */
    touch_x=20; touch_y=245; touch_down=1u;
    for(now=111003u;now<113500u;++now) app_step(&a,now);
    printf("GYRO UI cal: req %lu state %u count %lu gap %lu dt %lu raw %u\n",
        (unsigned long)a.gyro.data.cal_requests,(unsigned)a.gyro.data.calibration,(unsigned long)a.gyro.data.cal_count,
        (unsigned long)a.gyro.data.gaps,(unsigned long)a.gyro.data.max_interval_ms,a.gyro.data.raw_valid);
    CHECK(a.gyro.data.cal_requests==1u && a.gyro.data.calibration==GYRO_CAL_DONE);
    CHECK(a.gyro.data.angle_valid && a.gyro.data.rate_valid && !a.motor.manual);
    CHECK(motion_sends==before && a.flow.data.clears==cleared && a.qr.data.session==session);
    save_image("gyro_ready.pgm"); touch_down=0u; app_step(&a,113500u);
    requests=a.gyro.data.cal_requests; touch_x=108; touch_down=1u;
    for(now=113501u;now<114000u;++now) app_step(&a,now);
    CHECK(a.gyro.data.zeros==1u && a.gyro.data.cal_requests==requests && motion_sends==before);
    touch_down=0u; app_step(&a,114000u);
    click(&a,420,14,114001u); CHECK(a.ui.gyro_detail); save_image("gyro_diagnostics.pgm");
    click(&a,300,245,114003u); CHECK(!a.ui.gyro_page && !a.ui.motor_page && !a.ui.flow_page);
    seq=a.gyro.data.sequence;
    for(now=114005u;now<114500u;++now) app_step(&a,now);
    CHECK(a.gyro.data.sequence>seq && a.qr.data.valid); /* background sampling */
    click(&a,300,245,114500u); CHECK(a.ui.motor_page);
    click(&a,340,14,114502u); CHECK(a.ui.gyro_page && a.ui.motor_page);
    click(&a,20,184,114504u); CHECK(motion_sends==before && !a.ui.detail);
    GyroTest_Fail(1u);
    for(now=114506u;now<114700u;++now) app_step(&a,now);
    CHECK(a.gyro.data.state==GYRO_ERROR && !a.gyro.data.angle_valid && a.qr.data.valid);
    save_image("gyro_error.pgm"); GyroTest_Fail(0u);
    for(now=114700u;now<115400u;++now) app_step(&a,now);
    CHECK(a.gyro.data.raw_valid && !a.gyro.data.calibrated && !a.gyro.data.angle_valid);
    CHECK(motion_sends==before && a.flow.data.clears==cleared && a.qr.data.session==session);
    /* Max counters and invalid numbers fit the actual font, all rows. */
    Gyro_GetSnapshot(&a.gyro,115400u,&d);
    d.sequence=d.interval_ms=d.age_ms=d.max_interval_ms=d.max_poll_ms=d.max_process_us=UINT32_MAX;
    d.missed=d.gaps=d.errors=d.saturations=d.invalid=d.restarts=d.cal_count=UINT32_MAX;
    a.ui.redraw=1u; finish_ui(&a.ui,&a.qr,&a.motor,0,&d,115400u);
    a.ui.gyro_detail=1u; a.ui.redraw=1u; finish_ui(&a.ui,&a.qr,&a.motor,0,&d,115400u);
    CHECK(!bad_layout);
    click(&a,170,245,115401u); CHECK(a.motor.stop_mask); /* global STOP still available */
    click(&a,300,245,115403u); CHECK(!a.ui.gyro_page && a.ui.motor_page);
    CHECK(a.qr.data.valid && a.qr.data.session==session && a.flow.data.clears==cleared);
}

static void review_advance(QrApp *a,unsigned duration)
{
    uint32_t end=tick+duration,at,spent;
    while((int32_t)(tick-end)<0) {
        at=tick+1u; app_step(a,at); spent=tick-at;
        if(spent>lcd_max_step) lcd_max_step=spent;
    }
}
static void review_press(QrApp *a,int32_t x,int32_t y)
{ touch_x=x; touch_y=y; touch_down=1u; review_advance(a,1u); touch_down=0u; review_advance(a,1u); }
static void test_review_ui(void)
{
    QrApp a; MotorTransport t={0,motor_send,motor_busy}; unsigned before,pumps; uint32_t seq;
    tick=1000u; CHECK(BspQrUart_Init()); QrApp_Init(&a,t,1u,tick);
    review_advance(&a,800u); review_press(&a,340,14); review_advance(&a,500u);
    lcd_timed=1u; before=clears; review_press(&a,20,245); review_advance(&a,2800u);
    CHECK(a.gyro.data.calibration==GYRO_CAL_DONE && a.gyro.data.angle_valid);
    CHECK(clears==before); /* CAL/ZERO do not request full-screen clear */
    review_press(&a,108,245); review_advance(&a,100u);
    CHECK(a.gyro.data.zeros==1u && a.gyro.data.angle_valid);
    review_press(&a,420,14); review_advance(&a,700u); CHECK(a.gyro.data.angle_valid);
    review_press(&a,300,245); review_advance(&a,700u); CHECK(a.gyro.data.angle_valid);
    review_press(&a,420,14); review_advance(&a,700u); CHECK(a.ui.flow_page && a.gyro.data.angle_valid);
    review_press(&a,20,245); review_press(&a,420,14); review_advance(&a,700u);
    seq=a.gyro.data.sequence; pumps=motor_pumps;
    /* Cancel several incomplete page jobs; every step still services the app. */
    review_press(&a,340,14); review_press(&a,300,245); review_press(&a,300,245);
    review_press(&a,300,245); review_press(&a,300,245); review_advance(&a,700u);
    CHECK(a.gyro.data.sequence>seq && motor_pumps>pumps && a.gyro.data.angle_valid);
    inject("156+123+516+231\r",tick,1); review_advance(&a,700u); CHECK(a.qr.data.valid);
    CHECK(lcd_max_step<=2u && !bad_layout);
    printf("UI injected: max step=%u ms gyro poll=%lu dt=%lu missed=%lu pumps=%u\n",lcd_max_step,
        (unsigned long)a.gyro.data.max_poll_ms,(unsigned long)a.gyro.data.max_interval_ms,
        (unsigned long)a.gyro.data.missed,motor_pumps-pumps);
    /* A genuinely oversized primitive must still break continuity. */
    lcd_overrun=1u; a.ui.redraw=1u; review_advance(&a,30u);
    CHECK(!a.gyro.data.angle_valid && a.gyro.data.angle_gap);
    lcd_overrun=0u; lcd_timed=0u;
    puts("PASS: timed UI CAL/ZERO/pages/updates/cancellation and true GAP");
}
static void test_review_gate(void)
{
    QrApp a; MotorTransport t={0,motor_send,motor_busy}; MotorNode *n;
    tick=1000u; CHECK(BspQrUart_Init()); QrApp_Init(&a,t,1u,tick);
    review_advance(&a,800u); review_press(&a,340,14); n=&a.motor.nodes[6];
    n->data.valid=MOTOR_VALID_SPEED; n->data.speed_rpm=10; n->data.speed_ms=tick;
    review_press(&a,20,245); CHECK(!strcmp(Gyro_CalName(a.gyro.data.calibration),"MOTOR ACTIVE"));
    n->data.speed_ms=tick-10000u;
    review_press(&a,20,245); CHECK(!strcmp(Gyro_CalName(a.gyro.data.calibration),"MOTOR UNKNOWN"));
    n->data.valid=0u; n->action=MOTOR_UNKNOWN;
    review_press(&a,20,245); CHECK(!strcmp(Gyro_CalName(a.gyro.data.calibration),"MOTOR UNKNOWN"));
    n->action=MOTOR_IDLE; MotorBus_Stop(&a.motor,0x40u,tick);
    review_press(&a,20,245); CHECK(!strcmp(Gyro_CalName(a.gyro.data.calibration),"STOP UNCONF"));
    /* A new bench session, never connected/commanded nodes, remains usable. */
    QrApp_Init(&a,t,1u,tick); review_advance(&a,800u); review_press(&a,340,14);
    review_press(&a,20,245); review_advance(&a,2300u); CHECK(a.gyro.data.calibration==GYRO_CAL_DONE);
    puts("PASS: calibration motion/unknown/stop/absent bench gates");
}
static void test_review_parallel(void)
{
    QrApp a; uint32_t start,next_scan,stop_at,latency; unsigned phase=0u,i,before;
    CHECK(BspQrUart_Init()); app_fixture(&a,200000u); app_motor.online=MOTOR_ALL;
    lcd_timed=trigger_measure=1u; trigger_have=0u; trigger_max_jitter=0u; lcd_max_step=0u;
    start=tick; next_scan=tick;
    while(tick-start<15000u) {
        uint32_t elapsed=tick-start;
        if((int32_t)(tick-next_scan)>=0) { inject("156+123+516+231\r",tick,1); next_scan+=100u; }
        if(phase==0u && elapsed>=300u) { review_press(&a,340,14); ++phase; }
        if(phase==1u && elapsed>=900u) { review_press(&a,420,14); ++phase; }
        if(phase==2u && elapsed>=1500u) { review_press(&a,300,245); ++phase; }
        if(phase==3u && elapsed>=2100u) { review_press(&a,420,14); ++phase; }
        if(phase==4u && elapsed>=2700u) { review_press(&a,20,245); ++phase; }
        if(phase==5u && elapsed>=3300u) { review_press(&a,300,245); ++phase; }
        if(phase==6u && elapsed>=10000u) {
            CHECK(a.chassis.data.state==CHASSIS_DISARMED && !app_motor.count);
            review_press(&a,340,14); review_press(&a,20,245); ++phase;
        }
        if(phase==7u && elapsed>=13000u) {
            CHECK(a.gyro.data.calibration==GYRO_CAL_DONE && a.gyro.data.angle_valid);
            review_press(&a,108,245); review_press(&a,420,14); ++phase;
        }
        review_advance(&a,1u);
    }
    CHECK(phase==8u && a.gyro.data.angle_valid && a.qr.data.duplicates>140u && a.flow.data.valid);
    for(i=0u;i<7u;++i) CHECK(a.motor.nodes[i].data.online);
    before=app_motor.count; stop_at=tick;
    review_press(&a,170,245); review_advance(&a,50u);
    CHECK(app_motor.count>before && app_motor.code[before]==EMM_STOP);
    latency=app_motor.at[before]-stop_at; CHECK(latency<=15u);
    CHECK(lcd_max_step<=2u && trigger_max_jitter<=3u);
    printf("TIMED PARALLEL (injected): UI %u ms poll %lu ms dt %lu ms missed %lu STOP %lu ms trigger jitter %lu ms QR %lu\n",
        lcd_max_step,(unsigned long)a.gyro.data.max_poll_ms,(unsigned long)a.gyro.data.max_interval_ms,
        (unsigned long)a.gyro.data.missed,(unsigned long)latency,(unsigned long)trigger_max_jitter,
        (unsigned long)a.qr.data.duplicates);
    lcd_timed=trigger_measure=0u;
}
static void test_chassis_no_flow(void)
{
    QrApp a; ChassisConfig cfg; unsigned i; uint32_t flow_clear_before;
    CHECK(BspQrUart_Init()); app_fixture(&a,400000u);
    review_advance(&a,2500u); review_press(&a,340,14); review_press(&a,20,245);
    review_advance(&a,2300u); CHECK(a.gyro.data.angle_valid);
    Chassis_DefaultConfig(&cfg); cfg.mechanical_confirmed=cfg.feedback_confirmed=1u;
    cfg.wheel_diameter_mm=100; cfg.length_mm=200; cfg.width_mm=180; cfg.gear_ratio=1;
    for(i=0u;i<4u;++i) cfg.motor_sign[i]=1;
    Chassis_Init(&a.chassis,&a.motor,&cfg,tick); review_advance(&a,1000u);
    FlowTest_Fail(1u); review_advance(&a,1000u); CHECK(!a.flow.data.valid);
    CHECK(Chassis_StartAcceptance(&a.chassis,tick)==CHASSIS_ACCEPTED);
    review_advance(&a,700u);
    CHECK(a.chassis.data.state==CHASSIS_MOVING && a.chassis.data.sequence_active);
    flow_clear_before=a.flow.data.clears; Flow_ResetTotals(&a.flow); review_advance(&a,700u);
    CHECK(a.flow.data.clears==flow_clear_before+1u && a.chassis.data.state==CHASSIS_MOVING);
    CHECK(a.chassis.data.reason==CHASSIS_OK && a.chassis.data.odometry_valid);
    Chassis_Stop(&a.chassis,tick); review_advance(&a,2000u);
    CHECK(a.chassis.data.result==CHASSIS_CANCELLED && !a.motor.wheels_pending);
    FlowTest_Fail(0u);
    puts("PASS: actual QrApp/flow failure before START and CLEAR during move do not affect chassis; STOP preserved");
}
static void test_chassis_active(void)
{
    QrApp a; ChassisConfig cfg; uint32_t start,next_command,next_scan,cleared,zeros,requests;
    unsigned phase=0u,i,groups;
    CHECK(BspQrUart_Init()); app_fixture(&a,300000u);
    review_advance(&a,2500u); review_press(&a,340,14); review_press(&a,20,245);
    review_advance(&a,2300u); CHECK(a.gyro.data.angle_valid);
    Chassis_DefaultConfig(&cfg);
    cfg.mechanical_confirmed=cfg.feedback_confirmed=1u; /* synthetic fixture only */
    cfg.wheel_diameter_mm=100; cfg.length_mm=200; cfg.width_mm=180; cfg.gear_ratio=1;
    for(i=0u;i<4u;++i) cfg.motor_sign[i]=1;
    Chassis_Init(&a.chassis,&a.motor,&cfg,tick); review_advance(&a,20u);
    CHECK(Chassis_Prepare(&a.chassis,tick)==CHASSIS_ACCEPTED);
    review_advance(&a,1000u); CHECK(a.chassis.data.state==CHASSIS_READY);
    review_press(&a,300,245); review_press(&a,300,245); review_press(&a,20,14);
    CHECK(a.ui.chassis_page); a.ui.redraw=1u;
    cleared=a.flow.data.clears; zeros=a.gyro.data.zeros; requests=a.gyro.data.cal_requests;
    lcd_timed=trigger_measure=1u; trigger_have=0u; trigger_max_jitter=0u; lcd_max_step=0u;
    start=next_command=next_scan=tick;
    while(tick-start<2200u) {
        uint32_t elapsed=tick-start;
        if((int32_t)(tick-next_command)>=0) {
            CHECK(Chassis_SetBodyVelocity(&a.chassis,50,10,5,tick)==CHASSIS_ACCEPTED); next_command=tick+50u;
        }
        if((int32_t)(tick-next_scan)>=0) { inject("156+123+516+231\r",tick,1); next_scan=tick+100u; }
        if(phase==0u && elapsed>=100u) { review_press(&a,20,245); CHECK(a.chassis.data.rejection==CHASSIS_AXIS_BUSY); ++phase; }
        if(phase==1u && elapsed>=300u) { review_press(&a,300,245); review_press(&a,20,245); CHECK(!strcmp(a.motor.message,"CHASSIS BUSY")); ++phase; }
        if(phase==2u && elapsed>=500u) { review_press(&a,20,32); review_press(&a,20,184); CHECK(!strcmp(a.motor.message,"CHASSIS BUSY")); ++phase; }
        if(phase==3u && elapsed>=700u) { review_press(&a,420,14); review_press(&a,20,245); ++phase; }
        if(phase==4u && elapsed>=900u) { review_press(&a,340,14); review_press(&a,20,245); review_press(&a,108,245); ++phase; }
        if(phase==5u && elapsed>=1200u) { review_press(&a,300,245); review_press(&a,300,245); review_press(&a,300,202); review_press(&a,20,14); ++phase; }
        review_advance(&a,1u);
    }
    CHECK(phase==6u && a.chassis.data.state==CHASSIS_VELOCITY && app_motor.groups>=15u);
    CHECK(a.flow.data.clears==cleared+1u && a.gyro.data.zeros==zeros && a.gyro.data.cal_requests==requests);
    CHECK(a.gyro.data.angle_valid && a.qr.data.duplicates>=18u && !bad_layout);
    for(i=0u;i<app_motor.count;++i) CHECK(app_motor.address[i]<=4u);
    review_press(&a,170,245); review_advance(&a,1000u); groups=app_motor.groups;
    review_advance(&a,1000u); CHECK(groups==app_motor.groups && !a.chassis.data.sequence_active);
    CHECK(a.chassis.data.result==CHASSIS_CANCELLED);
    CHECK(lcd_max_step<=2u && a.gyro.data.max_poll_ms<10u && a.gyro.data.max_interval_ms<=14u);
    printf("ACTIVE CHASSIS (injected): groups %u UI %u ms gyro poll/dt %lu/%lu ms QR %lu; arbitration/STOP verified\n",
        app_motor.groups,lcd_max_step,(unsigned long)a.gyro.data.max_poll_ms,
        (unsigned long)a.gyro.data.max_interval_ms,(unsigned long)a.qr.data.duplicates);
    lcd_timed=trigger_measure=0u;
}
int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"chassis-active")) { test_chassis_active(); return 0; }
    if(argc>1 && !strcmp(argv[1],"chassis-no-flow")) { test_chassis_no_flow(); return 0; }
    if(argc>1 && !strcmp(argv[1],"review-parallel")) { test_review_parallel(); return 0; }
    if(argc>1 && !strcmp(argv[1],"review-ui")) { test_review_ui(); return 0; }
    if(argc>1 && !strcmp(argv[1],"review-gate")) { test_review_gate(); return 0; }
    if(argc>1 && !strcmp(argv[1],"chassis-screen")) { test_chassis_screen(); return 0; }
    if(argc>1) {
        QrService s;
        if(!strcmp(argv[1],"init")) init_fail=1;
        if(!strcmp(argv[1],"dma")) dma_init_fail=1;
        if(!strcmp(argv[1],"receive")) receive_fail=1;
        if(!strcmp(argv[1],"start")) dma_start_fail=1;
        CHECK(!BspQrUart_Init()); QrService_Init(&s,0u,0u); BspQrUart_Pump(&s,0u);
        CHECK(!s.data.valid && !s.data.uart_ready && !strcmp(QrService_StateName(&s),"UART ERROR"));
        printf("PASS: QR startup failure (%s)\n",argv[1]); return 0;
    }
    test_validation(); test_stream_sessions(); test_bsp(); test_app_ui(); test_seven_ui(); test_flow_ui(); test_gyro_ui();
#if MOTOR_TEST_CONFIGURED
    test_configured_actions();
#endif
#if !MOTOR_TEST_CONFIGURED && MOTOR_BENCH_JOG_ENABLED
    test_bench_jog();
#endif
    printf("PASS: %u QR checks (parser, sessions, real BSP/callbacks, app, UI)\n",checks);
    return 0;
}

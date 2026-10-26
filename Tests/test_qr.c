#include "app/qr_app.h"
#include "app/app_runtime.h"
#include "usart/bsp_qr_uart.h"
#include "usart/bsp_qr_trigger.h"
#include "usart/bsp_motor_uart.h"
#include "motor/motor_test_config.h"
#include "gyro/gyro_config.h"
#include "lcd/bsp_lcd.h"
#include "lcd/lcd_orientation.h"
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
GPIO_TypeDef fake_gpiog;
static uint8_t limit_levels[2];
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ CHECK(port==GPIOG && (pin==GPIO_PIN_2 || pin==GPIO_PIN_3)); return limit_levels[pin==GPIO_PIN_2 ? 0u : 1u]; }
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
    if (port==GPIOG) {
        CHECK(g->Pin==(GPIO_PIN_2|GPIO_PIN_3) && g->Mode==GPIO_MODE_INPUT && g->Pull==GPIO_PULLUP); return;
    }
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
/* Same monotonic clock as HAL/gyro. Costs are injected, never board timings. */
static unsigned lcd_timed, lcd_overrun, lcd_max_step;
static uint32_t lcd_clear_ms=12u, lcd_fill_ms=1u, lcd_text_ms=1u, lcd_setup_ms;
void LCD_SetFont(sFONT *f) { if(lcd_timed) tick+=lcd_setup_ms; font=f; }
void LCD_SetBackColor(uint32_t c) { if(lcd_timed) tick+=lcd_setup_ms; background=c; }
void LCD_SetTextColor(uint32_t c) { if(lcd_timed) tick+=lcd_setup_ms; foreground=c; }
void LCD_Clear(uint32_t c) { if(lcd_timed) tick+=lcd_clear_ms; ++clears; memset(pixels,c?255:0,sizeof(pixels)); }
void LCD_FillRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h)
{
    unsigned i,j;
    if(lcd_timed) tick+=lcd_overrun ? 12u : (lcd_fill_ms+((unsigned)w*h)/2000u);
    if((unsigned)x+w>480u || (unsigned)y+h>272u) { ++bad_layout; return; }
    if (!Lcd_OrientRect(&x,&y,&w,&h,480,272)) return;
    for(i=y;i<(unsigned)y+h;++i) for(j=x;j<(unsigned)x+w;++j) pixels[i][j]=foreground?255u:0u;
}
void LCD_DrawPixel(uint16_t x,uint16_t y,uint32_t color)
{
    uint16_t w=1u,h=1u;
    if (x>=480u || y>=272u) { ++bad_layout; return; }
    if (Lcd_OrientRect(&x,&y,&w,&h,480,272)) pixels[y][x]=color ? 255u : 0u;
}
void LCD_DisplayStringAt(uint16_t x,uint16_t y,uint8_t *text,unsigned mode)
{
    unsigned i,j,k,byte_width,offset;
    uint32_t bits;
    (void)mode; ++draws;
    if(lcd_timed) tick+=lcd_text_ms+(unsigned)strlen((char *)text)/2u;
    if(!font || (unsigned)x+strlen((char *)text)*font->Width>480u || (unsigned)y+font->Height>272u) {
        ++bad_layout; return;
    }
    byte_width=(font->Width+7u)/8u;
    for(k=0;text[k];++k) for(i=0;i<font->Height;++i) {
        /* bsp_lcd.c DrawChar first paints 3 background rows, then glyph rows 0..H-4. */
        offset=((unsigned)text[k]-32u)*font->Height*byte_width+(i<3u?0u:i-3u)*byte_width;
        bits=0u;
        if(i>=3u) for(j=0;j<byte_width;++j) bits=(bits<<8)|font->table[offset+j];
        for(j=0;j<font->Width;++j) LCD_DrawPixel((uint16_t)(x+k*font->Width+j),(uint16_t)(y+i),
            (bits & (1u << (byte_width*8u-1u-j))) ? foreground : background);
    }
}
static void save_image(const char *name)
{
    FILE *f=fopen(name,"wb"); CHECK(f!=0);
    unsigned x,y;
    fprintf(f,"P5\n480 272\n255\n");
    for(y=0u;y<272u;++y) for(x=0u;x<480u;++x) fputc(pixels[271u-y][479u-x],f);
    fclose(f);
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

static int app_busy(void *ctx) { (void)ctx; return 0; }
static int32_t touch_x,touch_y;
static uint8_t touch_down;
static unsigned touch_polls;
void GTP_TouchProcess(void) { ++touch_polls; }
static void finish_ui(QrUi *u,const QrService *q,const MotorBus *m,const FlowSnapshot *f,const GyroSnapshot *g,uint32_t now)
{
    unsigned budget=4096u;
    do { QrUi_Process(u,q,m,f,g,now); } while(u->drawing && --budget);
    CHECK(budget && u->count<=UI_WIDGETS);
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
static void advance(QrApp *a,unsigned duration)
{
    uint32_t end=tick+duration,at,spent;
    while((int32_t)(tick-end)<0) {
        at=tick+1u; app_step(a,at); spent=tick-at;
        if(spent>lcd_max_step) lcd_max_step=spent;
    }
}
/* Test clicks use the mounted, human-visible coordinate system. */
static void press(QrApp *a,int32_t x,int32_t y)
{
    touch_x=479-x; touch_y=271-y; touch_down=1u; advance(a,1u);
    touch_down=0u; advance(a,1u);
}
static int contains(const QrUi *u,const char *text)
{
    unsigned i;
    for(i=0u;i<u->count;++i) if(strstr(u->target[i].text,text)) return 1;
    return 0;
}
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
    MotorTransport t={0,app_send,app_busy};
    memset(&app_motor,0,sizeof(app_motor)); app_motor.online=0x0Fu;
    memset(app_motor.flags,2,sizeof(app_motor.flags)); motor_pump_hook=app_reply;
    touch_down=0u; tick=now; BspQrTrigger_Init();
    QrApp_Init(a,t,1u,now);
    CHECK(a->chassis.data.state==CHASSIS_DISARMED && !a->motor.manual);
}
static void test_orientation(void)
{
    uint16_t x,y,w,h; unsigned i;
    for(i=0u;i<4u;++i) {
        x=(uint16_t)(i&1u ? 479u : 0u); y=(uint16_t)(i&2u ? 271u : 0u); w=h=1u;
        CHECK(Lcd_OrientRect(&x,&y,&w,&h,480,272));
        CHECK(x==(i&1u ? 0u : 479u) && y==(i&2u ? 0u : 271u));
        CHECK(Lcd_OrientRect(&x,&y,&w,&h,480,272));
        CHECK(x==(i&1u ? 479u : 0u) && y==(i&2u ? 271u : 0u));
    }
    x=8; y=232; w=88; h=32;
    CHECK(Lcd_OrientRect(&x,&y,&w,&h,480,272) && x==384u && y==8u);
    x=478; y=270; w=h=100;
    CHECK(Lcd_OrientRect(&x,&y,&w,&h,480,272) && !x && !y && w==2u && h==2u);
    x=480; CHECK(!Lcd_OrientRect(&x,&y,&w,&h,480,272));
}
static void validate_widgets(const QrUi *u)
{
    unsigned i;
    CHECK(u->count<=UI_WIDGETS);
    for(i=0u;i<u->count;++i) {
        const UiWidget *v=&u->target[i]; const unsigned char *p=(const unsigned char *)v->text;
        unsigned width=0u,lines_count=1u;
        CHECK(v->x+v->width<=480u && v->y+v->height<=272u);
        while(*p) {
            if(*p=='\n') { CHECK(width<=v->width); width=0u; ++lines_count; ++p; }
            else if(*p<128u) { width+=v->large ? Font24.Width : Font16.Width; ++p; }
            else { CHECK(p[1] && p[2]); width+=20u; p+=3; }
        }
        if(width>v->width) fprintf(stderr,"OVERFLOW: %s width %u / %u\n",v->text,width,v->width);
        CHECK(width<=v->width);
        CHECK((lines_count-1u)*24u+(v->large ? 24u : 20u)<=v->height);
    }
}
static void test_dashboard(void)
{
    QrApp a; unsigned i,before,pumps; uint32_t session;
    CHECK(BspQrUart_Init()); app_fixture(&a,100000u); app_motor.online=MOTOR_ALL;
    advance(&a,2500u); CHECK(a.ui.page==UI_HOME && !a.ui.more && !app_motor.count);
    CHECK(contains(&a.ui,"\347\255\211\345\276\205\346\234\272\346\236\204\345\260\261\347\273\252") && contains(&a.ui,"7/7 ONLINE"));
    CHECK(!a.motor.manual); validate_widgets(&a.ui); save_image("dashboard_home.pgm");
    session=a.qr.data.session; pumps=motor_pumps;
    inject("156+123+516+231\r",tick,1); advance(&a,200u);
    CHECK(a.qr.data.valid && contains(&a.ui,sample));
    press(&a,32,120); CHECK(a.ui.page==UI_MOTOR && !a.ui.more);
    before=app_motor.count;
    for(i=0u;i<7u;++i) {
        press(&a,32,50+(int)i*24); CHECK(a.ui.more && a.ui.selected==i);
        press(&a,200,210); CHECK(a.ui.selected==(i+1u)%7u);
        press(&a,32,210); CHECK(a.ui.selected==i);
        press(&a,420,18); CHECK(!a.ui.more);
    }
    /* Former manual/run/cycle locations now only select telemetry. */
    for(i=0u;i<8u;++i) { press(&a,20,140); press(&a,20,184); press(&a,420,18); }
    CHECK(app_motor.count==before && !a.motor.manual);
    for(i=0u;i<6u;++i) {
        static const uint8_t pages[]={UI_MOTOR,UI_GYRO,UI_QR,UI_FLOW,UI_VISION,UI_MECHANISM};
        press(&a,40,248); press(&a,40+(int)(i%3u)*158,120+(int)(i/3u)*62);
        CHECK(a.ui.page==pages[i]); validate_widgets(&a.ui);
        press(&a,420,18); CHECK(a.ui.more); validate_widgets(&a.ui);
        press(&a,420,18); CHECK(!a.ui.more);
    }
    press(&a,140,248); CHECK(a.ui.page==UI_CHASSIS && !a.ui.more);
    press(&a,240,248); CHECK(a.ui.page==UI_MECHANISM);
    press(&a,330,248); CHECK(a.ui.page==UI_LIMITS);
    limit_levels[0]=1u; advance(&a,100u);
    CHECK(contains(&a.ui,"\346\214\211\344\270\213 / \346\226\255\350\267\257") && contains(&a.ui,"\351\207\212\346\224\276"));
    press(&a,420,18); CHECK(contains(&a.ui,"NC"));
    limit_levels[0]=0u; advance(&a,100u);
    CHECK(a.qr.data.session==session && a.qr.data.valid && motor_pumps>pumps);
    /* Invalid press never causes a new action by sliding into a button. */
    touch_x=-1; touch_y=0; touch_down=1; advance(&a,1u);
    touch_x=479-420; touch_y=271-248; advance(&a,1u);
    CHECK(!a.motor.stop_mask); touch_down=0; advance(&a,1u);
    press(&a,420,248); CHECK(a.motor.nodes[6].stop_unconfirmed);
    advance(&a,2000u); CHECK(!a.motor.wheels_pending && !a.chassis.data.sequence_active);
    CHECK(!bad_layout);
    puts("PASS: dashboard navigation, MORE, all 7 read-only motors, limits, locked QR and rotated STOP");
}
static void render_pages(void)
{
    QrApp a; FlowSnapshot f={0}; GyroSnapshot g={0}; unsigned page,more,i;
    static const char * const names[]={"home","motor","gyro","qr","flow","vision","chassis","arm","limits"};
    app_fixture(&a,200000u); receive(&a.qr,"156+123+516+231\r",tick);
    a.mechanism.auto_pending=0u;
    for(i=0u;i<7u;++i) {
        MotorSnapshot *d=&a.motor.nodes[i].data;
        d->online=1u; d->valid=7u; d->rx_count=3u; d->status_flags=3u;
        d->position_ms=d->speed_ms=d->status_ms=tick;
    }
    g.state=GYRO_RUNNING; g.calibrated=g.rate_valid=g.angle_valid=g.raw_valid=g.have_sample=1u;
    g.axis=2; g.sign=1; g.angle_deg=-12.34; g.raw_dps=15.86f; g.bias_dps=15.86f;
    f.state=FLOW_ERROR; f.error=FLOW_SPI; /* failed sensor is visible, never used for chassis */
    a.chassis.odometry.valid=1; a.chassis.odometry.x=123.45; a.chassis.odometry.y=-67.89;
    a.chassis.odometry.angle=-12.34; a.chassis.odometry.sample_ms=tick;
    a.mechanism.data.reference[0]=a.mechanism.data.reference[1]=MECH_HOMED;
    a.mechanism.data.reference[2]=MECH_HOMED; a.mechanism.data.trim_ready=1u;
    a.mechanism.data.position[0]=45.25; a.mechanism.data.position[1]=12.5; a.mechanism.data.position[2]=-1.25;
    a.mechanism.data.limit[0].valid=a.mechanism.data.limit[1].valid=1u;
    a.mechanism.data.limit[0].raw=a.mechanism.data.limit[0].stable=1u;
    for(page=0u;page<=UI_LIMITS;++page) for(more=0u;more<2u;++more) {
        char name[64]; a.ui.page=(uint8_t)page; a.ui.more=(uint8_t)more; a.ui.redraw=1;
        finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); validate_widgets(&a.ui);
        snprintf(name,sizeof(name),"ui_%s%s.pgm",names[page],more ? "_more" : ""); save_image(name);
        if(page==UI_CHASSIS && !more) CHECK(contains(&a.ui,"123.45") && contains(&a.ui,"-67.89"));
        if(page==UI_MECHANISM && !more) CHECK(contains(&a.ui,"45.25") && contains(&a.ui,"-1.25"));
        /* White button interiors and black margins are rotated in framebuffer. */
        CHECK(pixels[271-232][479-8]==255u && pixels[0][0]==0u);
    }
    a.ui.page=UI_HOME; a.ui.more=0; a.ui.redraw=1;
    a.mechanism.owned=1; a.mechanism.data.homing=1;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); CHECK(contains(&a.ui,"\346\234\272\346\236\204\345\233\236\351\233\266\344\270\255"));
    a.mechanism.data.stop_pending=1; a.ui.redraw=1;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); CHECK(contains(&a.ui,"\345\201\234\350\275\246\347\241\256\350\256\244\344\270\255"));
    a.mechanism.data.stop_pending=0; a.mechanism.owned=0; a.chassis.data.state=CHASSIS_FAULT; a.ui.redraw=1;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); CHECK(contains(&a.ui,"\346\225\205\351\232\234\345\276\205\345\244\204\347\220\206"));
    a.ui.page=UI_CHASSIS; a.chassis.odometry.valid=0; a.ui.redraw=1;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); CHECK(!contains(&a.ui,"123.45") && contains(&a.ui,"INVALID"));
    a.ui.page=UI_MECHANISM; a.mechanism.data.reference[0]=MECH_UNREFERENCED; a.ui.redraw=1;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); CHECK(!contains(&a.ui,"45.25"));
    a.ui.page=UI_MOTOR; a.ui.more=1; a.ui.redraw=1;
    a.motor.nodes[0].data.position_counts=INT64_MIN;
    a.motor.nodes[0].data.rx_count=a.motor.nodes[0].data.timeouts=UINT32_MAX;
    finish_ui(&a.ui,&a.qr,&a.motor,&f,&g,tick); validate_widgets(&a.ui);
    CHECK(!bad_layout); puts("PASS: rendered all 18 page variants, bounds, extreme counts, phase priority and invalid coordinates");
}
static void gyro_ready(QrApp *a)
{
    advance(a,2500u); press(a,40,248); press(a,200,120); CHECK(a->ui.page==UI_GYRO);
#if !GYRO_TEMP_FIXED_BIAS_ENABLED
    press(a,40,210); advance(a,2400u);
#endif
    CHECK(a->gyro.data.angle_valid);
}
static void test_actions(void)
{
    QrApp a; uint32_t request,session,zeros,clears_before; unsigned i,before;
    CHECK(BspQrUart_Init()); app_fixture(&a,300000u); app_motor.online=MOTOR_ALL; gyro_ready(&a);
    zeros=a.gyro.data.zeros; press(&a,200,210); CHECK(a.gyro.data.zeros==zeros+1u);
    {
        MotorNode *n=&a.motor.nodes[6];
        for(i=0u;i<3u;++i) n->poll_due[i]=tick+10000u;
        n->data.speed_rpm=10; n->data.speed_ms=tick;
        press(&a,40,210); CHECK(a.gyro.data.calibration==GYRO_CAL_MOTOR);
        n->data.speed_rpm=0; n->data.speed_ms=tick-10000u;
        press(&a,40,210); CHECK(a.gyro.data.calibration==GYRO_CAL_UNKNOWN);
        n->data.speed_ms=tick; n->data.status_ms=tick;
        MotorBus_Stop(&a.motor,0x40u,tick);
        press(&a,40,210); CHECK(a.gyro.data.calibration==GYRO_CAL_STOP);
        for(i=0u;i<3u;++i) n->poll_due[i]=tick;
        advance(&a,2000u);
    }
    press(&a,40,248); press(&a,350,120); session=a.qr.data.session;
    touch_x=479-40; touch_y=271-210; touch_down=1; advance(&a,30u);
    CHECK(a.qr.data.session==session+1u); touch_down=0; advance(&a,1u);
    press(&a,40,248); press(&a,40,180); clears_before=a.flow.data.clears;
    press(&a,40,210); CHECK(a.flow.data.clears==clears_before+1u);
    press(&a,240,248); request=a.mechanism.data.request_id;
    press(&a,40,210); CHECK(a.mechanism.data.request_id==request); /* overview has no motion controls */
    press(&a,420,18); press(&a,40,210); CHECK(a.mechanism.data.rejection==MECH_CONFIG);
    press(&a,200,210); CHECK(a.mechanism.data.request_id==request);
    press(&a,140,248); press(&a,420,18); a.chassis.config.mechanical_confirmed=0;
    request=a.chassis.data.request_id; before=app_motor.count;
    press(&a,40,210); CHECK(a.chassis.data.rejection==CHASSIS_CONFIG && a.chassis.data.request_id==request);
    CHECK(app_motor.count==before);
    a.chassis.config.mechanical_confirmed=1u;
    advance(&a,1000u); press(&a,40,210); advance(&a,700u);
    CHECK(a.chassis.data.sequence_active && a.chassis.data.state==CHASSIS_MOVING);
    request=a.chassis.data.request_id;
    press(&a,40,210); CHECK(a.chassis.data.request_id==request); /* no restart */
    press(&a,40,248); press(&a,200,120); zeros=a.gyro.data.zeros;
    press(&a,200,210); CHECK(a.gyro.data.zeros==zeros && a.chassis.data.rejection==CHASSIS_AXIS_BUSY);
    /* Every module page exposes exactly the same global STOP hit area. */
    press(&a,420,248); advance(&a,1800u);
    CHECK(a.chassis.data.result==CHASSIS_CANCELLED && !a.motor.wheels_pending);
    for(i=0u;i<7u;++i) CHECK(!a.motor.nodes[i].stop_unconfirmed);
    CHECK(!bad_layout); puts("PASS: action edges, config gates, chassis START deduplication, gyro arbitration and seven-axis STOP");
}
static void test_timing(void)
{
    QrApp a; uint32_t start,next_scan,seq,stop_at,latency; unsigned i,before,pumps;
    CHECK(BspQrUart_Init()); app_fixture(&a,400000u); app_motor.online=MOTOR_ALL; gyro_ready(&a);
    lcd_timed=trigger_measure=1u; trigger_have=0u; trigger_max_jitter=lcd_max_step=0u;
    /* Cancel incomplete draws repeatedly, then refresh changing data. */
    seq=a.gyro.data.sequence; pumps=motor_pumps; start=next_scan=tick;
    for(i=0u;i<9u;++i) {
        a.ui.page=(uint8_t)i; a.ui.redraw=1u; advance(&a,25u);
    }
    while(tick-start<6000u) {
        if((int32_t)(tick-next_scan)>=0) { inject("156+123+516+231\r",tick,1); next_scan+=100u; }
        advance(&a,1u);
    }
    CHECK(a.gyro.data.sequence>seq && motor_pumps>pumps && a.gyro.data.angle_valid && a.qr.data.duplicates>50u);
    for(i=0u;i<7u;++i) CHECK(a.motor.nodes[i].data.online);
    /* STOP remains live even while a brand-new page is still being drawn. */
    a.ui.page=UI_HOME; a.ui.redraw=1u; before=app_motor.count; stop_at=tick;
    press(&a,420,248); advance(&a,40u);
    CHECK(app_motor.count>before && app_motor.code[before]==EMM_STOP);
    latency=app_motor.at[before]-stop_at;
    CHECK(latency<=15u && lcd_max_step<=2u && trigger_max_jitter<=3u && !bad_layout);
    printf("TIMED UI (injected): max %u ms, STOP %lu ms, gyro poll %lu ms, trigger jitter %lu ms\n",
        lcd_max_step,(unsigned long)latency,(unsigned long)a.gyro.data.max_poll_ms,(unsigned long)trigger_max_jitter);
    lcd_overrun=1; a.ui.redraw=1; advance(&a,30u); CHECK(!a.gyro.data.angle_valid && a.gyro.data.angle_gap);
    lcd_overrun=lcd_timed=trigger_measure=0u;
}
#if MECHANISM_TRIAL_RUN
static void test_trial_boot(void)
{
    QrApp a; uint32_t request,start;
    CHECK(BspQrUart_Init()); app_fixture(&a,500000u); app_motor.online=MOTOR_ALL;
    CHECK(a.ui.page==UI_HOME); start=tick;
    /* Observe command start independently of the current length scale,
       search bound and RPM. Synthetic feedback is not physical calibration. */
    while (tick-start<4000u && !a.motor.nodes[5].motion) advance(&a,1u);
    CHECK(a.mechanism.data.axis==MECH_LIFT && a.mechanism.data.state==MECH_FAST_SEEK);
    CHECK(a.motor.nodes[5].motion && contains(&a.ui,"TRIAL") && contains(&a.ui,"\346\234\272\346\236\204\345\233\236\351\233\266\344\270\255"));
    request=a.mechanism.data.request_id;
    press(&a,240,248); press(&a,420,18); press(&a,40,210);
    CHECK(a.mechanism.data.request_id==request && a.mechanism.data.rejection==MECH_AXIS_BUSY);
    press(&a,420,248); advance(&a,1500u);
    CHECK(a.mechanism.data.result==MECH_CANCELLED && !a.mechanism.auto_pending && !a.mechanism.owned);
    CHECK(!bad_layout); puts("PASS: shipped TRIAL boots to dashboard, auto-homes lift, busy HOME rejected, STOP cancels");
}
#endif
int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"render")) { render_pages(); return 0; }
    if(argc>1 && !strcmp(argv[1],"timing")) { test_timing(); return 0; }
    if(argc>1 && !strcmp(argv[1],"actions")) { test_actions(); return 0; }
#if MECHANISM_TRIAL_RUN
    if(argc>1 && !strcmp(argv[1],"trial-boot")) { test_trial_boot(); return 0; }
#endif
    if(argc>1) {
        QrService s;
        if(!strcmp(argv[1],"init")) init_fail=1;
        else if(!strcmp(argv[1],"dma")) dma_init_fail=1;
        else if(!strcmp(argv[1],"receive")) receive_fail=1;
        else if(!strcmp(argv[1],"start")) dma_start_fail=1;
        else { fprintf(stderr,"Unknown test: %s\n",argv[1]); return 1; }
        CHECK(!BspQrUart_Init()); QrService_Init(&s,0u,0u); BspQrUart_Pump(&s,0u);
        CHECK(!s.data.valid && !s.data.uart_ready); return 0;
    }
    test_validation(); test_stream_sessions(); test_bsp(); test_orientation(); test_dashboard(); render_pages();
    printf("PASS: %u QR/dashboard checks\n",checks); return 0;
}

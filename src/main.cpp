#include <Arduino.h>
#include <usbd_cdc_if.h>

uint32_t configAdcContinuo(uint32_t fs);

HardwareTimer tim1(TIM1);

static volatile uint32_t fs=200'000;

void setup() {
  Serial.begin();
  Serial.setTimeout(10);
  analogReadResolution(12);
  pinMode(LED_BUILTIN,OUTPUT);
  (void)analogRead(PA0);
  fs=configAdcContinuo(fs);
}

int creaTramaSlipLecturas(uint8_t *buffer,int longitudBuffer);
bool lecturaDisponible(void);

void loop() {
  static uint32_t inicio;
  static uint8_t buffer[128];
  static int posPendiente = 0;
  static int lenPendiente = 0;

  if (CDC_connected() && Serial.availableForWrite() && (lenPendiente || lecturaDisponible())){
    if (!lenPendiente){
      lenPendiente = creaTramaSlipLecturas(buffer,sizeof(buffer));
      posPendiente = 0;
    }
    const int transmitido = Serial.write(buffer+posPendiente,lenPendiente);
    posPendiente += transmitido;
    lenPendiente -= transmitido;
  }else if (!CDC_connected()){
    posPendiente = 0;
    lenPendiente = 0;
  }
  if (millis()-inicio > 500)
  {
    inicio = millis();
    digitalToggle(LED_BUILTIN);
  }
}

#define LBUFF 1024
// Debe ser potencia de dos
static_assert(!((LBUFF-1)&LBUFF));

static volatile struct Fifo
{
  uint16_t buf[LBUFF];
  uint16_t lectura;
  uint16_t escritura;
}fifo;

uint32_t configAdcContinuo(uint32_t fs)
{  
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  // Conversión con disparo por evento TIM1-CC1
  ADC1->CR2 =  (ADC1->CR2 & ~_VAL2FLD(ADC_CR2_EXTSEL,-1))
              |(_VAL2FLD(ADC_CR2_EXTSEL,0))
              |(_VAL2FLD(ADC_CR2_EXTTRIG,1))
              |ADC_CR2_ADON;
  ADC1->CR1 |= ADC_CR1_EOCIE;

  uint32_t ftimer = 2*fs;
  const uint32_t fref = tim1.getTimerClkFreq(); 
  const uint32_t div = fref/ftimer; 
  uint32_t pscDiv = 1;
  uint32_t tmrDiv = div;
  while(tmrDiv > 65536){
    ++pscDiv;
    tmrDiv = div/pscDiv;
  }
  if (!tmrDiv) tmrDiv = 1;
  tim1.setPrescaleFactor(pscDiv);
  tim1.setOverflow(tmrDiv,TICK_FORMAT);
  tim1.setCaptureCompare(1,tmrDiv/2,TICK_COMPARE_FORMAT);
  tim1.setMode(1,TIMER_OUTPUT_COMPARE_TOGGLE,NC);
  tim1.resume();
  NVIC_EnableIRQ(ADC1_2_IRQn);
  ftimer = fref/(tmrDiv*pscDiv);
  return ftimer/2;
}

extern "C"{
  void ADC1_2_IRQHandler(void);
}

void ADC1_2_IRQHandler(void)
{
  const uint16_t dato = ADC1->DR;
  uint16_t escritura = fifo.escritura;
  if (escritura - fifo.lectura < LBUFF){
    fifo.buf[escritura++ % LBUFF] = dato;
    fifo.escritura = escritura;
  }
}

bool lecturaDisponible(void)
{
  return fifo.escritura != fifo.lectura;
}

static int byteEscape(uint8_t x){
  return (x == 0xC || x == 0xDB);
}
static void copiaConEscape(uint8_t *buffer,int &cuenta,uint8_t valor){
  switch (valor)
  {
  case 0xC:
    buffer[cuenta++]=0xDB;
    buffer[cuenta++]=0xDC;
  break; case 0xDB:
    buffer[cuenta++]=0xDB;
    buffer[cuenta++]=0xDD;
  break;default:
    buffer[cuenta++]=valor;
  break;
  }
}
int creaTramaSlipLecturas(uint8_t *buffer,int longitudBuffer)
{
  int cuenta = 0;
  buffer[0] = 0x0C;
  while((cuenta+2) < longitudBuffer && fifo.lectura != fifo.escritura){
    const uint16_t muestra = fifo.buf[(fifo.lectura)%LBUFF];
    const uint8_t muestra_H = (muestra >> 8)&0xFF;
    const uint8_t muestra_L = (muestra & 0xFF);
    const int nBytesRelleno = byteEscape(muestra_H)+byteEscape(muestra_L);
    if (cuenta + 2 + nBytesRelleno >= longitudBuffer) break;
    fifo.lectura++;
    copiaConEscape(buffer,cuenta,muestra_L);
    copiaConEscape(buffer,cuenta,muestra_H);
  }
  buffer[cuenta++]=0x0C;
  return cuenta;
}
#include <Arduino.h>
#include <usbd_cdc_if.h>

uint32_t configAdcContinuo(uint32_t fs);

HardwareTimer tim1(TIM1);

void setup() {
  Serial.begin();
  analogReadResolution(12);
  pinMode(LED_BUILTIN,OUTPUT);
  (void)analogRead(PA0);
  configAdcContinuo(1000);
}

int copiaLecturas(uint16_t *buffer,int longitudBuffer);
bool lecturaDisponible(void);

void loop() {
  static uint32_t inicio;
  static uint16_t buffer[512];

  if (CDC_connected() && Serial.availableForWrite() && lecturaDisponible()){
    const int numLecturas = copiaLecturas(buffer,128);
    Serial.write((uint8_t*)buffer,numLecturas * sizeof(*buffer));
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
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  const uint32_t tfreq = tim1.getTimerClkFreq(); 
  const uint32_t div = tfreq/fs; 
  uint32_t pscDiv = 1;
  uint32_t tmrDiv = div;
  while(tmrDiv > 65536){
    ++pscDiv;
    tmrDiv = div/pscDiv;
  }
  if (!tmrDiv) tmrDiv = 1;
  tim1.setPrescaleFactor(pscDiv);
  tim1.setOverflow(tmrDiv,TICK_FORMAT);
  tim1.setCaptureCompare(1,(tmrDiv-1)/2,TICK_COMPARE_FORMAT);
  tim1.setMode(1,TIMER_OUTPUT_COMPARE_TOGGLE,NC);
  tim1.resume();
  NVIC_EnableIRQ(ADC1_2_IRQn);
  return tfreq/(tmrDiv*pscDiv);
}

extern "C"{
  void ADC1_2_IRQHandler(void);
}

void ADC1_2_IRQHandler(void)
{
  const uint16_t dato = ADC1->DR;
  uint16_t escritura = fifo.escritura;
  fifo.buf[escritura++ % LBUFF] = dato;
  fifo.escritura = escritura;
  if (escritura - fifo.lectura > LBUFF){
    fifo.lectura = escritura - LBUFF;
  }
}

bool lecturaDisponible(void)
{
  return fifo.escritura != fifo.lectura;
}
int copiaLecturas(uint16_t *buffer,int longitudBuffer)
{
  int cuenta = 0;
  while(cuenta < longitudBuffer && fifo.lectura != fifo.escritura){
    const bool primeraPasada = !cuenta;
    if(primeraPasada) __disable_irq();
    buffer[cuenta++] = fifo.buf[(fifo.lectura++)%LBUFF];
    if(primeraPasada) __enable_irq();
  }
  return cuenta;
}
#include "camera.h"
#include "app_config.h"
#include "dcmi.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "ov5640.h"
#include <string.h>
#include <stdio.h>

#if CAMERA_JPEG_BUFFER_SIZE > 0xFFFFU || (CAMERA_JPEG_BUFFER_SIZE % 4U) != 0U
#error "Camera normal DMA buffer must be word-aligned and fit a 16-bit byte count"
#endif

static OV5640_Object_t camera_sensor;
static uint8_t camera_frame[CAMERA_JPEG_BUFFER_SIZE] __attribute__((aligned(32)));
static volatile uint32_t camera_frame_done;
static volatile uint32_t camera_error;

/* PC7 = TIM3_CH2, 160 MHz timer clock / (19 + 1) = 8 MHz. */
static void camera_xclk_stop(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM3->CCER &= ~TIM_CCER_CC2E;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  GPIO_InitTypeDef init = {0};
  init.Pin = GPIO_PIN_7;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &init);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
}

static void camera_xclk_start(void)
{
  GPIO_InitTypeDef init = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  init.Pin = GPIO_PIN_7;
  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &init);
  TIM3->CR1 = 0U;
  TIM3->PSC = 0U;
  TIM3->ARR = 19U;
  TIM3->CCR2 = 10U;
  TIM3->CCMR1 = (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CCER = TIM_CCER_CC2E;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void camera_log(const char *text)
{
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)text, (uint16_t)strlen(text), 1000U);
}

static int32_t camera_tick(void) { return (int32_t)HAL_GetTick(); }
static int32_t camera_bus_init(void) { return 0; }
static int32_t camera_bus_deinit(void) { return 0; }

/* The camera connector is behind two switched 2.8 V pull-ups and a bus
   switch.  Use GPIO open-drain I2C here so the camera probe does not depend
   on the I2C2 peripheral's AF/timing configuration. */
static void camera_i2c_delay(void)
{
#ifdef CAMERA_HOST_TEST
  return;
#else
  static uint8_t dwt_ready;
  if (dwt_ready == 0U) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = 1U;
  }
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = SystemCoreClock / 1000000U * 4U;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
#endif
}

static void camera_i2c_line_init(void)
{
#ifdef CAMERA_HOST_TEST
  return;
#else
  GPIO_InitTypeDef init = {0};
  init.Pin = OV_SCL_Pin | OV_SDA_Pin;
  init.Mode = GPIO_MODE_OUTPUT_OD;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &init);
  HAL_GPIO_WritePin(OV_SCL_GPIO_Port, OV_SCL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  camera_i2c_delay();
#endif
}

static uint8_t camera_i2c_scl_high(void)
{
  uint32_t start = HAL_GetTick();
  HAL_GPIO_WritePin(OV_SCL_GPIO_Port, OV_SCL_Pin, GPIO_PIN_SET);
  while (HAL_GPIO_ReadPin(OV_SCL_GPIO_Port, OV_SCL_Pin) == GPIO_PIN_RESET) {
    if ((HAL_GetTick() - start) > 2U) return 0U;
  }
  camera_i2c_delay();
  return 1U;
}

static void camera_i2c_scl_low(void)
{
  HAL_GPIO_WritePin(OV_SCL_GPIO_Port, OV_SCL_Pin, GPIO_PIN_RESET);
  camera_i2c_delay();
}

static void camera_i2c_start(void)
{
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  (void)camera_i2c_scl_high();
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_RESET);
  camera_i2c_delay();
  camera_i2c_scl_low();
}

static void camera_i2c_stop(void)
{
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_RESET);
  camera_i2c_delay();
  (void)camera_i2c_scl_high();
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  camera_i2c_delay();
}

static uint8_t camera_i2c_write_byte(uint8_t value)
{
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin,
                      (value & 0x80U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    camera_i2c_delay();
    if (camera_i2c_scl_high() == 0U) return 0U;
    camera_i2c_scl_low();
    value <<= 1;
  }
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  if (camera_i2c_scl_high() == 0U) return 0U;
  uint8_t ack = HAL_GPIO_ReadPin(OV_SDA_GPIO_Port, OV_SDA_Pin) == GPIO_PIN_RESET;
  camera_i2c_scl_low();
  return ack;
}

static uint8_t camera_i2c_read_byte(uint8_t ack)
{
  uint8_t value = 0U;
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    value <<= 1;
    if (camera_i2c_scl_high() == 0U) return 0U;
    if (HAL_GPIO_ReadPin(OV_SDA_GPIO_Port, OV_SDA_Pin) != GPIO_PIN_RESET) value |= 1U;
    camera_i2c_scl_low();
  }
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin,
                    ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
  if (camera_i2c_scl_high() == 0U) return value;
  camera_i2c_scl_low();
  HAL_GPIO_WritePin(OV_SDA_GPIO_Port, OV_SDA_Pin, GPIO_PIN_SET);
  return value;
}

static uint8_t camera_i2c_probe(uint8_t address7)
{
  camera_i2c_start();
  uint8_t ack = camera_i2c_write_byte((uint8_t)(address7 << 1));
  camera_i2c_stop();
  return ack;
}

static int camera_i2c_write(uint8_t address7, uint16_t reg,
                            const uint8_t *data, uint16_t length)
{
  camera_i2c_start();
  if (!camera_i2c_write_byte((uint8_t)(address7 << 1)) ||
      !camera_i2c_write_byte((uint8_t)(reg >> 8)) ||
      !camera_i2c_write_byte((uint8_t)reg)) {
    camera_i2c_stop();
    return -1;
  }
  for (uint16_t i = 0U; i < length; ++i) {
    if (!camera_i2c_write_byte(data[i])) {
      camera_i2c_stop();
      return -1;
    }
  }
  camera_i2c_stop();
  return 0;
}

static int camera_i2c_read(uint8_t address7, uint16_t reg,
                           uint8_t *data, uint16_t length)
{
  camera_i2c_start();
  if (!camera_i2c_write_byte((uint8_t)(address7 << 1)) ||
      !camera_i2c_write_byte((uint8_t)(reg >> 8)) ||
      !camera_i2c_write_byte((uint8_t)reg)) {
    camera_i2c_stop();
    return -1;
  }
  camera_i2c_start();
  if (!camera_i2c_write_byte((uint8_t)((address7 << 1) | 1U))) {
    camera_i2c_stop();
    return -1;
  }
  for (uint16_t i = 0U; i < length; ++i) {
    data[i] = camera_i2c_read_byte(i + 1U < length ? 1U : 0U);
  }
  camera_i2c_stop();
  return 0;
}

static int32_t camera_write_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
  if (camera_i2c_write((uint8_t)(address >> 1), reg, data, length) != 0) return -1;
  if (reg == 0x3008U && length != 0U && (data[0] & 0x80U) != 0U) HAL_Delay(10U);
  return 0;
}

static int32_t camera_read_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
  return camera_i2c_read((uint8_t)(address >> 1), reg, data, length);
}

/* All diagnostic reads use individual addresses so there is no dependency
   on SCCB sequential-read behaviour. These values are not a lock detector. */
void camera_log_state(const char *phase)
{
  static const uint16_t regs[] = {
    0x5000,0x5001,0x3406,0x3503,0x503D,0x5580,0x5583,0x5584,0x5588,
    0x3400,0x3401,0x3402,0x3403,0x3404,0x3405,
    0x519F,0x51A0,0x51A1,0x51A2,0x51A3,0x51A4,0x51A5,0x51A6,0x51A7,
    0x3500,0x3501,0x3502,0x350A,0x350B,0x3034,0x3035,0x3036,0x3037,0x3108
  };
  uint8_t v[sizeof(regs) / sizeof(regs[0])];
  char line[180];
  for (unsigned i = 0; i < sizeof(v); ++i) {
    if (camera_read_reg(0x78U, regs[i], &v[i], 1U) != 0) {
      (void)snprintf(line, sizeof(line), "CAM state %s read failed reg=%04X\r\n", phase, regs[i]);
      camera_log(line);
      return;
    }
  }
  (void)snprintf(line, sizeof(line),
    "CAM state %s ISP=%02X/%02X WB=%02X AE=%02X BAR=%02X SDE=%02X U/V=%02X/%02X UVCTL=%02X\r\n",
    phase,v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]);
  camera_log(line);
  (void)snprintf(line, sizeof(line),
    "CAM WB set=%04X/%04X/%04X calc R=%u G=%u B=%u avgRGB=%u/%u/%u exposure16=%lu gain16=%u\r\n",
    ((uint16_t)v[9]<<8)|v[10], ((uint16_t)v[11]<<8)|v[12], ((uint16_t)v[13]<<8)|v[14],
    ((v[15]&15U)<<8)|v[16], ((v[17]&15U)<<8)|v[18], ((v[19]&15U)<<8)|v[20],
    v[21],v[22],v[23], (unsigned long)(((v[24]&15UL)<<16)|((uint32_t)v[25]<<8)|v[26]),
    ((v[27]&3U)<<8)|v[28]);
  camera_log(line);
  (void)snprintf(line, sizeof(line), "CAM PLL 3034..37=%02X/%02X/%02X/%02X root=%02X\r\n",
    v[29],v[30],v[31],v[32],v[33]);
  camera_log(line);
}

static int camera_update_checked(uint16_t reg, uint8_t mask, uint8_t value)
{
  uint8_t old_value, actual;
  if (camera_read_reg(0x78U, reg, &old_value, 1U) != 0) return -1;
  uint8_t expected = (old_value & (uint8_t)~mask) | (value & mask);
  if (camera_write_reg(0x78U, reg, &expected, 1U) != 0 ||
      camera_read_reg(0x78U, reg, &actual, 1U) != 0) return -1;
  if ((actual & mask) != (expected & mask)) {
    char line[96];
    (void)snprintf(line, sizeof(line), "CAM config mismatch reg=%04X want=%02X got=%02X\r\n",
                   reg, expected, actual);
    camera_log(line);
    return -1;
  }
  return 0;
}

void camera_power_off(void)
{
  board_set_stage(BOARD_STAGE_CAMERA_OFF);
  camera_xclk_stop();
  HAL_GPIO_WritePin(OVRESET_EN_GPIO_Port, OVRESET_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OV_RESET_GPIO_Port, OV_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(I2C_EN_GPIO_Port, I2C_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAM_A_EN_GPIO_Port, CAM_A_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAM_B_EN_GPIO_Port, CAM_B_EN_Pin, GPIO_PIN_RESET);
  memset(&camera_sensor, 0, sizeof(camera_sensor));
  camera_log("CAM rails off\r\n");
  board_set_stage(BOARD_STAGE_IDLE);
}

static void camera_power_on(CameraPort port)
{
  /* Remove power and shared signals before changing the camera multiplexer. */
  camera_power_off();
  HAL_Delay(20U);
  /* U6.29/U6.30 are the camera-path select inputs.  The schematic labels
     LOW=CAM_A and HIGH=CAM_B; both inputs are tied to CAM_DATA_CTRL. */
  HAL_GPIO_WritePin(CAM_DATA_CTRL_GPIO_Port, CAM_DATA_CTRL_Pin,
                    port == CAMERA_B ? GPIO_PIN_SET : GPIO_PIN_RESET);
  camera_log(port == CAMERA_B ? "CAM select B\r\n" : "CAM select A\r\n");

  /* Match the original oscillator sequence: provide a stable XCLK while
     RESET is low, then power the sensor and release reset. */
  board_set_stage(BOARD_STAGE_CAMERA_CLOCK);
  camera_xclk_start();
  camera_log("CAM clock PC7 8MHz on\r\n");
  HAL_Delay(5U);
  camera_log("CAM rails enabling\r\n");
  board_set_stage(BOARD_STAGE_CAMERA_RAILS);
  if (port == CAMERA_B)
    HAL_GPIO_WritePin(CAM_B_EN_GPIO_Port, CAM_B_EN_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(CAM_A_EN_GPIO_Port, CAM_A_EN_Pin, GPIO_PIN_SET);
  camera_log("CAM rails on\r\n");
  HAL_Delay(5U);
  HAL_GPIO_WritePin(I2C_EN_GPIO_Port, I2C_EN_Pin, GPIO_PIN_SET);
  camera_log("CAM pullups on\r\n");
  /* Release the open-drain MCU side before enabling U22, which drives the
     2.8 V reset-high level for both camera modules. */
  HAL_GPIO_WritePin(OV_RESET_GPIO_Port, OV_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OVRESET_EN_GPIO_Port, OVRESET_EN_Pin, GPIO_PIN_SET);
  camera_log("CAM reset released\r\n");
  HAL_Delay(50U);
  camera_log("CAM power done\r\n");
}

int camera_init_resolution(CameraPort port, uint32_t resolution)
{
  OV5640_IO_t io = {0};
  uint32_t id = 0U;
  uint8_t ready;
  if (port != CAMERA_A && port != CAMERA_B) return -12;
  camera_log("CAM power\r\n");
  camera_power_on(port);
  board_set_stage(BOARD_STAGE_CAMERA_CONFIG);
  /* A failed transaction on the other sensor must not carry over to this one. */
  if (HAL_I2C_DeInit(&hi2c2) != HAL_OK) return -13;
  camera_i2c_line_init();
  camera_log("CAM I2C probe\r\n");
  ready = camera_i2c_probe(0x3CU);
  if (ready == 0U) {
    {
      char bus_text[80];
      int count = snprintf(bus_text, sizeof(bus_text),
                           "CAM soft I2C no ACK SCL=%u SDA=%u\r\n",
                           (unsigned)HAL_GPIO_ReadPin(OV_SCL_GPIO_Port, OV_SCL_Pin),
                           (unsigned)HAL_GPIO_ReadPin(OV_SDA_GPIO_Port, OV_SDA_Pin));
      if (count > 0) {
        (void)HAL_UART_Transmit(&huart4, (uint8_t *)bus_text,
                                (uint16_t)count, 1000U);
      }
    }
    if (camera_i2c_probe(0x30U) != 0U) {
      camera_log("CAM OV2640 ACK 0x60; driver not loaded\r\n");
      return -11;
    }
    camera_log("CAM I2C no ACK\r\n");
    return -10;
  }
  io.Init = camera_bus_init;
  io.DeInit = camera_bus_deinit;
  io.Address = 0x78U; /* OV5640 7-bit address 0x3C, HAL uses shifted address. */
  io.WriteReg = camera_write_reg;
  io.ReadReg = camera_read_reg;
  io.GetTick = camera_tick;
  if (OV5640_RegisterBusIO(&camera_sensor, &io) != OV5640_OK) return -1;
  camera_log("CAM read ID\r\n");
  if (OV5640_ReadID(&camera_sensor, &id) != OV5640_OK) return -2;
  if (id != 0x5640U) return -3;
  camera_log("CAM configure\r\n");
  if (OV5640_Init(&camera_sensor, resolution, OV5640_JPEG) != OV5640_OK) return -4;
  camera_log("CAM preset configured; PLL start\r\n");
  /* XCLK is 8 MHz from PC7, while the ST helper assumes a 24 MHz
     input. Keep the same internal/PCLK rates by multiplying 3x:
     0x3036=0xC0 with the 0x3037=0x16 divider gives the same
     clock tree as the original 24 MHz + 0x40/0x16 setting. */
  {
    uint8_t pll_mult = 0xC0U;
    uint8_t pll_div = 0x16U;
    if (camera_write_reg(0x78U, 0x3036U, &pll_mult, 1U) != 0 ||
        camera_write_reg(0x78U, 0x3037U, &pll_div, 1U) != 0) return -5;
  }
  camera_log("CAM PLL configured; ISP start\r\n");
  if (OV5640_SetLightMode(&camera_sensor, APP_CAMERA_WB_MODE) != OV5640_OK) return -16;
  /* AWB algorithm enable (5001.0) and gain source (3406.0) are separate.
     Reassert both, plus auto exposure/gain, and check the actual sensor. */
  if (camera_update_checked(0x5001U, 0x03U, 0x03U) != 0 ||
      camera_update_checked(0x3406U, 0x01U, APP_CAMERA_WB_MODE ? 1U : 0U) != 0 ||
      camera_update_checked(0x3503U, 0x03U, 0U) != 0 ||
      camera_update_checked(0x5580U, 0xFBU, 0x02U) != 0 ||
      camera_update_checked(0x503DU, 0xFFU, 0U) != 0) return -16;
#if APP_CAMERA_FIXED_UV
  /* Datasheet 5.10.1: equal U/V gain is meaningful ONLY with manual UV. */
  if (camera_update_checked(0x5583U, 0xFFU, 0x40U) != 0 ||
      camera_update_checked(0x5584U, 0xFFU, 0x40U) != 0 ||
      camera_update_checked(0x5588U, 0x40U, 0x40U) != 0) return -16;
#endif
  {
    uint8_t quality = 24U;
    if (camera_write_reg(0x78U, 0x4407U, &quality, 1U) != 0) return -7;
  }
#if APP_CAMERA_COLORBAR_TEST
  if (OV5640_ColorbarModeConfig(&camera_sensor, COLORBAR_MODE_ENABLE) != OV5640_OK) return -14;
  {
    uint8_t pattern = 0U;
    if (camera_read_reg(0x78U, OV5640_PRE_ISP_TEST_SETTING1, &pattern, 1U) != 0 ||
        pattern != 0x80U) return -15;
  }
  camera_log("CAM color bars ON (503D=80)\r\n");
#endif
  camera_log("CAM configured\r\n");
  HAL_Delay(1500U);
  camera_log_state("init-settled");
  return 0;
}

int camera_init_dimensions(CameraPort port, uint16_t width, uint16_t height)
{
  int32_t resolution = OV5640_ResolutionForSize(width, height);
  if (resolution < 0 || (width & 15U) != 0U || (height & 15U) != 0U) {
    camera_log("CAM unsupported size: use an SSDV-aligned preset\r\n");
    return -1;
  }
  /* Use exactly the preset path used by selftest, with no second timing rewrite. */
  return camera_init_resolution(port, (uint32_t)resolution);
}

int camera_init(CameraPort port)
{
  return camera_init_resolution(port, OV5640_R640x480);
}

static int camera_stop_capture(void)
{
  HAL_StatusTypeDef stopped;
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);
  stopped = HAL_DCMI_Stop(&hdcmi);
  /* This HAL's DCMI_Stop ignores the return value from DMA_Abort. Do not
     reuse or expose the buffer while DMA can still write to it. */
  if (hdma_dcmi.State != HAL_DMA_STATE_READY &&
      HAL_DMA_Abort(&hdma_dcmi) != HAL_OK) return -19;
  __DSB();
  return stopped == HAL_OK && hdma_dcmi.State == HAL_DMA_STATE_READY ? 0 : -19;
}

static int camera_capture_impl(uint8_t **jpeg, uint32_t *length,
                               CameraCapturePollFn poll, void *context)
{
  uint32_t remaining_bytes;
  uint32_t transferred;
  uint32_t start;
  int stopped;
  if (jpeg == NULL || length == NULL) return -22;
  *jpeg = NULL;
  *length = 0U;
  if (hdcmi.Instance == NULL || hdma_dcmi.Instance == NULL ||
      hdcmi.DMA_Handle != &hdma_dcmi) return -21;
  /* A failed/aborted capture can leave either HAL handle busy.  Clear that
     state before arming the next camera, especially after switching A/B. */
  if (hdma_dcmi.State != HAL_DMA_STATE_READY || hdcmi.State != HAL_DCMI_STATE_READY) {
    if (camera_stop_capture() != 0) {
      camera_log("CAM previous capture could not stop; buffer not reused\r\n");
      return -19;
    }
  }
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI);
  memset(camera_frame, 0, sizeof(camera_frame));
  camera_frame_done = 0U;
  camera_error = 0U;
  board_set_stage(BOARD_STAGE_CAPTURE);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);
  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)camera_frame,
                         sizeof(camera_frame) / 4U) != HAL_OK) {
    char line[96];
    (void)snprintf(line, sizeof(line),
                   "CAM DCMI start failed dcmi=0x%08lX dma=0x%08lX\r\n",
                   (unsigned long)hdcmi.ErrorCode,
                   (unsigned long)hdma_dcmi.ErrorCode);
    camera_log(line);
    if (camera_stop_capture() != 0) return -19;
    return -1;
  }
  start = HAL_GetTick();
  while (!camera_frame_done && !camera_error) {
    if (poll != NULL) poll(context);
    HAL_Delay(1U);
    if ((HAL_GetTick() - start) > 5000U) break;
  }
  HAL_Delay(2U);
  /* GPDMA CBR1/GET_COUNTER is expressed in bytes for this HAL. */
  remaining_bytes = __HAL_DMA_GET_COUNTER(&hdma_dcmi);
  stopped = camera_stop_capture();
  if (stopped != 0) {
    camera_log("CAM DMA stop failed; JPEG not released\r\n");
    return stopped;
  }
  if (remaining_bytes > sizeof(camera_frame)) {
    camera_log("CAM invalid DMA remaining count\r\n");
    return -18;
  }
  transferred = sizeof(camera_frame) - remaining_bytes;
  if ((HAL_GetTick() - start) > 5000U || !camera_frame_done ||
      camera_error != 0U || remaining_bytes == sizeof(camera_frame)) {
    char line[128];
    (void)snprintf(line, sizeof(line),
                   "CAM capture failed elapsed=%lu done=%lu error=0x%08lX remaining=%lu\r\n",
                   (unsigned long)(HAL_GetTick() - start), (unsigned long)camera_frame_done,
                   (unsigned long)camera_error, (unsigned long)remaining_bytes);
    camera_log(line);
    if (remaining_bytes == 0U) {
      camera_log("CAM JPEG buffer full: capacity=65532 bytes\r\n");
      return -17;
    }
    return -2;
  }
  for (uint32_t begin = 0U; begin + 1U < transferred && begin < 32U; ++begin) {
    if (camera_frame[begin] != 0xFFU || camera_frame[begin + 1U] != 0xD8U) continue;
    for (uint32_t end = begin + 2U; end + 1U < transferred; ++end) {
      if (camera_frame[end] == 0xFFU && camera_frame[end + 1U] == 0xD9U) {
        *jpeg = &camera_frame[begin];
        *length = end + 2U - begin;
        return 0;
      }
    }
    break;
  }
  if (remaining_bytes == 0U) {
    camera_log("CAM JPEG buffer full: capacity=65532 bytes\r\n");
    return -17;
  }
  return -3;
}

int camera_capture(uint8_t **jpeg, uint32_t *length)
{
  return camera_capture_impl(jpeg, length, NULL, NULL);
}

int camera_capture_with_poll(uint8_t **jpeg, uint32_t *length,
                             CameraCapturePollFn poll, void *context)
{
  return camera_capture_impl(jpeg, length, poll, context);
}

int camera_send_jpeg(const uint8_t *jpeg, uint32_t length)
{
  char header[40];
  int count = snprintf(header, sizeof(header), "JPEG %lu\r\n", (unsigned long)length);
  if (count <= 0 || (uint32_t)count >= sizeof(header)) return -1;
  if (HAL_UART_Transmit(&huart4, (uint8_t *)header, (uint16_t)count, 1000U) != HAL_OK) return -2;
  while (length != 0U) {
    uint16_t chunk = length > 1024U ? 1024U : (uint16_t)length;
    if (HAL_UART_Transmit(&huart4, (uint8_t *)jpeg, chunk, 2000U) != HAL_OK) return -3;
    jpeg += chunk;
    length -= chunk;
  }
  return 0;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *dcmi)
{
  (void)dcmi;
  camera_frame_done = 1U;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *dcmi)
{
  camera_error = dcmi->ErrorCode != 0U ? dcmi->ErrorCode : 0x80000000UL;
}

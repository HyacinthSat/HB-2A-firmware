/* Compile the actual camera driver against a deterministic HAL, no hardware. */
#define CAMERA_HOST_TEST
#include "../Target/Core/Src/camera.c"
#include <assert.h>

TIM_TypeDef mock_tim;
GPIO_TypeDef mock_gpio;
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
UART_HandleTypeDef huart4;
I2C_HandleTypeDef hi2c2;
uint32_t mock_remaining;
static uint32_t ticks, starts, stops, aborts;
static int stop_fails, abort_fails, frame_missing, no_eoi, start_fails, check_old_buffer;
static BoardStage stage;

void board_set_stage(BoardStage value) { stage = value; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint32_t pin, GPIO_PinState s) { (void)p; (void)pin; (void)s; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint32_t pin) { (void)p; (void)pin; return GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t)
{ (void)h; (void)d; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_Delay(uint32_t n) { ticks += n; }
uint32_t HAL_GetTick(void) { return ticks; }
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h)
{
  ++aborts;
  if (abort_fails) return HAL_TIMEOUT;
  h->State = HAL_DMA_STATE_READY;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef *h)
{
  ++stops;
  if (check_old_buffer) {
    assert(camera_frame[0] == 0xA5); /* Must stop DMA before memset. */
    check_old_buffer = 0;
  }
  (void)HAL_DMA_Abort(h->DMA_Handle); /* Match HAL: ignores abort status. */
  h->State = HAL_DCMI_STATE_READY;
  return stop_fails ? HAL_TIMEOUT : HAL_OK;
}
HAL_StatusTypeDef HAL_DCMI_Start_DMA(DCMI_HandleTypeDef *h, uint32_t mode, uint32_t address, uint32_t words)
{
  (void)address;
  assert(mode == DCMI_MODE_SNAPSHOT && words * 4U == sizeof(camera_frame));
  ++starts;
  h->State = HAL_DCMI_STATE_BUSY;
  h->DMA_Handle->State = HAL_DMA_STATE_BUSY;
  if (start_fails) return HAL_ERROR;
  camera_frame[0] = 0xFF; camera_frame[1] = 0xD8;
  if (!no_eoi) { camera_frame[6] = 0xFF; camera_frame[7] = 0xD9; }
  camera_frame_done = !frame_missing;
  return HAL_OK;
}
static void reset(void)
{
  ticks = starts = stops = aborts = 0;
  stop_fails = abort_fails = frame_missing = no_eoi = start_fails = check_old_buffer = 0;
  hdcmi.Instance = &hdcmi;
  hdma_dcmi.Instance = &hdma_dcmi;
  hdcmi.DMA_Handle = &hdma_dcmi;
  hdcmi.State = HAL_DCMI_STATE_READY;
  hdma_dcmi.State = HAL_DMA_STATE_READY;
  mock_remaining = sizeof(camera_frame) - 8U;
}
static void expect_failure(int expected)
{
  uint8_t *jpeg = camera_frame;
  uint32_t length = 999U;
  assert(camera_capture(&jpeg, &length) == expected);
  assert(jpeg == NULL && length == 0U);
}
int main(void)
{
  uint8_t *jpeg;
  uint32_t length;
  reset();
  assert(camera_capture(NULL, &length) == -22 && starts == 0);
  assert(camera_capture(&jpeg, NULL) == -22 && starts == 0);
  hdcmi.DMA_Handle = NULL;
  expect_failure(-21);
  assert(starts == 0);
  reset();
  assert(camera_capture(&jpeg, &length) == 0 && length == 8U && jpeg == camera_frame);
  assert(stops == 1 && hdma_dcmi.State == HAL_DMA_STATE_READY && stage == BOARD_STAGE_CAPTURE);
  reset();
  mock_remaining = sizeof(camera_frame) + 1U;
  expect_failure(-18); /* Prevent unsigned underflow and out-of-bounds JPEG scan. */
  reset();
  abort_fails = 1;
  expect_failure(-19); /* DCMI_Stop returning OK must not hide live DMA. */
  reset();
  hdcmi.State = HAL_DCMI_STATE_BUSY;
  hdma_dcmi.State = HAL_DMA_STATE_BUSY;
  camera_frame[0] = 0xA5;
  check_old_buffer = abort_fails = 1;
  expect_failure(-19);
  assert(starts == 0 && camera_frame[0] == 0xA5);
  reset();
  hdcmi.State = HAL_DCMI_STATE_BUSY;
  hdma_dcmi.State = HAL_DMA_STATE_BUSY;
  camera_frame[0] = 0xA5;
  check_old_buffer = 1;
  assert(camera_capture(&jpeg, &length) == 0 && stops == 2);
  reset(); stop_fails = 1; expect_failure(-19);
  reset(); start_fails = 1; expect_failure(-1); assert(stops == 1);
  reset(); frame_missing = 1; expect_failure(-2); assert(ticks < 5010U && stops == 1);
  reset(); no_eoi = 1; expect_failure(-3);
  reset(); no_eoi = 1; mock_remaining = 0; expect_failure(-17);
  reset(); mock_remaining = sizeof(camera_frame); expect_failure(-2);
  /* Recovery must remain usable over repeated capture cycles. */
  for (unsigned i = 0; i < 100; ++i) {
    reset();
    assert(camera_capture(&jpeg, &length) == 0 && length == 8U);
  }
  puts("Camera capture failure/recovery tests passed");
  return 0;
}

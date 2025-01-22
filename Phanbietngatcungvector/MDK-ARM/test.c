#include "stm32f1xx.h"


// Hàm x? lý ng?t cho PA10
void EXTI10_IRQHandler(void) {
  // Xóa c? ng?t
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  // B?t LED trên PB0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

// Hàm x? lý ng?t cho PB10 (n?m trong nhóm ng?t EXTI15_10_IRQn)
void EXTI15_10_IRQHandler(void) {
  // Ki?m tra xem ng?t d?n t? chân nào
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
    // Ng?t d?n t? PB10, b?t LED trên PB1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  }
  // Xóa c? ng?t
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

int main(void) {

  while (1) {
  }
}

// Hàm c?u hình GPIO
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Ng?t khi m?c logic chuy?n t? cao xu?ng th?p
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
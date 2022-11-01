#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#ifndef SIMULATOR
#include "main.h"
#include "cmsis_os.h"
extern osSemaphoreId_t KeyPress_Sema_FromISRHandle;
#endif

Model::Model() : modelListener(0),Minute(0),TickCount(0),ClockState(0)  //在外部定义的构造函数
{
}

void Model::tick()
{
#ifndef SIMULATOR
	if(osSemaphoreAcquire(KeyPress_Sema_FromISRHandle,0)==osOK)
	{
		modelListener->ChangeNumber();
	}
#endif
	if(TickCount==3600)
	{
		Minute++;
		Minute %= 60;
		TickCount = 0;
	}
	if(ClockState)
	{
		TickCount++;
	}
}

void Model::toggleLED()
{
#ifndef SIMULATOR
	HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_12);
#endif
}

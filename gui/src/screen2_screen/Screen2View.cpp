#include <gui/screen2_screen/Screen2View.hpp>

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::handleTickEvent()
{
		minute = presenter->GetMinute();
		tickCount = (presenter->GetTickCount())/60;
		Unicode::snprintf(textArea1Buffer1, TEXTAREA1BUFFER1_SIZE, "%02d", minute);
		Unicode::snprintf(textArea1Buffer2, TEXTAREA1BUFFER2_SIZE, "%02d", tickCount);
		textArea1.invalidate();
}

void Screen2View::SetNumber()
{
//
}

void Screen2View::ButtonClickFunction()
{

	presenter->ButtonClick();
}

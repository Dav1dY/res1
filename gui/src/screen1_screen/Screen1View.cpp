#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::ButtonClickFunc()
{
	presenter->ButtonClick();
}

void Screen1View::SetNumber()
{
	count++;
	Unicode::snprintf(textArea1Buffer,TEXTAREA1_SIZE,"%d",count);
	textArea1.resizeToCurrentText();
	textArea1.invalidate();
}

#ifndef MODEL_HPP
#define MODEL_HPP

class ModelListener;

class Model
{
public:
    Model(); //构造函数
    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }
    void tick();
    void toggleLED();

    int GetMinute()
    {
    	return Minute;
    }
    int GetTickCount()
    {
    	return TickCount;
    }
    void StartButton()
    {
    	if(!ClockState)
    	{
    		ClockState=1;
    	}
    	else
    	{
    		ClockState=0;
    	}
    }

protected:
    ModelListener* modelListener;
    int Minute;
    int TickCount;
    int ClockState;
};

#endif // MODEL_HPP

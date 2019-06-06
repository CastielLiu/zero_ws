#ifndef KEY_EVENT_H_
#define KEY_EVENT_H_
#include <unistd.h>
#include<cmath>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>


class KeyEvent
{
public:
	enum mode_t
	{
		ONLY_KEYBORD = 0,
		ONLY_MOUSE = 1,
		KEYBORD_AND_MOUSE =2
	};
	
	enum keyCode_t
	{
		KEY_Enter = 28,
	};
	
	enum key_status_t
	{
		KEY_Up = 0,
		KEY_Down =1,
	};
	
public:
	KeyEvent():
		keys_fd(-1),
		mouse_fd(-1){}
		
	~KeyEvent()
	{
		if(keys_fd!=-1)
		{
			close(keys_fd);
			keys_fd = -1;
		}
		if(mouse_fd !=-1)
		{
			close(mouse_fd);
			mouse_fd = -1;
		}
	}
	
	bool init(mode_t mode)
	{
		mode_ = mode;
		
		if(ONLY_KEYBORD==mode || KEYBORD_AND_MOUSE==mode)
		{
			keys_fd=open("/dev/input/event1",O_RDONLY & ~O_NONBLOCK);
			if(keys_fd<=0)
			{
				printf("open /dev/input/event1 failed !\n");
				return false;
			}
			printf("open /dev/input/event1 ok !\n");
		}
		if(ONLY_MOUSE==mode || KEYBORD_AND_MOUSE==mode)
		{
			mouse_fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);
			if(mouse_fd<=0)
			{
				printf("open /dev/input/event2 failed !\n");
				return false;
			}
			printf("open /dev/input/event2 ok !\n");
		}
		return true;
	}
	
	bool keyMonitor(const keyCode_t code, key_status_t status)
	{
		bool is_ok = false;
		if(ONLY_KEYBORD==mode_ || KEYBORD_AND_MOUSE==mode_)
			read(keys_fd,&key_event,sizeof(struct input_event));
		if(ONLY_MOUSE==mode_ || KEYBORD_AND_MOUSE==mode_)
			read(mouse_fd,&mouseEvent,sizeof(struct input_event));
		
		if(code == key_event.code && status == key_event.value)
			is_ok = true;
		else if(code == mouseEvent.code && status == mouseEvent.value)
			is_ok = true;
			
		return is_ok;
	}
	
private:
	int keys_fd ,mouse_fd;
	mode_t mode_;
	struct input_event key_event,mouseEvent;
	
};

#endif



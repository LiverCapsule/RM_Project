#include "BeepTask.h"
#include "config.h"





#define MUSIC_LENGTH(x)  (sizeof(x)/sizeof(x[0]))





const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

const Sound_tone_e Do_do[7] =
{
	Do1L,Re2L,Mi3L,Fa4L,So5L,La6L,Si7L 
};


const Sound_tone_e No_Music[7] =
{
	Silent,Silent,Silent,Silent,Silent,Silent,Silent
};

const Sound_tone_e Startup_music[] = 
{
  Do1H, Mi3H, So5H, So5H, So5H,  Silent,Silent,Silent,Silent
};

uint16_t aaaaa = 0;


//播放单个音符
void Sing(Sound_tone_e tone)
{
  if(tone == Silent)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}



void Sing_Music(const Sound_tone_e *music, uint8_t length)
{
	static int i = 0;
	if(i < length)
	{
		Sing(music[i++]);
	}
	else
	{
		i = 0;
	}
}




void BeepTask(void)
{
	static uint32_t time = 0;//突然知道为什么老是用static
	if((xTaskGetTickCount() -time > 150000))		//蜂鸣器启动音乐 150ms周期更换音符
	{
		time = xTaskGetTickCount();
		if(xTaskGetTickCount() < 1500000)						//开机音乐
		{																					//逻辑想好
			Sing_Music(Startup_music,MUSIC_LENGTH(Startup_music));
		}	
		else
		{
			Sing_Music(No_Music,MUSIC_LENGTH(No_Music));
		}
	}

}




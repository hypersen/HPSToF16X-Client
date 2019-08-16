//////////////////////////////////////////////////////////////////////////////////	 
//闁哄牜鍓涢埢鍏兼償韫囨挸娑у〒姘☉椤掔喐绋婇悩鍗炩枏闁汇埄鐓夌槐婵嬪嫉椤忓棛鐥呭ù锝嗙矎閿熺晫鎷嬬粙鍨闁挎稑濂旂粭澶婎嚗濡ゅ啯鏆忓ù婊冮閸欏墽锟介崘褎宕插ù锝嗘礈閺併倝鏌呴敓锟�/濞戞搩鍘藉▍娆撳炊椤撶姵鏆╅悗娑虫嫹
//閹煎瓨顨￠幗鐢稿捶閺夋寧绲婚柨娑欘劇ttp://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  闁哄偊鎷峰ù鐙呮嫹闁告熬鎷� : main.c
//  闁绘鎷烽柡鍫嫹闁告瑱鎷� : v2.0
//  濞达綇鎷�  闁煎府鎷� : HuangKai
//  闁汇垻鍠愰崹姘跺籍閵夛附鍩�  : 2014-0101
//  闁哄牞鎷风换搴㈢┍椤旇姤鏆�  :
//  闁告梻鍠曢崗姗�箵韫囨艾鐗�  : OLED 4闁规亽鍎辫ぐ娑橆煶閺冨倶浠涘〒姘儑閳伙拷51缂侇垵顕ч崹锟�
//              閻犲洤鐡ㄥΣ锟�
//              ----------------------------------------------------------------
//              GND    闁汇垹鐏氱花顕�捶閿燂拷//              VCC  闁圭尨鎷稸闁硅揪鎷�3v闁汇垹鐏氱花锟�//              D0   P1^0闁挎稑婀疌L闁挎冻鎷�//              D1   P1^1闁挎稑婀疍A闁挎冻鎷�
//              ----------------------------------------------------------------
// 濞ｅ浂鍠楅弫濂稿储閸℃钑�  :
// 闁哄喛鎷�  闁哄牞鎷� :
// 濞达綇鎷�  闁煎府鎷� : HuangKai
// 濞ｅ浂鍠楅弫濂稿礃閸涱収鍟�  : 闁告帗绋戠紓鎾诲棘閸ワ附顐�
//闁绘鐗婂鍫ュ箥閿熻姤绠掗柨娑樼灱濞插秹鎮ч崼婵堢畱缂佸矁缈伴敓锟�//Copyright(C) 濞戞搩鍘藉▍娆撳炊椤撶姵鏆╅悗娑虫嫹014/3/16
//All rights reserved
//******************************************************************************/

#if 1
//#include "REG51.h"
#ifndef __OLED_H
#define __OLED_H			  	 

#include <stdint.h>

#define OLED_CMD  0	//鍐欏懡浠�
#define OLED_DATA 1	//鍐欐暟鎹�
#define OLED_MODE 0


/*sbit OLED_SCL=P0^1;//闁哄啫鐖奸幐锟紻0闁挎稑婀疌LK闁跨噦鎷�
sbit OLED_SDIN=P0^0;//D1闁挎稑婀SI闁挎冻鎷烽柡浣哄瀹擄拷

#define OLED_CS_Clr()  OLED_CS=0
#define OLED_CS_Set()  OLED_CS=1

#define OLED_RST_Clr() OLED_RST=0
#define OLED_RST_Set() OLED_RST=1

#define OLED_DC_Clr() OLED_DC=0
#define OLED_DC_Set() OLED_DC=1

#define OLED_SCLK_Clr() OLED_SCL=0
#define OLED_SCLK_Set() OLED_SCL=1

#define OLED_SDIN_Clr() OLED_SDIN=0
#define OLED_SDIN_Set() OLED_SDIN=1*/





//OLED婵☆垪锟界槐锛勬媼閸撗呮瀭
//0:4缂佹儳銇樼憰鍡欐偘鐏炵伕浣割嚕閿燂拷//1:妤犵偞鍎奸、锟�80婵☆垪锟界槐锟�
#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED缂佹棏鍨拌ぐ娑氾拷濮橆偆鐤�---------------

void delay_ms(unsigned int ms);


void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void fill_picture(unsigned char fill_Data);
void Picture(void);
void IIC_Start(void);
void IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);

#endif  
	 
#endif


